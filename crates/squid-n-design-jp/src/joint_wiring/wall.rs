//! 耐震壁（Wall 要素 × RcWall 形状）のせん断検定配線。

use super::common::{rc_dt, ForcesAt, MemberInfo};
use crate::rc::wall::{rc_wall_shear_check, RcWallInput, WallSideColumn};
use crate::rc::wall_nonlinear::{wall_shear_trilinear, WallShearTrilinearInput};
use crate::wall_opening::{equivalent_opening, is_seismic_wall, opening_ratio_r0, WallJudgeInput};
use crate::{CheckComponent, CheckKind, CheckResult, LoadTerm};
use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::{ElementKind, Model};
use squid_n_core::section_shape::SectionShape;

/// 耐震壁（Wall 要素 × RcWall 形状）のせん断検定を一括で `out` へ追加する。
pub(super) fn check_walls(
    model: &Model,
    member_forces: &[(ElemId, ForcesAt<'_>)],
    members: &[MemberInfo<'_>],
    term: LoadTerm,
    out: &mut Vec<(NodeId, String, CheckResult)>,
) {
    // ── 耐震壁（Wall 要素 × RcWall 形状） ────────────────────────
    for (eid, forces) in member_forces {
        let Some(elem) = model.elements.iter().find(|e| e.id == *eid) else {
            continue;
        };
        if elem.kind != ElementKind::Wall {
            continue;
        }
        let Some(sec) = elem
            .section
            .and_then(|sid| model.sections.iter().find(|s| s.id == sid))
        else {
            continue;
        };
        let Some(SectionShape::RcWall { thickness, ps }) = sec.shape else {
            continue;
        };
        let Some(mat) = elem
            .material
            .and_then(|mid| model.materials.iter().find(|m| m.id == mid))
        else {
            continue;
        };
        let fc = mat.fc.unwrap_or(0.0);
        if fc <= 0.0 {
            continue;
        }
        // 壁の平面寸法: 節点群の水平距離の最大 = l、鉛直 extent = h。
        let coords: Vec<[f64; 3]> = elem
            .nodes
            .iter()
            .filter_map(|nid| model.nodes.get(nid.index()))
            .map(|n| n.coord)
            .collect();
        if coords.len() < 3 {
            continue;
        }
        let mut l = 0.0_f64;
        for i in 0..coords.len() {
            for jj in (i + 1)..coords.len() {
                let dx = coords[i][0] - coords[jj][0];
                let dy = coords[i][1] - coords[jj][1];
                l = l.max((dx * dx + dy * dy).sqrt());
            }
        }
        if l < 1e-9 {
            continue;
        }
        let h = coords.iter().map(|c| c[2]).fold(f64::MIN, f64::max)
            - coords.iter().map(|c| c[2]).fold(f64::MAX, f64::min);

        // 壁自重属性（開口面積合計・個別開口寸法・三方スリット）。未登録の壁は
        // 開口ゼロ・スリット無し（無開口の耐震壁）として扱う。
        let attr = model.wall_attrs.iter().find(|w| w.elem == elem.id);
        let has_slit = attr.map(|a| a.three_side_slit).unwrap_or(false);

        // 開口寸法 (l0',h0') の評価。h・l ≤ 0（寸法不定）の場合は開口ゼロ扱い
        // とする。
        let (mut l0p, mut h0p) = if h > 1e-9 && l > 1e-9 {
            match attr.and_then(|a| a.opening_dims_for(model.multi_opening_mode)) {
                // モード適用後の開口が単一（複数開口の包絡・統合の結果 1 個に
                // なった場合を含む）: 実寸法をそのまま使う（γ1=1-l0/l・
                // γ3=1-h0/h へ実寸法が直接効くため、等価開口への置換はしない）。
                Some(dims) if dims.len() == 1 => dims[0],
                // モード適用後も複数開口が残る場合（Auto で包絡しきれない対
                // が残る・Envelope で位置不明の開口が残る・Equivalent で
                // 複数開口のまま）は、面積総和を保つ単一の等価開口に統合する
                // （RC規準（耐震壁の複数開口の等価化））。
                Some(dims) => equivalent_opening(&dims, l, h),
                // 個別寸法が未入力（合計面積のみ）の場合は従来どおり、壁と
                // 同じ辺長比を持つ擬似ペアから等価開口を復元する（後方互換）。
                None => {
                    let area = attr
                        .map(|a| a.total_opening_area_for(model.multi_opening_mode))
                        .unwrap_or(0.0);
                    if area > 0.0 {
                        equivalent_opening(&[(area / h, h)], l, h)
                    } else {
                        (0.0, 0.0)
                    }
                }
            }
        } else {
            (0.0, 0.0)
        };
        // 開口寸法が壁寸法を超える場合のガード（実寸法入力の誤り等に対する
        // 安全側処理）。
        l0p = l0p.clamp(0.0, l);
        h0p = h0p.clamp(0.0, h);

        // 耐震壁判定（RC規準（耐震壁判定））。スリットあり・壁厚
        // <120mm・開口周比 r0>0.4 のいずれかに該当する壁は耐震壁として
        // 扱わないため、RC規準18条の耐震壁せん断検定自体を対象外とする。
        let r0 = opening_ratio_r0(h0p, l0p, h, l);
        let judge = WallJudgeInput {
            thickness,
            r0,
            has_slit,
        };
        if !is_seismic_wall(&judge) {
            continue;
        }
        // 側柱: 壁節点のうち 2 節点を両端に持つ鉛直部材。
        let wall_nodes = &elem.nodes;
        let mut side_columns = Vec::new();
        let mut sum_col_depth = 0.0;
        // せん断非線形トリリニア（Qc/βu/Qu）用の側柱諸元の集計。
        let mut col_gross_area = 0.0_f64; // Σ b·d（Aw の側柱分）
        let mut col_main_area_max = 0.0_f64; // 引張側柱1本の主筋量の代表値
        let mut dc_max = 0.0_f64; // 圧縮側柱せい Dc の代表値
        for m in members {
            if !m.is_column() {
                continue;
            }
            let n0 = m.elem.nodes[0];
            let n1 = m.elem.nodes[1];
            if !(wall_nodes.contains(&n0) && wall_nodes.contains(&n1)) {
                continue;
            }
            // SRC 側柱（内蔵鉄骨あり）はウェブせん断断面積 As と鋼種の F 値から
            // sfs・As を Qc への加算項として算定する（冒頭 doc 参照）。RC 側柱
            // （内蔵鉄骨なし）は 0。
            let steel_shear = match m.sec.shape {
                Some(SectionShape::SrcRect {
                    steel_height,
                    steel_web_thick,
                    steel_flange_thick,
                    ref steel_grade,
                    ..
                }) => {
                    let as_web =
                        (steel_web_thick * (steel_height - 2.0 * steel_flange_thick)).max(0.0);
                    let f = crate::steel::steel_f_value_prefix(
                        steel_grade,
                        steel_flange_thick.max(steel_web_thick),
                    )
                    .unwrap_or(235.0);
                    crate::steel::steel_fs(f, term) * as_web
                }
                _ => 0.0,
            };
            let bd_rebar = match m.sec.shape {
                Some(SectionShape::RcRect { b, d, ref rebar }) => Some((b, d, rebar)),
                Some(SectionShape::SrcRect {
                    b, d, ref rebar, ..
                }) => Some((b, d, rebar)),
                _ => None,
            };
            let Some((b, d, rebar)) = bd_rebar else {
                continue;
            };
            let dt = rc_dt(rebar);
            let pw = if rebar.shear.pitch > 0.0 {
                rebar.shear.legs as f64 * std::f64::consts::PI * rebar.shear.dia * rebar.shear.dia
                    / 4.0
                    / (b * rebar.shear.pitch)
            } else {
                0.0
            };
            side_columns.push(WallSideColumn {
                b,
                d_eff: d - dt,
                pw,
                w_ft: crate::rc::rebar_allowable_shear(&m.mat.name, term == LoadTerm::Long),
                steel_shear,
            });
            sum_col_depth += d;
            // 非線形トリリニア用: 側柱の全断面積・主筋量・せいを集計。
            col_gross_area += b * d;
            dc_max = dc_max.max(d);
            let bar_area = |bs: &squid_n_core::section_shape::BarSet| -> f64 {
                bs.count as f64 * std::f64::consts::PI / 4.0 * bs.dia * bs.dia
            };
            let main_area = bar_area(&rebar.main_x) + bar_area(&rebar.main_y);
            col_main_area_max = col_main_area_max.max(main_area);
        }
        let l_clear = (l - sum_col_depth / 2.0).max(0.1 * l);
        // 設計用せん断力: 等価梁化された壁要素内力の最大水平せん断成分（暫定）。
        let q_design = forces
            .iter()
            .map(|(_, f)| f[1].abs().max(f[2].abs()))
            .fold(0.0, f64::max);
        let inp = RcWallInput {
            t: thickness,
            l,
            l_clear,
            fc,
            ps,
            w_ft: crate::rc::rebar_allowable_shear(&mat.name, term == LoadTerm::Long),
            side_columns,
            // 開口寸法 (l0',h0')（単一開口は実寸法・複数開口は等価開口・
            // 面積のみは擬似等価開口）を 18条のγ式（r=min(γ1,γ2,γ3)）へ
            // 供給する（冒頭 doc 参照。02章の r0/r とは別式のため流用しない）。
            opening: if l0p > 1e-9 && h0p > 1e-9 {
                Some((l0p, h0p, h, l))
            } else {
                None
            },
            q_design,
            long_term: term == LoadTerm::Long,
        };
        let cr = rc_wall_shear_check(&inp);
        out.push((elem.nodes[0], "耐震壁(RC)".to_string(), cr));

        // ── せん断非線形トリリニア骨格（Qc/βu/Qu、技術基準解説書）──
        // 非線形解析のせん断ばね骨格。付帯柱の主筋量が得られる耐震壁のみ算定する。
        let aw = thickness * l + col_gross_area;
        let d_wall = l + sum_col_depth / 2.0;
        if col_main_area_max > 0.0 && aw > 0.0 && d_wall > 0.0 {
            // 等価壁厚 te = Aw/D（壁厚 t の 1.5 倍以下、t 以上）。
            let te = (aw / d_wall).clamp(thickness, 1.5 * thickness);
            // 平均軸方向応力度 σ0 = 圧縮軸力/Aw（引張は 0）。
            let n_comp = forces.iter().map(|(_, f)| -f[0]).fold(0.0_f64, f64::max);
            let sigma_0 = n_comp / aw;
            // せん断スパン比 M/(Q·D): |M| 最大位置の M/Q を D で割る。
            // せん断力が実質 0 の位置しかない場合は h/(2·D)（反曲点中央）で代用。
            let shear_span_ratio = forces
                .iter()
                .max_by(|a, b| {
                    a.1[5]
                        .abs()
                        .partial_cmp(&b.1[5].abs())
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .and_then(|(_, f)| {
                    let q = f[1].abs().max(f[2].abs());
                    (q > 1e-6).then(|| f[5].abs() / q / d_wall)
                })
                .unwrap_or_else(|| h / (2.0 * d_wall));
            let tri_inp = WallShearTrilinearInput {
                fc,
                aw,
                tension_column_main_area: col_main_area_max,
                pw_vertical: ps,
                sigma_y_wall: 295.0, // 壁縦筋 SD295 相当、要・原典照合
                te,
                t: thickness,
                d_wall,
                dc_compression: dc_max,
                tension_column_at: col_main_area_max,
                sigma_wh: 295.0, // 壁横筋 SD295 相当、要・原典照合
                pwh_ratio: ps,
                sigma_0,
                shear_span_ratio,
                high_strength_shear_rebar: false,
                opening: if l0p > 1e-9 && h0p > 1e-9 {
                    Some((l0p, h0p, h, l))
                } else {
                    None
                },
            };
            let tri = wall_shear_trilinear(&tri_inp);
            // 終局せん断強度に対する設計用せん断力の比（Qu 検定）。
            let ratio = if tri.qu > 0.0 { q_design / tri.qu } else { 0.0 };
            let detail = format!(
                "Qc={:.1} kN, βu={:.3}, Qu={:.1} kN, r={:.3}, QD={:.1} kN（せん断非線形トリリニア骨格）",
                tri.qc / 1000.0,
                tri.beta_u,
                tri.qu / 1000.0,
                tri.r_opening,
                q_design / 1000.0
            );
            out.push((
                elem.nodes[0],
                "耐震壁(RC)せん断非線形".to_string(),
                CheckResult {
                    ratio,
                    ok: ratio <= 1.0,
                    basis: "技術基準解説書 耐震壁せん断非線形(Qc/βu/Qu)".to_string(),
                    detail,
                    components: vec![CheckComponent {
                        kind: CheckKind::Shear,
                        ratio,
                    }],
                },
            ));
        }
    }
}
