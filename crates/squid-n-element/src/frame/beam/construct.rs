//! モデルデータからの [`BeamElement`] 構築（断面性能の組み立て）。
//!
//! 危険断面リスト算定、SRC/CFT 等価換算、スラブ協力幅・合成梁・壁エレメント上下
//! 大梁の剛性倍率適用（[`super::stiffness_factors`]）、フレーム内雑壁の断面性能算入
//! を行う。

use super::element::BeamElement;
use super::stiffness_factors::{
    composite_beam_stiffness_factor, is_wall_top_bottom_girder, slab_stiffness_factor,
    WALL_GIRDER_STIFF_FACTOR,
};
use crate::transform::LocalFrame;
use squid_n_core::ids::NodeId;
use squid_n_core::model::{Material, Model, Section};

fn get_section(model: &Model, sid: Option<squid_n_core::ids::SectionId>) -> Section {
    sid.and_then(|s| {
        if s.index() < model.sections.len() {
            let sec = &model.sections[s.index()];
            if sec.id == s {
                Some(sec.clone())
            } else {
                None
            }
        } else {
            None
        }
    })
    .unwrap_or_else(|| Section {
        id: squid_n_core::ids::SectionId(0),
        name: String::new(),
        area: 0.0,
        iy: 0.0,
        iz: 0.0,
        j: 0.0,
        depth: 0.0,
        width: 0.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    })
}

fn get_material(model: &Model, mid: Option<squid_n_core::ids::MaterialId>) -> Material {
    mid.and_then(|m| {
        if m.index() < model.materials.len() {
            let mat = &model.materials[m.index()];
            if mat.id == m {
                Some(mat.clone())
            } else {
                None
            }
        } else {
            None
        }
    })
    .unwrap_or_else(|| Material {
        concrete_class: Default::default(),
        id: squid_n_core::ids::MaterialId(0),
        name: String::new(),
        young: 0.0,
        poisson: 0.0,
        density: 0.0,
        shear: None,
        fc: None,
        fy: None,
    })
}

impl BeamElement {
    pub fn new(data: &squid_n_core::model::ElementData, model: &Model) -> Self {
        let n0 = data.nodes[0];
        let n1 = data.nodes[1];
        let p0 = if n0.index() < model.nodes.len() {
            model.nodes[n0.index()].coord
        } else {
            [0.0; 3]
        };
        let p1 = if n1.index() < model.nodes.len() {
            model.nodes[n1.index()].coord
        } else {
            [0.0; 3]
        };
        let dx = p1[0] - p0[0];
        let dy = p1[1] - p0[1];
        let dz = p1[2] - p0[2];
        let len = (dx * dx + dy * dy + dz * dz).sqrt();

        let axis = LocalFrame::from_nodes(p0, p1, data.local_axis.ref_vector);
        let sec = get_section(model, data.section);
        let mat = get_material(model, data.material);
        let g = mat.shear_modulus();

        // 危険断面位置（§6.2.3、既定は柱フェース＝節点から face_i/j）を正規化座標へ変換し、
        // 節点芯 [0.0, 1.0] と部材中央 0.5 に加えて評価断面リストへ含める。
        // face=0（直交材が無い端）では従来どおり [0.0, 0.5, 1.0] と完全一致する。
        // 部材付帯情報（ハンチ・継手位置。剛性には影響しない）があれば、その
        // 追加検定位置（ハンチ端・継手位置）も評価断面へ含める（§6.2.3 の
        // 「位置はユーザが追加・変更可能」に対応。剛性は基準断面のまま）。
        let eval_sections = if len > 1e-12 {
            let xi_i = (data.rigid_zone.face_i / len).clamp(0.0, 0.5 - 1e-9);
            let xi_j = (1.0 - data.rigid_zone.face_j / len).clamp(0.5 + 1e-9, 1.0);
            let mut xs = vec![0.0, xi_i, 0.5, xi_j, 1.0];
            if let Some(detail) = model.member_detail(data.id) {
                xs.extend(detail.extra_check_positions(&data.rigid_zone, len));
            }
            xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
            xs.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
            xs
        } else {
            vec![0.0, 0.5, 1.0]
        };

        let as_y = if sec.as_y != 0.0 {
            sec.as_y
        } else {
            squid_n_core::model::rect_shear_area(sec.area)
        };
        let as_z = if sec.as_z != 0.0 {
            sec.as_z
        } else {
            squid_n_core::model::rect_shear_area(sec.area)
        };

        // SRC/CFT の複合換算断面性能（SRC規準の考え方・ヤング係数比による等価換算）。
        // 要素材料からヤング係数比を算定して剛性用の断面性能を上書きする。
        // - SRC: 材料=コンクリート（fc あり）のとき ns=Es/Ec で累加。
        // - CFT: 材料=鋼管の young と充填コンクリート強度 fc から 1/n 換算で累加。
        // 算定不能（fc 無し・Ec≤0 等）なら to_section の既定値
        // （SRC: N_S_EQ 固定、CFT: 鋼管のみ）のまま。質量用 a_mass は常に幾何断面。
        use squid_n_core::section_shape::SectionShape;
        let composite = sec.shape.as_ref().and_then(|shape| match shape {
            SectionShape::SrcRect { .. } => mat
                .fc
                .is_some()
                .then(|| shape.src_equivalent_props(mat.young, mat.poisson))
                .flatten(),
            SectionShape::CftBox { .. } | SectionShape::CftPipe { .. } => mat
                .fc
                .and_then(|fc| shape.cft_equivalent_props(mat.young, mat.poisson, fc)),
            _ => None,
        });

        // SRC で材料から算定できない場合も、軸剛性だけは既定 N_S_EQ の累加を維持する。
        let a_stiff = match (&composite, &sec.shape) {
            (Some(p), _) => p.area_ax,
            (None, Some(shape @ SectionShape::SrcRect { .. })) => shape.calc_axial_stiffness_area(),
            _ => sec.area,
        };
        let (sec_iy, sec_iz, j, sec_as_y, sec_as_z) = match &composite {
            Some(p) => (p.iy, p.iz, p.j, p.as_y, p.as_z),
            None => (sec.iy, sec.iz, sec.j, as_y, as_z),
        };

        // 断面レイヤ→要素座標系のクロス変換。
        // 断面レイヤの規約（P1 §2.3・builder.rs）は「iy=強軸（せい方向 D³ 系）、
        // as_z=強軸曲げ用せん断面積（ウェブ）」だが、要素座標系はせい方向＝ローカル y
        // のため、強軸曲げは z 軸まわり（Mz 面、たわみ y 方向、(uy,rz) ブロック）に
        // 対応する（stiffness.rs は iz・as_y をこのブロックに用いる）。
        //   要素 iz（Mz 面）  ← 断面 iy（強軸）    要素 as_y（yせん断）← 断面 as_z（ウェブ）
        //   要素 iy（My 面）  ← 断面 iz（弱軸）    要素 as_z（zせん断）← 断面 as_y（フランジ）
        // このクロスを行わないと、梁の鉛直曲げが弱軸剛性・水平曲げが強軸剛性で
        // 解かれてしまう（軸名の取り違え）。
        let (iy, iz, as_y, as_z) = (sec_iz, sec_iy, sec_as_z, sec_as_y);

        // スラブ協力幅による強軸剛性増大（RC規準8条のスラブ協力幅・合成梁は
        // 各種合成構造設計指針）。RC 矩形梁は T 形断面の Ie/I0、H 形鋼梁は合成梁の平均
        // 剛性 (I+sI)/(2·sI)。Model::slab_thickness=0（既定）では 1.0 で無効。
        // 強軸（鉛直曲げ）＝要素座標系では iz（Mz 面）へ乗じる。
        let iz = match &sec.shape {
            Some(SectionShape::RcRect { .. }) => {
                iz * slab_stiffness_factor(model, data, sec.width, sec.depth)
            }
            Some(SectionShape::SteelH { .. }) => {
                iz * composite_beam_stiffness_factor(model, data, &sec, mat.young)
            }
            _ => iz,
        };

        // 壁エレメントモデルの上下大梁の剛性倍率（壁エレメント置換モデルの上下大梁の断面性能）。
        // 対象は水平材（協力幅判定と同様、勾配 5% までは水平とみなす）かつ、両端節点が
        // 四隅を持つ Wall 要素の節点集合に含まれる（=壁の上辺・下辺の大梁）場合のみ。
        // 倍率は剛性用の値（a, iy, iz, j, as_y, as_z）にのみ乗じ、質量用 a_mass は
        // 幾何断面のまま変更しない。
        let lp = (dx * dx + dy * dy).sqrt();
        let is_horizontal = lp > 1e-9 && dz.abs() <= 0.05 * lp;
        let wall_girder_factor = if is_horizontal && is_wall_top_bottom_girder(model, n0, n1) {
            WALL_GIRDER_STIFF_FACTOR
        } else {
            1.0
        };
        let mut a_stiff = a_stiff * wall_girder_factor;
        let mut iy = iy * wall_girder_factor;
        let mut iz = iz * wall_girder_factor;
        let j = j * wall_girder_factor;
        let mut as_y = as_y * wall_girder_factor;
        let mut as_z = as_z * wall_girder_factor;

        // フレーム内雑壁（耐震壁不成立）の周辺部材への断面性能算入
        // （フレーム内雑壁のモデル化）。柱（鉛直材）には袖壁を、
        // 梁（水平材）には腰壁/垂壁を、平行軸の定理で剛性用断面性能へ合成する。
        // 対象は不成立壁のみ（成立壁は上下大梁100倍で別途考慮済み・排他）。
        // 合成は「腰壁・垂壁のヤング係数は母材と同じと仮定」の規定に基づく
        // 同材累加であり、コンクリート系（RC/SRC、`mat.fc` あり）の部材のみ対象。
        // S 造部材へ無換算（ヤング係数比なし）で壁断面を合成すると壁寄与を
        // 1 桁近く過大評価するため適用しない。
        let is_concrete_member = mat.fc.is_some();
        let misc_walls = if is_concrete_member {
            crate::misc_wall::collect_misc_walls(model)
        } else {
            Vec::new()
        };
        if !misc_walls.is_empty() {
            // 不変条件の確認: 壁要素と周辺部材（柱・梁）の ElemId は別空間ではなく
            // モデル全体で一意のはずなので、自部材自身が雑壁として収集される
            // （壁要素IDと自部材IDの衝突）ことはない。
            for w in &misc_walls {
                debug_assert_ne!(
                    w.elem, data.id,
                    "壁要素と周辺部材のIDが衝突している（model 不整合）"
                );
            }
            let is_vertical_member = dz.abs() > (dx.abs() + dy.abs()) * 0.5;

            // 自部材の両端節点集合が節点対 b と一致するか（順序不問）
            let same_pair = |a: [NodeId; 2], b: (NodeId, NodeId)| -> bool {
                (a[0] == b.0 && a[1] == b.1) || (a[0] == b.1 && a[1] == b.0)
            };
            // 平行軸の定理による合成: contrib = (合成断面積 Aw, 部材中心からの
            // 符号付き距離 e, 合成断面の自身回りの断面2次モーメント)。
            // 図心 g = Σ(Aw·e)/(Ac+ΣAw) を求めた上で I を再合成する。
            let compose = |i0: f64, ac: f64, contrib: &[(f64, f64, f64)]| -> f64 {
                let sum_aw: f64 = contrib.iter().map(|c| c.0).sum();
                if sum_aw <= 0.0 {
                    return i0;
                }
                let sum_aw_e: f64 = contrib.iter().map(|c| c.0 * c.1).sum();
                let g = sum_aw_e / (ac + sum_aw);
                let mut i_new = i0 + ac * g * g;
                for &(aw, e, self_i) in contrib {
                    i_new += self_i + aw * (e - g).powi(2);
                }
                i_new
            };

            if is_vertical_member {
                // 柱（鉛直材）: 袖壁の算入。面内せいは近似として断面の大きい方の
                // 辺（sec.depth.max(sec.width)）を用いる。
                let d_col = sec.depth.max(sec.width);
                let ac = a_stiff;
                let mut contrib_y: Vec<(f64, f64, f64)> = Vec::new(); // iz・as_y を増強
                let mut contrib_z: Vec<(f64, f64, f64)> = Vec::new(); // iy・as_z を増強
                let mut a_add = 0.0;

                for wall in &misc_walls {
                    for s in 0..2 {
                        let pair = (wall.bottom_pair[s], wall.top_pair[s]);
                        if !same_pair([n0, n1], pair) {
                            continue;
                        }
                        let lww = (wall.wing_length(s) - d_col / 2.0).clamp(0.0, wall.lw);
                        if lww <= 0.0 {
                            continue;
                        }
                        let Some(pa) = model
                            .nodes
                            .get(wall.bottom_pair[0].index())
                            .map(|n| n.coord)
                        else {
                            continue;
                        };
                        let Some(pb) = model
                            .nodes
                            .get(wall.bottom_pair[1].index())
                            .map(|n| n.coord)
                        else {
                            continue;
                        };
                        let wdx = pb[0] - pa[0];
                        let wdy = pb[1] - pa[1];
                        let wl = (wdx * wdx + wdy * wdy).sqrt();
                        if wl < 1e-9 {
                            continue;
                        }
                        // 壁下辺方向の水平単位ベクトルと柱の局所 ey・ez との内積で
                        // 面内たわみ方向（iz↔as_y か iy↔as_z か）を判定する。
                        let e_wall = [wdx / wl, wdy / wl, 0.0];
                        let dot_ey = (axis.rot[1][0] * e_wall[0]
                            + axis.rot[1][1] * e_wall[1]
                            + axis.rot[1][2] * e_wall[2])
                            .abs();
                        let dot_ez = (axis.rot[2][0] * e_wall[0]
                            + axis.rot[2][1] * e_wall[1]
                            + axis.rot[2][2] * e_wall[2])
                            .abs();

                        let aw = wall.t * lww;
                        let e_i = if s == 0 {
                            -(d_col / 2.0 + lww / 2.0)
                        } else {
                            d_col / 2.0 + lww / 2.0
                        };
                        let self_i = wall.t * lww.powi(3) / 12.0;
                        a_add += aw;
                        if dot_ey >= dot_ez {
                            contrib_y.push((aw, e_i, self_i));
                        } else {
                            contrib_z.push((aw, e_i, self_i));
                        }
                    }
                }

                if !contrib_y.is_empty() {
                    iz = compose(iz, ac, &contrib_y);
                    as_y += contrib_y.iter().map(|c| c.0).sum::<f64>() / 1.2;
                }
                if !contrib_z.is_empty() {
                    iy = compose(iy, ac, &contrib_z);
                    as_z += contrib_z.iter().map(|c| c.0).sum::<f64>() / 1.2;
                }
                a_stiff += a_add;
            } else if is_horizontal {
                // 梁（水平材）: 腰壁（下辺の梁に載る壁）・垂壁（上辺の梁から垂れる壁）の
                // 算入。鉛直曲げ（要素座標系では iz・as_y＝Mz 面）へ合成する。
                let d_beam = sec.depth;
                let ac = a_stiff;
                let mut contrib: Vec<(f64, f64, f64)> = Vec::new();
                let mut a_add = 0.0;

                for wall in &misc_walls {
                    let bottom = (wall.bottom_pair[0], wall.bottom_pair[1]);
                    let top = (wall.top_pair[0], wall.top_pair[1]);
                    // 下辺の梁なら壁は上に載る（腰壁）、上辺の梁なら壁は下に垂れる（垂壁）。
                    let (matched, hw_raw, sign) = if same_pair([n0, n1], bottom) {
                        (true, wall.strip_height(false), 1.0)
                    } else if same_pair([n0, n1], top) {
                        (true, wall.strip_height(true), -1.0)
                    } else {
                        (false, 0.0, 0.0)
                    };
                    if !matched {
                        continue;
                    }
                    let hw = (hw_raw - d_beam / 2.0).clamp(0.0, wall.h);
                    if hw <= 0.0 {
                        continue;
                    }
                    let aw = wall.t * hw;
                    let e_i = sign * (d_beam / 2.0 + hw / 2.0);
                    let self_i = wall.t * hw.powi(3) / 12.0;
                    a_add += aw;
                    contrib.push((aw, e_i, self_i));
                }

                if !contrib.is_empty() {
                    iz = compose(iz, ac, &contrib);
                    as_y += contrib.iter().map(|c| c.0).sum::<f64>() / 1.2;
                }
                a_stiff += a_add;
            }
        }

        Self {
            id: data.id,
            e: mat.young,
            g,
            a: a_stiff,
            a_mass: sec.area,
            iy,
            iz,
            j,
            as_y,
            as_z,
            length: len,
            density: mat.density,
            nodes: [n0, n1],
            axis,
            rigid: data.rigid_zone,
            end_cond: data.end_cond,
            eval_sections,
            section: data.section,
            material: data.material,
            committed_disp: [0.0; 12],
        }
    }
}
