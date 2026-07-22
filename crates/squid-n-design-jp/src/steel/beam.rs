//! 鉄骨造梁の断面検定（鋼構造設計規準の
//! 鉄骨造梁の許容応力度検定）。
//!
//! 軸力（引張/圧縮）+ 二軸曲げ + せん断 + von Mises 型合成応力度の各検定比の
//! 最大値を検定比とする（柱と同様の複合検定。梁は弱軸曲げ Qz・My も
//! 検定する点が柱と異なる）。
//!
//! 検定比には含まれない参考情報として、大梁の必要横補剛数とたわみ
//! （長期のみ）も併せて算定する。

use crate::material_strength::{steel_fc, steel_fs, steel_ft};
use crate::{effective_slenderness, CheckResult, DesignCtx, LoadTerm, MemberForcesAt, SteelFbRule};
use squid_n_core::model::{Material, Section};
use squid_n_core::section_shape::SectionShape;

use super::section::{
    resolve_lb, steel_fb_h, steel_fb_h_new, steel_h_z_with_loss, steel_lateral_buckling_c,
    steel_lateral_buckling_i_af, steel_p_lambda_b,
};
use super::{nonzero, safe_denom, section_modulus, shape_of, ShapeCategory};

/// 鉄骨造梁の断面検定（鋼構造設計規準）。
///
/// - 応力度: `σax=|N|/A`、`σby=|Mz|/Z強軸`（既存の断面欠損 Z' 処理は維持）、
///   `σbz=|My|/Z弱軸`。円形鋼管は二軸曲げを合成した `σb=√(Mz²+My²)/Z強軸`
///   を併せて用いる。
/// - 組合せ検定: 圧縮時 `σc/fc+ΣσB/fb`、引張時 `(σt+ΣσB)/ft`（fc は座屈考慮、
///   fb は H形強軸のみ横座屈考慮）。
/// - 単独曲げ検定: `σby/fb_strong`・`σbz/fb_weak` を組合せ式とは別に検定比の
///   `max` へ含める（軸力 N=0 の純曲げでも横座屈による fb の低減が効くように
///   するため）。
/// - せん断検定: 強軸/弱軸それぞれのせん断有効断面積 Ay/Az による `τ/fs`。
/// - von Mises 検定: 形状ごとの合成応力度（H形はウェブ端の曲げ応力度 σb′
///   を用いた 2 方向せん断それぞれの式の大きい方）を ft で検定する。
///
/// 検定比には含まれない参考情報として、detail 末尾に大梁の必要横補剛数
/// （[`steel_required_lateral_bracing_count`]）とたわみ
/// （[`steel_beam_deflection`]、長期のみ）を付記する。
pub(crate) fn check_beam(
    forces: &MemberForcesAt,
    sec: &Section,
    mat: &Material,
    ctx: &DesignCtx,
    f: f64,
    term: LoadTerm,
) -> CheckResult {
    let h = sec.depth;
    let b = sec.width;
    let area = nonzero(sec.area);
    let (shape, tf, tw) = shape_of(sec);

    // 断面欠損（継手部の欠損率 βf/βw・端部スカラップ αw）を考慮した断面係数。
    // H 形で SteelDesignAttr が与えられている場合のみ Z' に置き換える
    // （鋼構造設計規準「鉄骨の断面検定における断面性能」）。端部判定は評価位置
    // pos<=0.25 / >=0.75 を端部とする（検定位置＝柱フェイス・中央の分類と同じ）。
    let z_strong = match (&ctx.steel_attr, shape) {
        (Some(attr), ShapeCategory::H)
            if attr.joint_flange_loss > 0.0
                || attr.joint_web_loss > 0.0
                || attr.scallop_web_loss > 0.0 =>
        {
            let is_end = !(0.25 < forces.pos && forces.pos < 0.75);
            nonzero(steel_h_z_with_loss(
                h,
                b,
                tw,
                tf,
                attr.joint_flange_loss,
                attr.joint_web_loss,
                attr.scallop_web_loss,
                is_end,
            ))
        }
        _ => nonzero(section_modulus(sec.iy, h / 2.0)),
    };
    let z_weak = nonzero(section_modulus(sec.iz, b / 2.0));

    // 応力度: σax=|N|/A（引張/圧縮共通）、σby=|Mz|/Z強軸、σbz=|My|/Z弱軸。
    let sigma_ax = forces.n.abs() / area;
    let sigma_by = forces.mz.abs() / z_strong;
    let sigma_bz = forces.my.abs() / z_weak;
    // 円形鋼管の合成曲げ応力度 σb=√(Mz²+My²)/Z強軸（強軸/弱軸を区別しない）。
    let sigma_b_pipe = (forces.mz.powi(2) + forces.my.powi(2)).sqrt() / z_strong;

    let ft_val = steel_ft(f, term);
    let fs_val = steel_fs(f, term);

    // 座屈を考慮した許容圧縮応力度 fc（column.rs と同じ流儀）。
    // λ = max(lk_y/i_y, lk_z/i_z)（強軸・弱軸を個別の座屈長さで評価）。
    let lambda = effective_slenderness(sec.iy, sec.iz, area, ctx.length, ctx.lk_y, ctx.lk_z);
    let fc_val = steel_fc(f, lambda, term);

    // 許容曲げ応力度 fb: H形強軸のみ横座屈考慮（旧基準/新基準の切替）、他は ft。
    // 弱軸は横座屈を考慮しないため常に ft。
    let fb_weak = ft_val;
    let fb_strong = match shape {
        ShapeCategory::H => {
            // 横座屈長さ lb の優先順位: ctx.lb 直接指定 > SteelDesignAttr
            // （直接入力 (始端,中央,終端)／等間隔補剛 L/(n+1)）> 部材長。
            let lb = ctx.lb.unwrap_or_else(|| {
                ctx.steel_attr
                    .as_ref()
                    .map(|a| resolve_lb(forces.pos, ctx.length, a.lb_direct, a.lateral_brace_count))
                    .unwrap_or(ctx.length)
            });
            // C 係数は「座屈区間端部」のモーメント比によるが、実装が保持するのは
            // 部材端モーメントのみ。横補剛で lb が部材の部分区間となる場合は
            // 区間端モーメント比が不明なため、安全側の C=1.0 とする。
            let c = if lb < ctx.length - 1e-9 {
                1.0
            } else {
                steel_lateral_buckling_c(ctx)
            };
            match ctx.steel_fb_rule {
                SteelFbRule::Old => {
                    let (i_t, af) = steel_lateral_buckling_i_af(sec, tf, tw);
                    steel_fb_h(f, term, lb, i_t, h, af, c)
                }
                SteelFbRule::New => {
                    let iz = sec.iz;
                    let iw = steel_warping_constant(sec, tf);
                    let j = sec.j;
                    let e = mat.young;
                    let g = mat.shear.unwrap_or(e / (2.0 * (1.0 + mat.poisson)));
                    let p_lambda_b = steel_p_lambda_b(ctx);
                    steel_fb_h_new(f, term, lb, iz, iw, j, e, g, z_strong, c, p_lambda_b)
                }
            }
        }
        _ => ft_val,
    };

    // 組合せ検定（軸力+二軸曲げ）。
    let (ratio_comb, axial_basis) = if forces.n < 0.0 {
        let ratio = match shape {
            ShapeCategory::Pipe => {
                sigma_ax / safe_denom(fc_val) + sigma_b_pipe / safe_denom(fb_strong)
            }
            _ => {
                sigma_ax / safe_denom(fc_val)
                    + sigma_by / safe_denom(fb_strong)
                    + sigma_bz / safe_denom(fb_weak)
            }
        };
        (ratio, "圧縮+曲げ: σc/fc(座屈考慮)+ΣσB/fb")
    } else {
        let ratio = match shape {
            ShapeCategory::Pipe => (sigma_ax + sigma_b_pipe) / safe_denom(ft_val),
            _ => (sigma_ax + sigma_by + sigma_bz) / safe_denom(ft_val),
        };
        (ratio, "引張+曲げ: (σt+ΣσB)/ft")
    };

    // 単独曲げ検定（Util-My/Util-Mz）。N=0 の純曲げでも横座屈による fb の
    // 低減が効くよう、組合せ式とは別に検定比の max へ加える。
    let (ratio_my, ratio_mz) = match shape {
        ShapeCategory::Pipe => (sigma_b_pipe / safe_denom(fb_strong), 0.0),
        _ => (
            sigma_by / safe_denom(fb_strong),
            sigma_bz / safe_denom(fb_weak),
        ),
    };

    // せん断検定（強軸 Qy・弱軸 Qz それぞれのせん断有効断面積による τ/fs）。
    let (ay, az) = beam_shear_area(shape, sec, tf, tw);
    let tau_y = forces.qy.abs() / safe_denom(ay);
    let tau_z = forces.qz.abs() / safe_denom(az);
    let ratio_vy = tau_y / safe_denom(fs_val);
    let ratio_vz = tau_z / safe_denom(fs_val);

    // von Mises 型合成検定（すべて分母 ft）。
    let mises_ratio = match shape {
        ShapeCategory::H => {
            // σb′ = σby・(H−2tf)/H（ウェブ端の曲げ応力度に換算）。
            let sigma_b_prime = sigma_by * (h - 2.0 * tf).max(0.0) / safe_denom(h);
            let case_y = ((sigma_ax + sigma_b_prime).powi(2) + 3.0 * tau_y.powi(2)).sqrt();
            let case_z = ((sigma_ax + sigma_by + sigma_bz).powi(2) + 3.0 * tau_z.powi(2)).sqrt();
            case_y.max(case_z) / safe_denom(ft_val)
        }
        ShapeCategory::Pipe => {
            // 円形鋼管は中立軸検定（曲げ項なし）。
            (sigma_ax.powi(2) + 3.0 * (tau_y.powi(2) + tau_z.powi(2))).sqrt() / safe_denom(ft_val)
        }
        // 角形鋼管・その他: 安全側の一般化として角形と同式を用いる。
        _ => {
            let tau_max = tau_y.max(tau_z);
            ((sigma_ax + sigma_by + sigma_bz).powi(2) + 3.0 * tau_max.powi(2)).sqrt()
                / safe_denom(ft_val)
        }
    };

    let ratio = ratio_comb
        .max(ratio_my)
        .max(ratio_mz)
        .max(ratio_vy)
        .max(ratio_vz)
        .max(mises_ratio);

    let term_label = match term {
        LoadTerm::Long => "長期",
        LoadTerm::Short => "短期",
    };
    let fb_rule_label = match ctx.steel_fb_rule {
        SteelFbRule::Old => "旧基準",
        SteelFbRule::New => "新基準",
    };
    let basis = format!(
        "鋼構造設計規準 {} 梁: 軸力+二軸曲げ・せん断・von Mises ({}, fb={})",
        term_label, axial_basis, fb_rule_label
    );
    let mut detail = format!(
        "σax={:.4} N/mm², σby={:.4} N/mm², σbz={:.4} N/mm², fc={:.4} N/mm², fb={:.4} N/mm², \
τy={:.4} N/mm², τz={:.4} N/mm², 組合せ比={:.4}, My比={:.4}, Mz比={:.4}, Vy比={:.4}, Vz比={:.4}, \
Mises比={:.4}",
        sigma_ax,
        sigma_by,
        sigma_bz,
        fc_val,
        fb_strong,
        tau_y,
        tau_z,
        ratio_comb,
        ratio_my,
        ratio_mz,
        ratio_vy,
        ratio_vz,
        mises_ratio
    );

    if let Some((n, lambda_y)) = steel_required_lateral_bracing_count(f, ctx.length, sec) {
        detail.push_str(&format!(", 必要横補剛数n={} (λy={:.3})", n, lambda_y));
    }
    if let Some(s) = steel_beam_deflection(ctx, sec, mat) {
        let ratio_str = if s.abs() > 1e-9 {
            format!("1/{:.0}", ctx.length / s.abs())
        } else {
            "1/∞".to_string()
        };
        detail.push_str(&format!(", たわみS={:.4} mm (S/l={})", s, ratio_str));
    }

    CheckResult {
        ratio,
        ok: ratio <= 1.0,
        basis,
        detail,
    }
}

// ---------------------------------------------------------------------
// 新基準 fb 用の曲げねじり定数 Iw
// ---------------------------------------------------------------------

/// 曲げねじり定数 Iw [mm⁶]（新基準 fb 用）。
///
/// - `SteelBuiltH`（非対称組立 H）: 上下フランジの寸法から個別に
///   `I_u=t_u・b_u³/12`、`I_l=t_l・b_l³/12` を求め、`hf=H−(t_u+t_l)/2`
///   （上下フランジ図心間距離）として `Iw=hf²・I_u・I_l/(I_u+I_l)`
///   （上下フランジの曲げ剛性比に応じてねじり中心が偏心する非対称 H 用の式）。
/// - それ以外（`SteelH` および shape 無しのフォールバック）: `Iz・(H−tf)²/4`
///   （上下フランジ対称、`Iz` はフランジ図心間の距離の半分だけ離れた 2 枚の
///   フランジが負担するとみなす近似式）。
fn steel_warping_constant(sec: &Section, tf: f64) -> f64 {
    match &sec.shape {
        Some(SectionShape::SteelBuiltH {
            height,
            upper_width,
            upper_thick,
            lower_width,
            lower_thick,
            ..
        }) => {
            let hf = height - (upper_thick + lower_thick) / 2.0;
            let i_u = upper_thick * upper_width.powi(3) / 12.0;
            let i_l = lower_thick * lower_width.powi(3) / 12.0;
            let denom = i_u + i_l;
            if denom > 1e-12 {
                hf * hf * i_u * i_l / denom
            } else {
                0.0
            }
        }
        _ => {
            let h = sec.depth;
            sec.iz * (h - tf).powi(2) / 4.0
        }
    }
}

// ---------------------------------------------------------------------
// 梁のせん断有効断面積（強軸 Ay・弱軸 Az）
// ---------------------------------------------------------------------

/// 梁のせん断有効断面積 `(Ay, Az)` [mm²]。柱の検定で共有する
/// [`super::shear_area`]（強軸のみ）とは別に、梁は弱軸せん断 Qz も検定する
/// ため両方向を求める。
///
/// - H形: `Ay=tw・(H−2tf)`（ウェブ有効せい×ウェブ厚）、
///   `Az=2・B・tf/1.5`（上下フランジ断面積を応力分布係数 1.5 で低減）。
/// - 角形鋼管: 角部外半径 r は断面定義時の入力値（`SteelBox.corner_r`）を
///   用いる。`r>0` は角部を 1/4 円弧とみなし直線部＋角部円弧の断面積を
///   合算する: `Ay=2{t・max(H−2r,0)+π・t・(2r−t)/4}`（`Az` は `H` を `B` に
///   置き換えた同式）。`r=0`（未入力・名前推定フォールバック・角部半径を
///   持たない CftBox）は角部を直角とみなし `Ay=2t・max(H−2t,0)`（`Az` は
///   同様に `B`）。
/// - 円形鋼管: `Ay=Az=π・t・(D−t)/2`（薄肉円管のせん断有効断面積。
///   `D=sec.depth` は外径）。
/// - その他: `Ay=as_y>0 ? as_y : area`、`Az=as_z>0 ? as_z : area`。
fn beam_shear_area(shape: ShapeCategory, sec: &Section, tf: f64, tw: f64) -> (f64, f64) {
    let h = sec.depth;
    let b = sec.width;
    match shape {
        ShapeCategory::H => {
            let ay = (tw * (h - 2.0 * tf).max(0.0)).max(0.0);
            let az = (2.0 * b * tf / 1.5).max(0.0);
            (ay, az)
        }
        ShapeCategory::Box => {
            // 角形鋼管は tf=tw=t（shape_of 参照）。角部外半径 r は断面入力値。
            let t = tw;
            let r = match &sec.shape {
                Some(SectionShape::SteelBox { corner_r, .. }) => corner_r.max(0.0),
                _ => 0.0,
            };
            let (ay, az) = if r > 1e-9 {
                let corner = (std::f64::consts::PI * t * (2.0 * r - t) / 4.0).max(0.0);
                (
                    2.0 * (t * (h - 2.0 * r).max(0.0) + corner),
                    2.0 * (t * (b - 2.0 * r).max(0.0) + corner),
                )
            } else {
                // 角部直角（未入力・CftBox・名前推定フォールバック）。
                (
                    2.0 * t * (h - 2.0 * t).max(0.0),
                    2.0 * t * (b - 2.0 * t).max(0.0),
                )
            };
            (ay.max(0.0), az.max(0.0))
        }
        ShapeCategory::Pipe => {
            let t = tw;
            let d = sec.depth;
            let a = (std::f64::consts::PI * t * (d - t) / 2.0).max(0.0);
            (a, a)
        }
        ShapeCategory::Other => {
            let ay = if sec.as_y > 0.0 { sec.as_y } else { sec.area };
            let az = if sec.as_z > 0.0 { sec.as_z } else { sec.area };
            (ay, az)
        }
    }
}

// ---------------------------------------------------------------------
// 大梁必要横補剛数（情報出力のみ。検定比には含めない）
// ---------------------------------------------------------------------

/// 大梁の必要横補剛数 n と弱軸細長比 λy を求める（保有耐力横補剛・
/// 均等間隔配置。昭55建告1791号第2・技術基準解説書）。検定比には含めない
/// 参考情報。
///
/// `λy = L/iy_weak`（`iy_weak = √(Iz/A)`：squid-n の弱軸＝断面二次モーメント
/// `Section.iz` に対応する断面二次半径、`L = DesignCtx.length`）に対し、
/// 均等間隔配置の条件は
/// - F値 235・215（400N/mm²級）: `λy ≦ 170 + 20n`
/// - それ以外（275以上・490N/mm²級）: `λy ≦ 130 + 20n`
///
/// であり、必要本数は `n = ceil(max(0, (λy − 170)/20))`（490級は 130）となる。
/// 細長い梁（λy が大きい）ほど必要本数が増える。`length` が 0 以下の
/// 場合は `None`（算定省略）。
///
/// 注: `n = (170 − λy)/20` と逆向きの式が示されることもあるが、λy が大きい
/// ほど n=0 となり技術基準解説書の条件と矛盾するため誤記と判断し、告示・
/// 解説書の向きで実装する。
fn steel_required_lateral_bracing_count(f: f64, length: f64, sec: &Section) -> Option<(u32, f64)> {
    if length <= 1e-9 {
        return None;
    }
    let area = nonzero(sec.area);
    let iy_weak_sq = (sec.iz / area).max(0.0);
    let iy_weak = iy_weak_sq.sqrt();
    let lambda_y = if iy_weak > 1e-9 {
        length / iy_weak
    } else {
        0.0
    };

    let is_400_grade = (f - 235.0).abs() < 1e-6 || (f - 215.0).abs() < 1e-6;
    let coef = if is_400_grade { 170.0 } else { 130.0 };

    let n_raw = (lambda_y - coef) / 20.0;
    let n = n_raw.max(0.0).ceil() as u32;
    Some((n, lambda_y))
}

// ---------------------------------------------------------------------
// たわみの検定（情報出力のみ。検定比には含めない。長期のみ）
// ---------------------------------------------------------------------

/// 大梁のたわみ S [mm] を求める（たわみの検定、長期のみ）。
///
/// `S = (5·M0·l²)/(48·E·I) − ((ML+MR)·l²)/(16·E·I)`
///
/// - `ML`, `MR`: [`DesignCtx::end_moments_z`] の絶対値、`l = DesignCtx.length`、
///   `E = Material.young`、`I = Section.iy`（強軸まわり断面二次モーメント）。
/// - `M0`（単純梁と仮定した場合の中央モーメント）は、モーメント図が２次
///   曲線分布（等分布荷重相当）であるという仮定の下、区間中央の実際の
///   曲げモーメント `Mc`（[`DesignCtx::mid_moment_z`]）に「両端モーメント
///   による中央部の低減分」を足し戻すことで近似復元する:
///   `M0 = |Mc| + (|ML| + |MR|) / 2`。
///   （等分布荷重・両端モーメント無しの単純梁では `Mc = M0 = wl²/8` となり、
///   本式は `S = 5wl⁴/(384EI)` に一致する。）
/// - たわみの変形制限（例: `l/300` 等）は本実装では設けず、S の算定値を
///   情報として出力するのみで、変形量に基づく合否判定は行わない。
///
/// `end_moments_z` または `mid_moment_z` が `None`、`term` が長期以外、
/// あるいは `length <= 0` の場合は `None`（算定省略）。
fn steel_beam_deflection(ctx: &DesignCtx, sec: &Section, mat: &Material) -> Option<f64> {
    if ctx.term != LoadTerm::Long {
        return None;
    }
    let (m_i, m_j) = ctx.end_moments_z?;
    let mc = ctx.mid_moment_z?;
    let l = ctx.length;
    if l <= 1e-9 {
        return None;
    }
    let e = mat.young;
    let i = sec.iy;
    if e <= 1e-9 || i <= 1e-9 {
        return None;
    }

    let m_l = m_i.abs();
    let m_r = m_j.abs();
    let m0 = mc.abs() + (m_l + m_r) / 2.0;

    let s = (5.0 * m0 * l * l) / (48.0 * e * i) - ((m_l + m_r) * l * l) / (16.0 * e * i);
    Some(s)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::steel::test_support::{h_section, mat, rect_section};
    use crate::steel::SteelDesign;
    use crate::{DesignCheck, MemberKind};
    use squid_n_core::ids::SectionId;

    // -------------------------------------------------------------
    // SteelDesignAttr の配線（断面欠損 Z'・横座屈長さ lb）
    // -------------------------------------------------------------

    #[test]
    fn test_check_beam_applies_section_loss_attr() {
        use squid_n_core::ids::ElemId;
        use squid_n_core::model::SteelDesignAttr;
        let sec = h_section(400.0, 200.0, 8.0, 13.0);
        let m = mat("SN400B");
        // 端部（pos=0.0）で曲げ支配となる内力。
        let forces = MemberForcesAt {
            pos: 0.0,
            n: 0.0,
            qy: 10_000.0,
            qz: 0.0,
            my: 0.0,
            mz: 1.0e8,
        };
        let ctx_base = DesignCtx {
            kind: MemberKind::Beam,
            length: 4000.0,
            ..Default::default()
        };
        let base = SteelDesign.check(&forces, &sec, &m, &ctx_base);
        let ctx_loss = DesignCtx {
            kind: MemberKind::Beam,
            length: 4000.0,
            steel_attr: Some(SteelDesignAttr {
                elem: ElemId(0),
                joint_flange_loss: 10.0,
                joint_web_loss: 0.0,
                scallop_web_loss: 20.0,
                lb_direct: None,
                lateral_brace_count: None,
            }),
            ..Default::default()
        };
        let with_loss = SteelDesign.check(&forces, &sec, &m, &ctx_loss);
        // 欠損で Z′ が減り、曲げ応力度・検定比が大きくなる。
        assert!(
            with_loss.ratio > base.ratio,
            "loss ratio={} <= base ratio={}",
            with_loss.ratio,
            base.ratio
        );
    }

    #[test]
    fn test_check_beam_lb_from_attr_brace_count() {
        use squid_n_core::ids::ElemId;
        use squid_n_core::model::SteelDesignAttr;
        // 細長い横座屈支配の梁: lb = L/(n+1) の短縮で fb が上がり検定比が下がる。
        let sec = h_section(600.0, 150.0, 8.0, 10.0);
        let m = mat("SN400B");
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 0.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 1.0e8,
        };
        let ctx_no_brace = DesignCtx {
            kind: MemberKind::Beam,
            length: 12_000.0,
            ..Default::default()
        };
        let base = SteelDesign.check(&forces, &sec, &m, &ctx_no_brace);
        let ctx_braced = DesignCtx {
            kind: MemberKind::Beam,
            length: 12_000.0,
            steel_attr: Some(SteelDesignAttr {
                elem: ElemId(0),
                joint_flange_loss: 0.0,
                joint_web_loss: 0.0,
                scallop_web_loss: 0.0,
                lb_direct: None,
                lateral_brace_count: Some(5),
            }),
            ..Default::default()
        };
        let braced = SteelDesign.check(&forces, &sec, &m, &ctx_braced);
        assert!(
            braced.ratio < base.ratio,
            "braced ratio={} >= base ratio={}",
            braced.ratio,
            base.ratio
        );
    }

    // -------------------------------------------------------------
    // 梁検定
    // -------------------------------------------------------------

    /// 仕様 P3 §6.4 の検算例を新 API で再現する。
    /// 矩形 B=200, D=400 ⇒ Z=B·D²/6=5.3333e6 mm³, M=1e8 N·mm
    /// σ=18.75 N/mm², fb=F/1.5=156.6667 N/mm²（矩形は横座屈対象外＝fb=ft）,
    /// 検定比=0.1197（相対 1e-9）。
    #[test]
    fn test_beam_check_bending_spec_p3_6_4() {
        let sec = rect_section(200.0, 400.0, "矩形200x400");
        let mat_v = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 0.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 1e8,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Beam,
            length: 0.0,
            ..Default::default()
        };
        let result = SteelDesign.check(&forces, &sec, &mat_v, &ctx);

        let expected_sigma = 18.75;
        let expected_fb = 235.0 / 1.5;
        let expected_ratio = expected_sigma / expected_fb;
        assert!(
            (result.ratio - expected_ratio).abs() < 1e-9,
            "ratio {} != {}",
            result.ratio,
            expected_ratio
        );
        assert!(result.ok);
        assert!(result.detail.contains("18.7500"));
    }

    #[test]
    fn test_beam_check_shear_h_shape_von_mises() {
        // H-300x300x10x15 相当（厚さ 15mm を単一 thickness として近似）。
        let mut sec = rect_section(300.0, 300.0, "H-300x300x10x15");
        sec.thickness = Some(15.0);
        let mat_v = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 0.0,
            qy: 200_000.0,
            qz: 0.0,
            my: 0.0,
            mz: 0.0,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Beam,
            length: 3000.0,
            ..Default::default()
        };
        let result = SteelDesign.check(&forces, &sec, &mat_v, &ctx);
        // 名前推定フォールバックは tf=tw=15 の単一板厚近似のため
        // Ay=tw・(H−2tf)=15・(300−30)=4050（新式・H形）。
        let ay = 15.0 * (300.0 - 2.0 * 15.0);
        let tau = 200_000.0 / ay;
        let fs = 235.0 / (1.5 * 3.0_f64.sqrt());
        let expected_ratio_shear = tau / fs; // σ=0 なので von Mises 側は τ/fs と一致するはず
        assert!(
            (result.ratio - expected_ratio_shear).abs() < 1e-6,
            "ratio={} expected={}",
            result.ratio,
            expected_ratio_shear
        );
    }

    // -------------------------------------------------------------
    // SectionShape 経由の形状解決（tf ≠ tw の実断面）
    // -------------------------------------------------------------

    /// `Section.shape` がある場合は実寸の tw でウェブせん断面積を計算する
    /// （名前推定＋単一板厚近似ではなく、tw=10 が使われること）。
    #[test]
    fn test_beam_check_uses_shape_tw_for_web_shear() {
        let sec = h_section(400.0, 200.0, 10.0, 15.0);
        let mat_v = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 0.0,
            qy: 100_000.0,
            qz: 0.0,
            my: 0.0,
            mz: 0.0,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Beam,
            length: 0.0,
            ..Default::default()
        };
        let result = SteelDesign.check(&forces, &sec, &mat_v, &ctx);
        // Ay=tw・(H−2tf)=10・(400−30)=3700。
        let ay = 10.0 * (400.0 - 2.0 * 15.0);
        let tau = 100_000.0 / ay;
        let fs = 235.0 / (1.5 * 3.0_f64.sqrt());
        let expected = ((3.0_f64.sqrt() * tau) / (235.0 / 1.5)).max(tau / fs);
        assert!(
            (result.ratio - expected).abs() < 1e-9,
            "ratio={} expected={}",
            result.ratio,
            expected
        );
    }

    /// F 値の板厚区分は shape の最大板厚で判定する（tf=45 → 40mm 超区分）。
    #[test]
    fn test_f_value_bucket_uses_shape_max_thickness() {
        let sec = h_section(900.0, 400.0, 20.0, 45.0);
        let mat_v = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 0.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 1e6,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Beam,
            length: 0.0,
            ..Default::default()
        };
        let result = SteelDesign.check(&forces, &sec, &mat_v, &ctx);
        // F=215（40mm 超）→ fb=ft=215/1.5=143.33...
        assert!(
            result.detail.contains("fb=143.3"),
            "detail should show fb from F=215: {}",
            result.detail
        );
    }

    // -------------------------------------------------------------
    // 組合せ検定（軸力+二軸曲げ）
    // -------------------------------------------------------------

    /// 圧縮軸力+二軸曲げ（H形）: σc/fc+σby/fb+σbz/ft を手計算照合する。
    #[test]
    fn test_beam_check_compression_biaxial_bending_hand_calc() {
        let sec = h_section(400.0, 200.0, 8.0, 13.0);
        let mat_v = mat("SN400B");
        let forces = MemberForcesAt {
            pos: 0.5,
            n: -200_000.0,
            qy: 0.0,
            qz: 0.0,
            my: 5e6,
            mz: 8e7,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Beam,
            length: 4000.0,
            ..Default::default()
        };
        let result = SteelDesign.check(&forces, &sec, &mat_v, &ctx);

        let f = 235.0;
        let area = sec.area;
        let z_strong = sec.iy / (sec.depth / 2.0);
        let z_weak = sec.iz / (sec.width / 2.0);
        let sigma_c = 200_000.0 / area;
        let sigma_by = 8e7_f64 / z_strong;
        let sigma_bz = 5e6_f64 / z_weak;

        let i_min = (sec.iy.min(sec.iz) / area).sqrt();
        let lambda = 4000.0 / i_min;
        let fc = steel_fc(f, lambda, LoadTerm::Long);
        let ft = steel_ft(f, LoadTerm::Long);
        // fb_strong は横座屈考慮（既存ロジック、H形は steel_lateral_buckling_i_af
        // で (i,af) を解決する）。この内力配分では組合せ式が支配的となる
        // （σax が大きく、mises 式の σax+σby′ 項に対し fc・fb の分母が効くため）。
        let (i_t, af) = steel_lateral_buckling_i_af(&sec, 13.0, 8.0);
        let fb_strong = steel_fb_h(f, LoadTerm::Long, 4000.0, i_t, 400.0, af, 1.0);
        let expected_comb = sigma_c / fc + sigma_by / fb_strong + sigma_bz / ft;
        assert!(
            (result.ratio - expected_comb).abs() < 1e-9,
            "ratio={} expected_comb={}",
            result.ratio,
            expected_comb
        );
    }

    /// 引張軸力+二軸曲げ: (σt+σby+σbz)/ft を手計算照合する。
    #[test]
    fn test_beam_check_tension_biaxial_bending_hand_calc() {
        let sec = h_section(400.0, 200.0, 8.0, 13.0);
        let mat_v = mat("SN400B");
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 200_000.0,
            qy: 0.0,
            qz: 0.0,
            my: 5e6,
            mz: 8e7,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Beam,
            length: 4000.0,
            ..Default::default()
        };
        let result = SteelDesign.check(&forces, &sec, &mat_v, &ctx);

        let f = 235.0;
        let area = sec.area;
        let z_strong = sec.iy / (sec.depth / 2.0);
        let z_weak = sec.iz / (sec.width / 2.0);
        let sigma_t = 200_000.0 / area;
        let sigma_by = 8e7_f64 / z_strong;
        let sigma_bz = 5e6_f64 / z_weak;
        let ft = steel_ft(f, LoadTerm::Long);
        let expected_comb = (sigma_t + sigma_by + sigma_bz) / ft;
        // 引張側は von Mises 式（Qz=0）の case_z 項と同値になり、必ず検定比の
        // max に一致する（(σt+σby+σbz)/ft = √((σt+σby+σbz)²+3・0²)/ft）。
        assert!(
            (result.ratio - expected_comb).abs() < 1e-9,
            "ratio={} expected_comb={}",
            result.ratio,
            expected_comb
        );
    }

    /// 円形鋼管の合成曲げ: mz 単独と mz/my 分配で検定比がほぼ一致する
    /// （σb=√(mz²+my²)/Z強軸に一本化されるため、合成モーメントの大きさが
    /// 同じであれば分配方法によらず一致するはず）。
    #[test]
    fn test_beam_check_pipe_combines_biaxial_bending() {
        let mut sec = rect_section(300.0, 300.0, "PIPE-300x12");
        sec.iz = sec.iy; // 円形は iy=iz
        sec.thickness = Some(12.0);
        let mat_v = mat("SN400");
        let forces_mz_only = MemberForcesAt {
            pos: 0.5,
            n: -50_000.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 30e6,
        };
        let forces_split = MemberForcesAt {
            pos: 0.5,
            n: -50_000.0,
            qy: 0.0,
            qz: 0.0,
            my: 30e6 / std::f64::consts::SQRT_2,
            mz: 30e6 / std::f64::consts::SQRT_2,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Beam,
            length: 3000.0,
            ..Default::default()
        };
        let r1 = SteelDesign.check(&forces_mz_only, &sec, &mat_v, &ctx);
        let r2 = SteelDesign.check(&forces_split, &sec, &mat_v, &ctx);
        assert!(
            (r1.ratio - r2.ratio).abs() < 1e-6,
            "pipe combined bending mismatch: {} vs {}",
            r1.ratio,
            r2.ratio
        );
    }

    // -------------------------------------------------------------
    // せん断検定（弱軸 Qz・角形鋼管）
    // -------------------------------------------------------------

    /// H形の弱軸せん断 Qz: Az=2・B・tf/1.5 の手計算照合。
    #[test]
    fn test_beam_check_shear_qz_h_shape_hand_calc() {
        let sec = h_section(400.0, 200.0, 10.0, 15.0);
        let mat_v = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 0.0,
            qy: 0.0,
            qz: 60_000.0,
            my: 0.0,
            mz: 0.0,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Beam,
            length: 0.0,
            ..Default::default()
        };
        let result = SteelDesign.check(&forces, &sec, &mat_v, &ctx);

        let az = 2.0 * 200.0 * 15.0 / 1.5;
        let tau_z = 60_000.0_f64 / az;
        let fs = steel_fs(235.0, LoadTerm::Long);
        let expected = tau_z / fs;
        assert!(
            (result.ratio - expected).abs() < 1e-9,
            "ratio={} expected={}",
            result.ratio,
            expected
        );
    }

    /// 角形鋼管のせん断有効断面積: 断面入力の角部外半径 corner_r を用いた
    /// 式の手計算照合（r=30mm を明示入力）。
    #[test]
    fn test_beam_check_shear_box_corner_radius_hand_calc() {
        use squid_n_core::ids::SectionId;
        use squid_n_core::section_shape::SectionShape;
        let shape = SectionShape::SteelBox {
            height: 300.0,
            width: 300.0,
            thick: 12.0,
            corner_r: 30.0,
        };
        let sec = shape.to_section(SectionId(0), "BOX-300x300x12".to_string());
        let mat_v = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 0.0,
            qy: 150_000.0,
            qz: 0.0,
            my: 0.0,
            mz: 0.0,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Beam,
            length: 0.0,
            ..Default::default()
        };
        let result = SteelDesign.check(&forces, &sec, &mat_v, &ctx);

        let t = 12.0_f64;
        let h = 300.0_f64;
        let r = 30.0_f64;
        let corner = std::f64::consts::PI * t * (2.0 * r - t) / 4.0;
        let ay = 2.0 * (t * (h - 2.0 * r).max(0.0) + corner);
        let tau_y = 150_000.0_f64 / ay;
        let fs = steel_fs(235.0, LoadTerm::Long);
        let expected = tau_y / fs;
        assert!(
            (result.ratio - expected).abs() < 1e-9,
            "ratio={} expected={}",
            result.ratio,
            expected
        );
    }

    /// 角形鋼管の角部外半径が未入力（r=0。名前推定フォールバック含む）の場合は
    /// 角部を直角とみなし Ay=2t(H−2t) となる。
    #[test]
    fn test_beam_check_shear_box_r_zero_falls_back_to_sharp_corner() {
        let mut sec = rect_section(300.0, 300.0, "BOX-300x300x12");
        sec.thickness = Some(12.0);
        let mat_v = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 0.0,
            qy: 150_000.0,
            qz: 0.0,
            my: 0.0,
            mz: 0.0,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Beam,
            length: 0.0,
            ..Default::default()
        };
        let result = SteelDesign.check(&forces, &sec, &mat_v, &ctx);

        let t = 12.0_f64;
        let h = 300.0_f64;
        let ay = 2.0 * t * (h - 2.0 * t);
        let tau_y = 150_000.0_f64 / ay;
        let fs = steel_fs(235.0, LoadTerm::Long);
        let expected = tau_y / fs;
        assert!(
            (result.ratio - expected).abs() < 1e-9,
            "ratio={} expected={}",
            result.ratio,
            expected
        );
    }

    // -------------------------------------------------------------
    // 新基準 fb（AIJ-ASD19）
    // -------------------------------------------------------------

    /// steel_fb_rule 未指定（既定 Old）では従来値（steel_fb_h）と一致する。
    #[test]
    fn test_beam_check_fb_rule_default_matches_old() {
        let sec = h_section(400.0, 200.0, 8.0, 13.0);
        let mat_v = mat("SN400B");
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 0.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 5e7,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Beam,
            length: 6000.0,
            ..Default::default()
        };
        assert_eq!(ctx.steel_fb_rule, SteelFbRule::Old);
        let result = SteelDesign.check(&forces, &sec, &mat_v, &ctx);

        let f = 235.0;
        let (i_t, af) = steel_lateral_buckling_i_af(&sec, 13.0, 8.0);
        let fb_expected = steel_fb_h(f, LoadTerm::Long, 6000.0, i_t, 400.0, af, 1.0);
        assert!(
            result.detail.contains(&format!("fb={:.4}", fb_expected)),
            "detail={}",
            result.detail
        );
    }

    /// 新基準 fb: λb ≤ pλb（横座屈長さが短い）では fb=F/ν（全塑性域）。
    #[test]
    fn test_beam_check_fb_rule_new_plastic_region() {
        let sec = h_section(400.0, 200.0, 8.0, 13.0);
        let mat_v = mat("SN400B");
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 0.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 5e7,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Beam,
            length: 300.0, // 十分短い横座屈長さ→全塑性域
            steel_fb_rule: SteelFbRule::New,
            ..Default::default()
        };
        let result = SteelDesign.check(&forces, &sec, &mat_v, &ctx);

        let f = 235.0;
        let z_strong = sec.iy / (sec.depth / 2.0);
        let iw = steel_warping_constant(&sec, 13.0);
        let e = mat_v.young;
        let g = e / (2.0 * (1.0 + mat_v.poisson));
        let p_lambda_b = steel_p_lambda_b(&ctx);
        let lb = 300.0_f64;

        // My, Me, λb を独立に計算し、λb ≤ pλb（全塑性域）であることを確認したうえで
        // fb=F/ν と一致することを検算する。
        let my = f * z_strong;
        let pi2 = std::f64::consts::PI.powi(2);
        let pi4 = std::f64::consts::PI.powi(4);
        let me = (pi4 * e * sec.iz * e * iw / lb.powi(4)
            + pi2 * e * sec.iz * g * sec.j / lb.powi(2))
        .sqrt();
        let lambda_b = (my / me).sqrt();
        assert!(
            lambda_b <= p_lambda_b,
            "lambda_b={} p_lambda_b={} 全塑性域前提",
            lambda_b,
            p_lambda_b
        );
        let e_lambda_b = 1.0 / 0.6_f64.sqrt();
        let nu = 1.5 + (2.0 / 3.0) * (lambda_b / e_lambda_b).powi(2);
        let expected = f / nu;

        let fb_expected = steel_fb_h_new(
            f,
            LoadTerm::Long,
            lb,
            sec.iz,
            iw,
            sec.j,
            e,
            g,
            z_strong,
            1.0,
            p_lambda_b,
        );
        assert!(
            (fb_expected - expected).abs() < 1e-6,
            "fb_expected={} expected={}",
            fb_expected,
            expected
        );
        assert!(
            result.detail.contains(&format!("fb={:.4}", fb_expected)),
            "detail={}",
            result.detail
        );
    }

    /// 新基準 fb: eλb < λb（横座屈長さが長い）では弾性域式 fb=F/(2.17λb²)。
    #[test]
    fn test_beam_check_fb_rule_new_elastic_region() {
        let f = 235.0;
        let sec = h_section(400.0, 200.0, 8.0, 13.0);
        let mat_v = mat("SN400B");
        let z_strong = sec.iy / (sec.depth / 2.0);
        let iw = steel_warping_constant(&sec, 13.0);
        let g = mat_v.young / (2.0 * (1.0 + mat_v.poisson));
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Beam,
            length: 20_000.0, // 十分長い横座屈長さ→弾性域
            steel_fb_rule: SteelFbRule::New,
            ..Default::default()
        };
        let p_lambda_b = steel_p_lambda_b(&ctx);
        let lb = 20_000.0;
        let fb = steel_fb_h_new(
            f,
            LoadTerm::Long,
            lb,
            sec.iz,
            iw,
            sec.j,
            mat_v.young,
            g,
            z_strong,
            1.0,
            p_lambda_b,
        );

        let my = f * z_strong;
        let e = mat_v.young;
        let pi2 = std::f64::consts::PI.powi(2);
        let pi4 = std::f64::consts::PI.powi(4);
        let me = (pi4 * e * sec.iz * e * iw / lb.powi(4)
            + pi2 * e * sec.iz * g * sec.j / lb.powi(2))
        .sqrt();
        let lambda_b = (my / me).sqrt();
        let e_lambda_b = 1.0 / 0.6_f64.sqrt();
        assert!(lambda_b > e_lambda_b, "lambda_b={} 弾性域前提", lambda_b);
        let expected = f / (2.17 * lambda_b * lambda_b);
        assert!(
            (fb - expected).abs() < 1e-6,
            "fb={} expected={}",
            fb,
            expected
        );
    }

    /// 新基準 fb: lb=0 では横座屈を考慮しない fb=ft。
    #[test]
    fn test_beam_check_fb_rule_new_lb_zero_equals_ft() {
        let f = 235.0;
        let z_strong = 1.0e6;
        let fb = steel_fb_h_new(
            f,
            LoadTerm::Long,
            0.0,
            1.0e7,
            1.0e12,
            1.0e5,
            205_000.0,
            79_000.0,
            z_strong,
            1.0,
            0.3,
        );
        let ft = steel_ft(f, LoadTerm::Long);
        assert!((fb - ft).abs() < 1e-9, "fb={} ft={}", fb, ft);
    }

    // -------------------------------------------------------------
    // steel_p_lambda_b（塑性限界細長比 pλb）
    // -------------------------------------------------------------

    /// 座屈区間中央の曲げが両端部より大きい場合は安全側 pλb=0.3。
    #[test]
    fn test_p_lambda_b_mid_moment_dominant_is_0_3() {
        let ctx = DesignCtx {
            end_moments_z: Some((50.0, 50.0)),
            mid_moment_z: Some(200.0),
            ..Default::default()
        };
        let p = steel_p_lambda_b(&ctx);
        assert!((p - 0.3).abs() < 1e-9, "p={}", p);
    }

    /// 単曲率・等曲げ（M2/M1=−1）→ pλb=0.6−0.3=0.3。
    #[test]
    fn test_p_lambda_b_single_curvature_uniform_is_0_3() {
        let ctx = DesignCtx {
            end_moments_z: Some((100.0, 100.0)),
            ..Default::default()
        };
        let p = steel_p_lambda_b(&ctx);
        assert!((p - 0.3).abs() < 1e-9, "p={}", p);
    }

    /// 複曲率・等曲げ（M2/M1=+1）→ pλb=0.6+0.3=0.9。
    #[test]
    fn test_p_lambda_b_double_curvature_uniform_is_0_9() {
        let ctx = DesignCtx {
            end_moments_z: Some((100.0, -100.0)),
            ..Default::default()
        };
        let p = steel_p_lambda_b(&ctx);
        assert!((p - 0.9).abs() < 1e-9, "p={}", p);
    }

    /// end_moments_z が None の場合は安全側 pλb=0.3。
    #[test]
    fn test_p_lambda_b_none_end_moments_is_0_3() {
        let ctx = DesignCtx {
            end_moments_z: None,
            ..Default::default()
        };
        let p = steel_p_lambda_b(&ctx);
        assert!((p - 0.3).abs() < 1e-9, "p={}", p);
    }

    // -------------------------------------------------------------
    // 大梁必要横補剛数
    // -------------------------------------------------------------

    /// 均等間隔配置 λy ≦ 170 + 20n（400N/mm²級。告示1791号・技術基準解説書）:
    /// λy=90 ≦ 170 → n=0（補剛不要）、λy=250 → n=(250−170)/20=4。
    #[test]
    fn test_required_lateral_bracing_count_hand_calc() {
        let sec = Section {
            id: SectionId(0),
            name: "H-dummy".to_string(),
            area: 100.0,
            iy: 0.0,
            iz: 100.0 * 100.0_f64.powi(2), // iy_weak=√(iz/A)=100mm となるよう設定
            j: 0.0,
            depth: 0.0,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let (n, lambda_y) = steel_required_lateral_bracing_count(235.0, 9000.0, &sec).unwrap();
        assert!((lambda_y - 90.0).abs() < 1e-9, "λy={}", lambda_y);
        assert_eq!(n, 0, "λy=90 ≦ 170 のため補剛不要");

        let (n, lambda_y) = steel_required_lateral_bracing_count(235.0, 25000.0, &sec).unwrap();
        assert!((lambda_y - 250.0).abs() < 1e-9, "λy={}", lambda_y);
        assert_eq!(n, 4, "λy=250 → n=(250−170)/20=4");
    }

    /// length=0 の場合は算定を省略する（None）。
    #[test]
    fn test_required_lateral_bracing_count_skipped_when_length_zero() {
        let sec = Section {
            id: SectionId(0),
            name: "H-dummy".to_string(),
            area: 100.0,
            iy: 0.0,
            iz: 1_000_000.0,
            j: 0.0,
            depth: 0.0,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        assert!(steel_required_lateral_bracing_count(235.0, 0.0, &sec).is_none());
    }

    /// 梁検定 detail 末尾に必要横補剛数が出力されることを確認する。
    #[test]
    fn test_beam_check_detail_contains_bracing_count() {
        let sec = h_section(400.0, 200.0, 10.0, 15.0);
        let mat_v = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 0.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 1e7,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Beam,
            length: 9000.0,
            ..Default::default()
        };
        let result = SteelDesign.check(&forces, &sec, &mat_v, &ctx);
        assert!(
            result.detail.contains("必要横補剛数n="),
            "detail={}",
            result.detail
        );
    }

    // -------------------------------------------------------------
    // たわみの検定
    // -------------------------------------------------------------

    /// 等分布荷重 w [N/mm] の単純梁相当（端部モーメント無し）を
    /// M0=Mc=wl²/8 として与えると、標準公式 5wl⁴/(384EI) と一致する。
    #[test]
    fn test_deflection_matches_uniform_load_formula() {
        let w = 10.0;
        let l = 6000.0;
        let e = 205_000.0;
        let i = 5.0e7;
        let mc = w * l * l / 8.0;

        let sec = Section {
            id: SectionId(0),
            name: "dummy".to_string(),
            area: 1.0,
            iy: i,
            iz: 1.0,
            j: 0.0,
            depth: 0.0,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let material = mat("SN400");
        let material = Material {
            concrete_class: Default::default(),
            young: e,
            ..material
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Beam,
            length: l,
            end_moments_z: Some((0.0, 0.0)),
            mid_moment_z: Some(mc),
            ..Default::default()
        };
        let s = steel_beam_deflection(&ctx, &sec, &material).unwrap();
        let expected = 5.0 * w * l.powi(4) / (384.0 * e * i);
        assert!(
            (s - expected).abs() / expected.abs() < 1e-9,
            "s={} expected={}",
            s,
            expected
        );
    }

    /// 短期（term=Short）ではたわみ算定は省略される（None）。
    #[test]
    fn test_deflection_none_for_short_term() {
        let sec = Section {
            id: SectionId(0),
            name: "dummy".to_string(),
            area: 1.0,
            iy: 5.0e7,
            iz: 1.0,
            j: 0.0,
            depth: 0.0,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let material = mat("SN400");
        let ctx = DesignCtx {
            term: LoadTerm::Short,
            kind: MemberKind::Beam,
            length: 6000.0,
            end_moments_z: Some((1e6, 1e6)),
            mid_moment_z: Some(2e6),
            ..Default::default()
        };
        assert!(steel_beam_deflection(&ctx, &sec, &material).is_none());
    }

    /// 梁検定 detail に短期ではたわみ出力が無いこと（長期では出力されること）
    /// を確認する。
    #[test]
    fn test_beam_check_detail_deflection_only_for_long_term() {
        let sec = h_section(400.0, 200.0, 10.0, 15.0);
        let mat_v = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 0.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 1e7,
        };
        let ctx_long = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Beam,
            length: 6000.0,
            end_moments_z: Some((5e6, 5e6)),
            mid_moment_z: Some(1e7),
            ..Default::default()
        };
        let result_long = SteelDesign.check(&forces, &sec, &mat_v, &ctx_long);
        assert!(
            result_long.detail.contains("たわみS="),
            "detail={}",
            result_long.detail
        );

        let ctx_short = DesignCtx {
            term: LoadTerm::Short,
            kind: MemberKind::Beam,
            length: 6000.0,
            end_moments_z: Some((5e6, 5e6)),
            mid_moment_z: Some(1e7),
            ..Default::default()
        };
        let result_short = SteelDesign.check(&forces, &sec, &mat_v, &ctx_short);
        assert!(
            !result_short.detail.contains("たわみS="),
            "detail={}",
            result_short.detail
        );
    }
}
