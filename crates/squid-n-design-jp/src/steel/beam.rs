//! 鉄骨造梁の断面検定（鋼構造設計規準の
//! 鉄骨造梁の許容応力度検定）。
//!
//! 検定比には含まれない参考情報として、大梁の必要横補剛数とたわみ
//! （長期のみ）も併せて算定する。

use crate::material_strength::{steel_fs, steel_ft};
use crate::{CheckResult, DesignCtx, LoadTerm, MemberForcesAt};
use squid_n_core::model::{Material, Section};

use super::section::{
    resolve_lb, steel_fb_h, steel_h_z_with_loss, steel_i_t, steel_lateral_buckling_c,
};
use super::{nonzero, safe_denom, section_modulus, shape_of, shear_area, ShapeCategory};

/// 鉄骨造梁の断面検定（鋼構造設計規準）。
///
/// σb = |mz|/Z強軸 を fb（H形強軸は横座屈考慮、他は ft）で検定する。
/// せん断は H形のみ von Mises 型（σb′, τ の合成）、他は単純 τ/fs。
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
    // 曲げ応力度 σb = |Mz| / Z強軸（強軸まわり断面係数）。
    let sigma_b = forces.mz.abs() / z_strong;

    let ft_val = steel_ft(f, term);
    let fs_val = steel_fs(f, term);

    let as_shear = shear_area(shape, sec, tw);
    // せん断応力度 τ = Qy / As（せん断有効断面積）。
    let tau = forces.qy.abs() / safe_denom(as_shear);

    let (fb, ratio_shear, shear_basis);
    match shape {
        ShapeCategory::H => {
            let af = b * tf;
            let i_t = steel_i_t(b, tf, h, tw);
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
            fb = steel_fb_h(f, term, lb, i_t, h, af, c);
            // H形ウェブの von Mises 型合成検定（鋼構造設計規準）:
            // σb′ = σb・(H−2tf)/H（ウェブ負担分に換算した曲げ応力度）、
            // √(σb′² + 3τ²)/ft と τ/fs の大きい方を検定比とする。
            let sigma_b_prime = sigma_b * (h - 2.0 * tf).max(0.0) / safe_denom(h);
            let von_mises = (sigma_b_prime.powi(2) + 3.0 * tau.powi(2)).sqrt() / safe_denom(ft_val);
            ratio_shear = von_mises.max(tau / safe_denom(fs_val));
            shear_basis = "H形ウェブ von Mises 照査 (鋼構造設計規準)";
        }
        _ => {
            fb = ft_val;
            // H形以外は単純せん断検定 τ/fs。
            ratio_shear = tau / safe_denom(fs_val);
            shear_basis = "H形以外 τ/fs (鋼構造設計規準)";
        }
    }

    // 曲げ検定比 σb/fb。
    let ratio_bend = sigma_b / safe_denom(fb);
    let ratio = ratio_bend.max(ratio_shear);

    let term_label = match term {
        LoadTerm::Long => "長期",
        LoadTerm::Short => "短期",
    };
    let basis = format!(
        "鋼構造設計規準 {} 梁: 曲げ σ/fb (横座屈考慮={}) と せん断 {}",
        term_label,
        matches!(shape, ShapeCategory::H),
        shear_basis
    );
    let mut detail = format!(
        "σ={:.4} N/mm², fb={:.4} N/mm², τ={:.4} N/mm², fs={:.4} N/mm², 曲げ比={:.4}, せん断比={:.4}",
        sigma_b, fb, tau, fs_val, ratio_bend, ratio_shear
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
        // τ = Q/(t·H) = 200000/(15*300) = 44.444..., fs = 235/(1.5√3)=90.44
        let tau = 200_000.0 / (15.0 * 300.0);
        let fs = 235.0 / (1.5 * 3.0_f64.sqrt());
        let expected_ratio_shear = tau / fs; // σb=0 なので von Mises 側は τ/fs と一致するはず
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
        // τ = Q/(tw·H) = 100000/(10·400) = 25.0
        let tau = 100_000.0 / (10.0 * 400.0);
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
