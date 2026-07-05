use crate::{CheckResult, DesignCheck, DesignCtx, LoadTerm, MemberForcesAt};
use squid_n_core::model::{Material, Section};

/// 鋼材の F 値表（グレード名 → F値 [N/mm²]）（令98条/告示。板厚 t<=40mm の代表値）。
///
/// `steel_f_value`（完全一致）・`steel_f_value_prefix`（前方一致）の両方が
/// この表を共有する（挙動の重複を避けるためテーブルを一本化）。
const STEEL_F_TABLE: &[(&str, f64)] = &[
    ("SS400", 235.0),
    ("SN400", 235.0),
    ("SM400", 235.0),
    ("SS490", 285.0),
    ("SN490", 325.0),
    ("SM490", 325.0),
    ("SN520", 355.0),
    ("SM520", 355.0),
    ("SN550", 450.0),
    ("SM570", 450.0),
];

/// 鋼材の F 値 [N/mm²]（完全一致）。
///
/// 戻り値は F 値。長期許容引張・圧縮・曲げ ft = F/1.5、
/// 長期許容せん断 fs = F/(1.5·√3)。短期は長期の 1.5 倍（=F, F/√3）。
fn steel_f_value(grade: &str) -> Option<f64> {
    STEEL_F_TABLE
        .iter()
        .find(|(g, _)| *g == grade)
        .map(|(_, f)| *f)
}

/// 鋼材の F 値 [N/mm²]（前方一致）。
///
/// 材料名が板厚区分や強度記号等の接尾辞を伴う場合（例 "SN400B"→235、
/// "SM490A"→325）に対応するため、`STEEL_F_TABLE` の記号と材料名の前方一致で
/// 判定する。複数の記号が前方一致しうる場合（表の拡張時等）は、最も記号が
/// 長い（＝最も具体的な）ものを優先する（例: 将来 "SN4" のような短い記号が
/// 追加されても "SN490" の記号自体が優先され、"SN4.." に誤マッチしない）。
pub fn steel_f_value_prefix(name: &str) -> Option<f64> {
    STEEL_F_TABLE
        .iter()
        .filter(|(g, _)| name.starts_with(g))
        .max_by_key(|(g, _)| g.len())
        .map(|(_, f)| *f)
}

/// 鋼材の許容曲げ応力度 fb [N/mm²]。
fn allowable_steel_bending(grade: &str, long_term: bool) -> f64 {
    let f = steel_f_value(grade).unwrap_or(235.0);
    if long_term {
        f / 1.5
    } else {
        f
    }
}

/// 鋼材の許容せん断応力度 fs [N/mm²]。
fn allowable_steel_shear(grade: &str, long_term: bool) -> f64 {
    let f = steel_f_value(grade).unwrap_or(235.0);
    if long_term {
        f / (1.5 * 3.0_f64.sqrt())
    } else {
        f / 3.0_f64.sqrt()
    }
}

pub struct SteelDesign;

impl DesignCheck for SteelDesign {
    fn check(
        &self,
        forces: &MemberForcesAt,
        sec: &Section,
        mat: &Material,
        ctx: &DesignCtx,
    ) -> CheckResult {
        let grade = &mat.name;
        let long_term = ctx.term == LoadTerm::Long;
        let fb = allowable_steel_bending(grade, long_term);
        let fs = allowable_steel_shear(grade, long_term);

        // 断面係数 Z [mm³]（強軸曲げは iz、なければ iy）。
        let z = if sec.iz != 0.0 { sec.iz } else { sec.iy };
        let z_eff = if z > 0.0 { z } else { 1.0 };
        // σ = M[N·mm] / Z[mm³] [N/mm²]
        let sigma_b = forces.m.abs() / z_eff;

        // せん断有効断面積 As [mm²]（z 軸まわりせん断は as_z、なければ as_y、最後に area）。
        let as_eff = if sec.as_z > 0.0 {
            sec.as_z
        } else if sec.as_y > 0.0 {
            sec.as_y
        } else {
            sec.area
        };
        let as_eff = if as_eff > 0.0 { as_eff } else { 1.0 };
        // τ = Q[N] / As[mm²] [N/mm²]
        let tau = forces.q.abs() / as_eff;

        let ratio_b = if fb > 0.0 { sigma_b / fb } else { 0.0 };
        let ratio_s = if fs > 0.0 { tau / fs } else { 0.0 };
        let ratio = ratio_b.max(ratio_s);

        let basis = if long_term {
            "令90条 長期許容応力度 F/1.5"
        } else {
            "令90条 短期許容応力度 F"
        };

        let detail = format!(
            "σ={:.4} N/mm², fb={:.4} N/mm², τ={:.4} N/mm², fs={:.4} N/mm²",
            sigma_b, fb, tau, fs
        );

        CheckResult {
            ratio,
            ok: ratio <= 1.0,
            basis: basis.to_string(),
            detail,
        }
    }
}

pub struct RcDesign;

/// コンクリートの許容圧縮応力度 [N/mm²]（令91条）。
/// 長期 = Fc/3, 短期 = 2·Fc/3。
fn allowable_concrete_compress(fc: f64, long_term: bool) -> f64 {
    if long_term {
        fc / 3.0
    } else {
        2.0 * fc / 3.0
    }
}

/// コンクリートの許容せん断応力度 [N/mm²]（令91条）。
/// 長期 = Fc/20 かつ 0.55 以下、短期は長期の 2 倍（令91条 第1項・第3項の概略）。
fn allowable_concrete_shear(fc: f64, long_term: bool) -> f64 {
    let long = (fc / 20.0).min(0.55);
    if long_term {
        long
    } else {
        2.0 * long
    }
}

/// 異形鉄筋の F 値 [N/mm²]（令96条）。
fn rebar_f_value(grade: &str) -> Option<f64> {
    match grade {
        "SD295" => Some(295.0),
        "SD345" => Some(345.0),
        "SD390" => Some(390.0),
        _ => None,
    }
}

/// 異形鉄筋の長期許容引張応力度 ft [N/mm²]（令90条 表）。
/// SD295=195, SD345=215（D<=25）/195（D>25）, SD390=215。
fn rebar_long_term_allowable(grade: &str) -> f64 {
    match grade {
        "SD295" => 195.0,
        "SD345" => 215.0,
        "SD390" => 215.0,
        _ => 195.0,
    }
}

/// 異形鉄筋の短期許容引張応力度 [N/mm²] = min(長期×1.5, F値)。
fn rebar_short_term_allowable(grade: &str) -> f64 {
    let long = rebar_long_term_allowable(grade);
    let f = rebar_f_value(grade).unwrap_or(295.0);
    (long * 1.5).min(f)
}

impl DesignCheck for RcDesign {
    fn check(
        &self,
        forces: &MemberForcesAt,
        sec: &Section,
        mat: &Material,
        ctx: &DesignCtx,
    ) -> CheckResult {
        let long_term = ctx.term == LoadTerm::Long;
        let fc = mat.fc.unwrap_or(0.0);
        if fc <= 0.0 {
            return CheckResult {
                ratio: 0.0,
                ok: true,
                basis: "RC 検定: Fc 未設定".to_string(),
                detail: "Material.fc が None/0 です。コンクリート強度を設定してください。"
                    .to_string(),
            };
        }

        let fc_allow = allowable_concrete_compress(fc, long_term);
        let fs_c = allowable_concrete_shear(fc, long_term);

        // コンクリート縁応力 σc = M[N·mm] / Z[mm³] [N/mm²]
        // （Z は RC 断面の等価断面係数相当。本格的な鉄筋換算 Z は P4 で SectionShape 経路が
        //  整ってから。ここではSection.iz を用いる暫定。）
        let z = if sec.iz != 0.0 { sec.iz } else { sec.iy };
        let z_eff = if z > 0.0 { z } else { 1.0 };
        let sigma_c = forces.m.abs() / z_eff;

        // せん断応力度 τ = Q[N] / (b·j) [N/mm²]。j = 7d/8 の暫定。
        // b = sec.width, d = sec.depth（RC 矩形の有効せいは depth 暫定）。
        let b = if sec.width > 0.0 { sec.width } else { 1.0 };
        let d = sec.depth;
        let j = 7.0 * d / 8.0;
        let bj = if j > 0.0 { b * j } else { 1.0 };
        let tau = forces.q.abs() / bj;

        let ratio_c = if fc_allow > 0.0 {
            sigma_c / fc_allow
        } else {
            0.0
        };
        let ratio_s = if fs_c > 0.0 { tau / fs_c } else { 0.0 };

        // 鉄筋引張検定（暫定: σs = M/(a_t·j) は a_t が Section に無いため P4 で本格実装。
        //  ここではコンクリート圧縮・せん断を主とし、鉄筋は情報表示のみ。）
        let rebar_grade = &mat.name;
        let ft = if long_term {
            rebar_long_term_allowable(rebar_grade)
        } else {
            rebar_short_term_allowable(rebar_grade)
        };

        let ratio = ratio_c.max(ratio_s);

        let basis = if long_term {
            "令90条・令91条 長期許容応力度"
        } else {
            "令90条・令91条 短期許容応力度"
        };

        let detail = format!(
            "σc={:.4} N/mm², fc_allow={:.4} N/mm², τ={:.4} N/mm², fs_c={:.4} N/mm², 鉄筋 ft={:.4} N/mm² ({})",
            sigma_c, fc_allow, tau, fs_c, ft, rebar_grade
        );

        CheckResult {
            ratio,
            ok: ratio <= 1.0,
            basis: basis.to_string(),
            detail,
        }
    }
}

/// 軸応力と曲げ応力の複合比（参考値）。各々の許容応力度で割った和。
pub fn combined_stress_ratio(
    axial_stress: f64,
    bending_stress: f64,
    allowable_axial: f64,
    allowable_bending: f64,
) -> f64 {
    let axial_ratio = if allowable_axial != 0.0 {
        axial_stress / allowable_axial
    } else {
        0.0
    };
    let bend_ratio = if allowable_bending != 0.0 {
        bending_stress / allowable_bending
    } else {
        0.0
    };
    axial_ratio + bend_ratio
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ss400_allowable_long() {
        let fb = allowable_steel_bending("SS400", true);
        assert!((fb - 235.0 / 1.5).abs() < 1e-9);
        let fs = allowable_steel_shear("SS400", true);
        assert!((fs - 235.0 / (1.5 * 3.0_f64.sqrt())).abs() < 1e-9);
    }

    #[test]
    fn test_ss400_allowable_short() {
        let fb = allowable_steel_bending("SS400", false);
        assert!((fb - 235.0).abs() < 1e-9);
        let fs = allowable_steel_shear("SS400", false);
        assert!((fs - 235.0 / 3.0_f64.sqrt()).abs() < 1e-9);
    }

    #[test]
    fn test_steel_f_value_table() {
        assert!((steel_f_value("SN400").unwrap() - 235.0).abs() < 1e-9);
        assert!((steel_f_value("SN490").unwrap() - 325.0).abs() < 1e-9);
        assert!((steel_f_value("SS490").unwrap() - 285.0).abs() < 1e-9);
        assert!(steel_f_value("UNKNOWN").is_none());
    }

    #[test]
    fn test_steel_f_value_prefix_matches_suffixed_grade() {
        // "SN400B"（板厚区分等の接尾辞付き）→ "SN400" の前方一致で 235。
        assert!((steel_f_value_prefix("SN400B").unwrap() - 235.0).abs() < 1e-9);
        // "SM490A" → "SM490" の前方一致で 325。
        assert!((steel_f_value_prefix("SM490A").unwrap() - 325.0).abs() < 1e-9);
    }

    #[test]
    fn test_steel_f_value_prefix_does_not_confuse_sn400_and_sn490() {
        // "SN490" は "SN400" を前方一致しない（別の記号）→ 325 のまま。
        assert!((steel_f_value_prefix("SN490").unwrap() - 325.0).abs() < 1e-9);
        assert!((steel_f_value_prefix("SN490B").unwrap() - 325.0).abs() < 1e-9);
    }

    #[test]
    fn test_steel_f_value_prefix_unknown_is_none() {
        assert!(steel_f_value_prefix("XYZ").is_none());
    }

    #[test]
    fn test_steel_f_value_prefix_exact_grade_still_matches() {
        // 接尾辞なしの完全一致も前方一致経路で拾えること。
        assert!((steel_f_value_prefix("SS400").unwrap() - 235.0).abs() < 1e-9);
    }

    /// 仕様 P3 §6.4 の検算例（鋼梁の長期曲げ、SN400・板厚<=40 ⇒ F=235、横座屈なし）。
    /// 矩形 B=200, D=400 ⇒ Z = B·D²/6 = 200·400²/6 = 5.3333e6 mm³
    /// M = 100 kN·m = 1e8 N·mm
    /// σ = M/Z = 1e8 / 5.3333e6 = 18.75 N/mm²
    /// fb = F/1.5 = 235/1.5 = 156.6667 N/mm²
    /// 検定比 = σ / fb = 18.75 / 156.6667 = 0.1197（相対 1e-9 で厳密一致）
    #[test]
    fn test_steel_check_bending_spec_p3_6_4() {
        // 矩形断面 B=200, D=400
        let b = 200.0_f64;
        let d = 400.0_f64;
        let z = b * d * d / 6.0;
        let sec = Section {
            id: squid_n_core::ids::SectionId(0),
            name: "矩形200x400".into(),
            area: b * d,
            iy: b * d.powi(3) / 12.0,
            iz: d * b.powi(3) / 12.0,
            j: 0.0,
            depth: d,
            width: b,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        // iz は強軸まわり（ここでは b 側）だが、§6.4 の Z=B·D²/6 は D を載せる方向。
        // 検算例の Z=5.3333e6 は B·D²/6 = 200·400²/6。これを sec.iz に設定して検定する。
        let mut sec = sec;
        sec.iz = z; // 仕様例の断面係数を直接設定

        let mat = Material {
            id: squid_n_core::ids::MaterialId(0),
            name: "SN400".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: None,
            fy: None,
        };
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 0.0,
            q: 0.0,
            m: 1e8, // 100 kN·m = 1e8 N·mm
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
        };
        let result = SteelDesign.check(&forces, &sec, &mat, &ctx);

        // σ = 18.75 N/mm²
        assert!(
            (result
                .detail
                .split("σ=")
                .nth(1)
                .and_then(|s| s.split_whitespace().next())
                .and_then(|s| s.parse::<f64>().ok())
                .unwrap_or(999.9)
                - 18.75)
                .abs()
                < 1e-6,
            "σ should be 18.75, detail={}",
            result.detail
        );
        // 検定比 = 0.1197（相対 1e-9）
        let expected_ratio = 18.75 / (235.0 / 1.5);
        assert!(
            (result.ratio - expected_ratio).abs() < 1e-9,
            "ratio {} != expected {}",
            result.ratio,
            expected_ratio
        );
        assert!(result.ok);
    }

    #[test]
    fn test_steel_check_shear_units() {
        // Q=10000 N, As=800 mm² ⇒ τ=12.5 N/mm²
        let sec = Section {
            id: squid_n_core::ids::SectionId(0),
            name: "test".into(),
            area: 1000.0,
            iy: 1e7,
            iz: 5e6,
            j: 1e5,
            depth: 200.0,
            width: 100.0,
            as_y: 800.0,
            as_z: 800.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let mat = Material {
            id: squid_n_core::ids::MaterialId(0),
            name: "SN400".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: None,
            fy: None,
        };
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 0.0,
            q: 10000.0,
            m: 0.0,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
        };
        let result = SteelDesign.check(&forces, &sec, &mat, &ctx);
        let fs = 235.0 / (1.5 * 3.0_f64.sqrt());
        let expected_tau = 10000.0 / 800.0;
        assert!((expected_tau - 12.5_f64).abs() < 1e-9);
        assert!((result.ratio - expected_tau / fs).abs() < 1e-9);
    }

    #[test]
    fn test_concrete_allowable_long() {
        let fc = allowable_concrete_compress(24.0, true);
        assert!((fc - 24.0 / 3.0).abs() < 1e-9);
    }

    #[test]
    fn test_concrete_allowable_short() {
        let fc = allowable_concrete_compress(24.0, false);
        assert!((fc - 2.0 * 24.0 / 3.0).abs() < 1e-9);
    }

    #[test]
    fn test_rebar_long_term_sd345() {
        let ft = rebar_long_term_allowable("SD345");
        assert!((ft - 215.0).abs() < 1e-9);
    }

    #[test]
    fn test_rebar_short_term_sd345_upper_bound_is_f() {
        // SD345: 長期 215 × 1.5 = 322.5 < F=345 ⇒ 322.5
        let ft = rebar_short_term_allowable("SD345");
        assert!(
            (ft - 322.5).abs() < 1e-9,
            "SD345 short term should be 322.5, got {}",
            ft
        );
    }

    #[test]
    fn test_rebar_short_term_sd295_upper_bound_is_f() {
        // SD295: 長期 195 × 1.5 = 292.5 < F=295 ⇒ 292.5
        let ft = rebar_short_term_allowable("SD295");
        assert!((ft - 292.5).abs() < 1e-9);
    }

    #[test]
    fn test_rc_check_uses_fc_field() {
        // Fc=24, M=1e8 N·mm, Z=5.3333e6 ⇒ σc=18.75, fc_allow=8.0 ⇒ ratio=2.34375 (NG)
        let b = 200.0_f64;
        let d = 400.0_f64;
        let z = b * d * d / 6.0;
        let sec = Section {
            id: squid_n_core::ids::SectionId(0),
            name: "RC200x400".into(),
            area: b * d,
            iy: b * d.powi(3) / 12.0,
            iz: z,
            j: 0.0,
            depth: d,
            width: b,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let mat = Material {
            id: squid_n_core::ids::MaterialId(0),
            name: "SD345".into(),
            young: 25000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        };
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 0.0,
            q: 0.0,
            m: 1e8,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
        };
        let result = RcDesign.check(&forces, &sec, &mat, &ctx);
        let expected = 18.75 / (24.0 / 3.0);
        assert!(
            (result.ratio - expected).abs() < 1e-9,
            "ratio {} != expected {}",
            result.ratio,
            expected
        );
        assert!(!result.ok);
    }

    #[test]
    fn test_rc_check_without_fc_reports_unset() {
        let sec = Section {
            id: squid_n_core::ids::SectionId(0),
            name: "rc".into(),
            area: 80000.0,
            iy: 1e8,
            iz: 5e6,
            j: 0.0,
            depth: 400.0,
            width: 200.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let mat = Material {
            id: squid_n_core::ids::MaterialId(0),
            name: "SD345".into(),
            young: 25000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: None,
            fy: None,
        };
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 0.0,
            q: 0.0,
            m: 1e8,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
        };
        let result = RcDesign.check(&forces, &sec, &mat, &ctx);
        assert!(result.ok);
        assert!(result.detail.contains("Fc 未設定") || result.basis.contains("Fc 未設定"));
    }
}
