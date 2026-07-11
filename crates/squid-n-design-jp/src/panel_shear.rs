//! T5: 柱梁接合部パネルのせん断検定。仕様 specs/P7_二次設計.md §6。
//! P1 §6.7 の PanelResult.tau（パネルせん断応力度）に対する検定。
//! 一次資料: 鋼構造接合部設計指針。補強・周囲拘束の割増は factor で外部化。
use crate::{CheckResult, LoadTerm};
use squid_n_core::model::Material;
use squid_n_element::panel::PanelResult;

/// 鋼パネルの許容せん断応力度 [N/mm²]。F 値 = f。
/// 短期: f / √3、長期: f / (1.5 · √3)。
fn allowable_panel_shear(f: f64, term: LoadTerm) -> f64 {
    match term {
        LoadTerm::Short => f / 3f64.sqrt(),
        LoadTerm::Long => f / (1.5 * 3f64.sqrt()),
    }
}

/// パネルせん断検定。
///
/// - `panel`: パネルゾーン解析結果（τ は [N/mm²]）。
/// - `mat`: 鋼材。F 値は `mat.fy` から取得。
/// - `term`: 長期/短期の種別。
/// - `factor`: 接合部指針の割増係数（通常呼び出しは 1.0 を渡す）。
pub fn check_panel_shear(
    panel: &PanelResult,
    mat: &Material,
    term: LoadTerm,
    factor: f64,
) -> CheckResult {
    let f = match mat.fy {
        Some(v) => v,
        None => {
            return CheckResult {
                ratio: 0.0,
                ok: true,
                basis: "F値(fy)未定義のため検定不能".into(),
                detail: format!(
                    "material '{}' has no fy; panel tau = {:.4} N/mm2",
                    mat.name, panel.tau
                ),
            };
        }
    };

    let fs_base = allowable_panel_shear(f, term);
    let fs = fs_base * factor;

    let ratio = if fs > 0.0 { panel.tau.abs() / fs } else { 0.0 };
    let ok = ratio <= 1.0;

    let term_label = match term {
        LoadTerm::Short => "短期 F/√3",
        LoadTerm::Long => "長期 F/(1.5·√3)",
    };
    let basis = format!("鋼構造接合部設計指針 パネルせん断 {}", term_label);
    let detail = format!(
        "tau = {:.4} N/mm2, fs_base = {:.4} N/mm2, factor = {:.4}, fs = {:.4} N/mm2, ratio = {:.4}",
        panel.tau, fs_base, factor, fs, ratio
    );

    CheckResult {
        ratio,
        ok,
        basis,
        detail,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::ids::MaterialId;
    use squid_n_core::model::Material;
    use squid_n_element::panel::PanelResult;

    fn mat_fy(fy: f64) -> Material {
        Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "S".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: Some(fy),
        }
    }

    fn mat_no_fy() -> Material {
        Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "S".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }
    }

    fn panel(tau: f64) -> PanelResult {
        PanelResult {
            b_ml: 0.0,
            b_mr: 0.0,
            c_ml: 0.0,
            c_mu: 0.0,
            pqc: 0.0,
            pqb: 0.0,
            tau,
        }
    }

    // 短期 fs = 235/√3 ≈ 135.6772...
    fn fs_short_235() -> f64 {
        235.0 / 3f64.sqrt()
    }

    // 長期 fs = 235/(1.5·√3)
    fn fs_long_235() -> f64 {
        235.0 / (1.5 * 3f64.sqrt())
    }

    /// τ = fs ちょうど → ratio = 1.0, ok = true（境界）
    #[test]
    fn test_short_ratio_exactly_one() {
        let fs = fs_short_235();
        let result = check_panel_shear(&panel(fs), &mat_fy(235.0), LoadTerm::Short, 1.0);
        assert!((result.ratio - 1.0).abs() < 1e-9, "ratio={}", result.ratio);
        assert!(result.ok);
    }

    /// τ < fs → ratio < 1.0, ok = true
    #[test]
    fn test_short_ratio_below_one() {
        let fs = fs_short_235();
        let tau = fs * 0.5;
        let result = check_panel_shear(&panel(tau), &mat_fy(235.0), LoadTerm::Short, 1.0);
        assert!((result.ratio - 0.5).abs() < 1e-9, "ratio={}", result.ratio);
        assert!(result.ok);
    }

    /// τ > fs → ratio > 1.0, ok = false
    #[test]
    fn test_short_ratio_above_one() {
        let fs = fs_short_235();
        let tau = fs * 1.2;
        let result = check_panel_shear(&panel(tau), &mat_fy(235.0), LoadTerm::Short, 1.0);
        assert!((result.ratio - 1.2).abs() < 1e-9, "ratio={}", result.ratio);
        assert!(!result.ok);
    }

    /// 長期は fs が短期の 1/1.5 になる
    #[test]
    fn test_long_fs_is_short_divided_by_1p5() {
        let fs_s = fs_short_235();
        let fs_l = fs_long_235();
        assert!((fs_l - fs_s / 1.5).abs() < 1e-9);

        let tau = fs_l; // ちょうど長期 fs
        let result = check_panel_shear(&panel(tau), &mat_fy(235.0), LoadTerm::Long, 1.0);
        assert!((result.ratio - 1.0).abs() < 1e-9, "ratio={}", result.ratio);
        assert!(result.ok);
    }

    /// fy=None → ratio=0, ok=true, basis に「未定義」を含む
    #[test]
    fn test_no_fy_skips_check() {
        let result = check_panel_shear(&panel(200.0), &mat_no_fy(), LoadTerm::Short, 1.0);
        assert_eq!(result.ratio, 0.0);
        assert!(result.ok);
        assert!(
            result.basis.contains("未定義"),
            "basis should contain '未定義': {}",
            result.basis
        );
    }

    /// factor=4/3 → fs が 4/3 倍 → 同じ τ で ratio が 3/4 倍
    #[test]
    fn test_factor_scales_fs() {
        let tau = 100.0;
        let result_1 = check_panel_shear(&panel(tau), &mat_fy(235.0), LoadTerm::Short, 1.0);
        let result_f = check_panel_shear(&panel(tau), &mat_fy(235.0), LoadTerm::Short, 4.0 / 3.0);
        let expected = result_1.ratio * 3.0 / 4.0;
        assert!(
            (result_f.ratio - expected).abs() < 1e-9,
            "factor ratio={}, expected={}",
            result_f.ratio,
            expected
        );
    }
}
