//! 鉄骨ブレースの断面検定（鋼構造設計規準の
//! 鉄骨ブレースの許容応力度検定）。

use crate::material_strength::{steel_fc, steel_ft};
use crate::{effective_slenderness, CheckResult, DesignCtx, LoadTerm, MemberForcesAt};
use squid_n_core::model::Section;

use super::{nonzero, safe_denom};

/// 鉄骨ブレースの断面検定（鋼構造設計規準）。
///
/// 軸力のみ（曲げ・せん断は考慮しない）: 引張 `σt/ft`、圧縮 `σc/fc`（座屈考慮）。
pub(crate) fn check_brace(
    forces: &MemberForcesAt,
    sec: &Section,
    ctx: &DesignCtx,
    f: f64,
    term: LoadTerm,
) -> CheckResult {
    let area = nonzero(sec.area);
    // 有効細長比 λ = max(lk_y/i_y, lk_z/i_z)（強軸・弱軸を個別の座屈長さで評価）。
    let lk_y_resolved = ctx.lk_y.unwrap_or(ctx.length);
    let lk_z_resolved = ctx.lk_z.unwrap_or(ctx.length);
    let buckling_note = if lk_y_resolved <= 1e-9 && lk_z_resolved <= 1e-9 {
        "（座屈長さ0のため座屈無視 λ=0）"
    } else {
        ""
    };
    let lambda = effective_slenderness(sec.iy, sec.iz, area, ctx.length, ctx.lk_y, ctx.lk_z);

    let term_label = match term {
        LoadTerm::Long => "長期",
        LoadTerm::Short => "短期",
    };

    if forces.n < 0.0 {
        // 圧縮: σc/fc（座屈を考慮した許容圧縮応力度、鋼構造設計規準 1973）。
        let sigma_c = forces.n.abs() / area;
        let fc_val = steel_fc(f, lambda, term);
        let ratio = sigma_c / safe_denom(fc_val);
        CheckResult {
            ratio,
            ok: ratio <= 1.0,
            basis: format!(
                "鋼構造設計規準 {} ブレース: 圧縮 σc/fc(座屈考慮){}",
                term_label, buckling_note
            ),
            detail: format!(
                "σc={:.4} N/mm², fc={:.4} N/mm², λ={:.3}",
                sigma_c, fc_val, lambda
            ),
        }
    } else {
        // 引張: σt/ft（座屈を考慮しない単純検定）。
        let sigma_t = forces.n / area;
        let ft_val = steel_ft(f, term);
        let ratio = sigma_t / safe_denom(ft_val);
        CheckResult {
            ratio,
            ok: ratio <= 1.0,
            basis: format!("鋼構造設計規準 {} ブレース: 引張 σt/ft", term_label),
            detail: format!("σt={:.4} N/mm², ft={:.4} N/mm²", sigma_t, ft_val),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::steel::test_support::{mat, rect_section};
    use crate::steel::SteelDesign;
    use crate::{DesignCheck, MemberKind};

    #[test]
    fn test_brace_tension_ok() {
        let sec = rect_section(100.0, 100.0, "L-100x100x10");
        let mat_v = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 200_000.0, // 引張
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 0.0,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Brace,
            length: 4000.0,
            ..Default::default()
        };
        let result = SteelDesign.check(&forces, &sec, &mat_v, &ctx);
        let expected = (200_000.0 / sec.area) / (235.0 / 1.5);
        assert!((result.ratio - expected).abs() < 1e-9);
        assert!(result.ok);
    }

    #[test]
    fn test_brace_compression_slender_fails() {
        // 細長比が大きい（断面が小さく部材長が長い）圧縮ブレースは fc が下がり NG になる。
        let sec = rect_section(20.0, 20.0, "L-20x20x3");
        let mat_v = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.5,
            n: -50_000.0, // 圧縮
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 0.0,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Brace,
            length: 6000.0, // 非常に細長い
            ..Default::default()
        };
        let result = SteelDesign.check(&forces, &sec, &mat_v, &ctx);
        assert!(
            !result.ok,
            "slender brace should fail: ratio={}",
            result.ratio
        );
        assert!(result.ratio > 1.0);
    }

    #[test]
    fn test_brace_compression_stocky_passes() {
        // 太く短いブレースは座屈の影響が小さく OK になりやすい。
        let sec = rect_section(300.0, 300.0, "BOX-300x300x16");
        let mat_v = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.5,
            n: -100_000.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 0.0,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Brace,
            length: 1000.0,
            ..Default::default()
        };
        let result = SteelDesign.check(&forces, &sec, &mat_v, &ctx);
        assert!(
            result.ok,
            "stocky brace should pass: ratio={}",
            result.ratio
        );
    }
}
