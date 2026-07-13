use super::*;
use squid_n_core::ids::{MaterialId, SectionId};
use squid_n_core::units::ConcreteClass;

fn make_material(fc: f64, grade: &str) -> Material {
    Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: grade.to_string(),
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: Some(fc),
        fy: None,
    }
}

fn make_material_no_fc(grade: &str) -> Material {
    Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: grade.to_string(),
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: None,
    }
}

fn make_section(shape: SectionShape) -> Section {
    shape.to_section(SectionId(0), "test".to_string())
}

fn zero_forces() -> MemberForcesAt {
    MemberForcesAt {
        pos: 0.0,
        n: 0.0,
        qy: 0.0,
        qz: 0.0,
        my: 0.0,
        mz: 0.0,
    }
}

fn ctx_column(term: LoadTerm) -> DesignCtx {
    DesignCtx {
        term,
        kind: crate::MemberKind::Column,
        ..Default::default()
    }
}

fn cft_box_section(height: f64, width: f64, thick: f64) -> Section {
    make_section(SectionShape::CftBox {
        height,
        width,
        thick,
    })
}

fn cft_pipe_section(outer_dia: f64, thick: f64) -> Section {
    make_section(SectionShape::CftPipe { outer_dia, thick })
}

// ------------------------------------------------------------------
// CFT 矩形: 閉形式
// ------------------------------------------------------------------

#[test]
fn test_cft_rect_xn_half_d() {
    let (cb, cd, fc) = (400.0, 400.0, 8.0);
    let xn = 0.5 * cd;
    let (cn, cm) = cft_rect_cn_cm(cb, cd, fc, xn);
    let expected_cn = cb * cd * fc * (0.5 / 2.0);
    let expected_cm = cb * cd * cd * fc * (0.5 * (3.0 - 1.0) / 12.0);
    assert!((cn - expected_cn).abs() / expected_cn < 1e-9);
    assert!((cm - expected_cm).abs() / expected_cm < 1e-9);
}

#[test]
fn test_cft_rect_xn_eq_d_continuity() {
    let (cb, cd, fc) = (400.0, 400.0, 8.0);
    let (cn, cm) = cft_rect_cn_cm(cb, cd, fc, cd);
    // Xn=1 の境界で両分岐が一致することを確認する。
    let expected_cn = cb * cd * fc * 0.5;
    let expected_cm = cb * cd * cd * fc * (1.0 / 12.0);
    assert!((cn - expected_cn).abs() / expected_cn < 1e-6);
    assert!((cm - expected_cm).abs() / expected_cm < 1e-6);
}

#[test]
fn test_cft_rect_xn_2d() {
    let (cb, cd, fc) = (400.0, 400.0, 8.0);
    let xn = 2.0 * cd;
    let (cn, cm) = cft_rect_cn_cm(cb, cd, fc, xn);
    let expected_cn = cb * cd * fc * (1.0 - 1.0 / 4.0);
    let expected_cm = cb * cd * cd * fc * (1.0 / 24.0);
    assert!((cn - expected_cn).abs() / expected_cn < 1e-9);
    assert!((cm - expected_cm).abs() / expected_cm < 1e-9);
}

#[test]
fn test_cft_rect_matches_numeric_integration() {
    let (cb, cd, fc) = (350.0, 500.0, 10.0);
    for &xr in &[0.3, 0.8, 1.0, 1.5, 3.0] {
        let xn = xr * cd;
        let (cn_closed, cm_closed) = cft_rect_cn_cm(cb, cd, fc, xn);
        let (cn_num, cm_num) = numeric_cn_cm(cd, fc, xn, |_| cb);
        assert!(
            (cn_closed - cn_num).abs() / cn_closed.max(1.0) < 5e-3,
            "xr={xr}: cn_closed={cn_closed}, cn_num={cn_num}"
        );
        assert!(
            (cm_closed - cm_num).abs() / cm_closed.max(1.0) < 5e-3,
            "xr={xr}: cm_closed={cm_closed}, cm_num={cm_num}"
        );
    }
}

// ------------------------------------------------------------------
// CFT 円形
// ------------------------------------------------------------------

#[test]
fn test_cft_circle_positive_and_small_at_small_xn() {
    let dc = 400.0;
    let fc = 8.0;
    let (cn, cm) = cft_circle_cn_cm(dc, fc, 0.05 * dc);
    assert!(cn > 0.0 && cm > 0.0);
    assert!(cn < std::f64::consts::PI * dc * dc / 4.0 * fc);
}

#[test]
fn test_cft_circle_converges_to_area_times_fc() {
    let dc = 400.0;
    let fc = 8.0;
    let (cn, _) = cft_circle_cn_cm(dc, fc, 1000.0 * dc);
    let ca_fc = std::f64::consts::PI * dc * dc / 4.0 * fc;
    assert!((cn - ca_fc).abs() / ca_fc < 1e-3, "cn={cn}, ca_fc={ca_fc}");
}

// ------------------------------------------------------------------
// CFT 柱: DesignCheck 経由
// ------------------------------------------------------------------

#[test]
fn test_cft_box_n0_ma_equals_sm0() {
    let sec = cft_box_section(400.0, 300.0, 9.0);
    let mat = make_material(24.0, "SN400B");
    let ctx = ctx_column(LoadTerm::Long);
    let design = CftDesign;

    let forces = MemberForcesAt {
        mz: 1.0,
        ..zero_forces()
    };
    let r = design.check(&forces, &sec, &mat, &ctx);
    let ma_z = 1.0 / r.ratio;

    let (_sa, sz_z, _sz_y) = cft_box_steel_props(400.0, 300.0, 9.0);
    let f_value = steel_f_value_prefix("SN400B", 9.0).unwrap();
    let s_ft = steel_ft(f_value, LoadTerm::Long);
    let s_mo = sz_z * s_ft;
    assert!(
        (ma_z - s_mo).abs() / s_mo < 1e-6,
        "ma_z={ma_z}, s_mo={s_mo}"
    );
}

#[test]
fn test_cft_box_n_exceeds_cnc_steel_only() {
    let sec = cft_box_section(400.0, 300.0, 9.0);
    let mat = make_material(24.0, "SN400B");
    let ctx = ctx_column(LoadTerm::Long);
    let design = CftDesign;

    let forces = MemberForcesAt {
        n: -20_000_000.0,
        mz: 1_000_000.0,
        ..zero_forces()
    };
    let r = design.check(&forces, &sec, &mat, &ctx);
    assert!(r.ratio.is_finite());
    assert!(r.detail.contains("cNc"));
}

#[test]
fn test_cft_pipe_biaxial_smoke() {
    let sec = cft_pipe_section(400.0, 12.0);
    let mat = make_material(24.0, "STKR400");
    let ctx = ctx_column(LoadTerm::Short);
    let design = CftDesign;

    let forces = MemberForcesAt {
        pos: 0.0,
        n: -500_000.0,
        qy: 30_000.0,
        qz: 20_000.0,
        my: 8_000_000.0,
        mz: 15_000_000.0,
    };
    let r = design.check(&forces, &sec, &mat, &ctx);
    assert!(r.ratio.is_finite() && r.ratio >= 0.0);
    assert!(r.basis.contains("円形"));
}

#[test]
fn test_cft_shear_box() {
    let sec = cft_box_section(400.0, 300.0, 9.0);
    let mat = make_material(24.0, "SN400B");
    let ctx = ctx_column(LoadTerm::Long);
    let design = CftDesign;

    let (sa, _, _) = cft_box_steel_props(400.0, 300.0, 9.0);
    let f_value = steel_f_value_prefix("SN400B", 9.0).unwrap();
    let s_fs = steel_fs(f_value, LoadTerm::Long);
    let dw = 400.0 - 2.0 * 9.0;
    let s_aw = 2.0 * 9.0 * dw;
    let s_qa = s_aw * s_fs;
    let _ = sa;

    let forces = MemberForcesAt {
        qy: s_qa * 0.4,
        ..zero_forces()
    };
    let r = design.check(&forces, &sec, &mat, &ctx);
    assert!((r.ratio - 0.4).abs() < 1e-3, "ratio={}", r.ratio);
}

/// 軽量コンクリート1種の充填 CFT は cNc が 0.9 倍に低減され、
/// 圧縮軸力超過時の検定比が普通コンクリートより大きくなる
/// （`mat.concrete_class` が許容応力度算定に反映されている）。
#[test]
fn test_cft_box_lightweight_reduces_cnc() {
    let sec = cft_box_section(400.0, 300.0, 9.0);
    let mut mat_n = make_material(24.0, "SN400B");
    mat_n.concrete_class = ConcreteClass::Normal;
    let mut mat_l = make_material(24.0, "SN400B");
    mat_l.concrete_class = ConcreteClass::Lightweight1;
    let ctx = ctx_column(LoadTerm::Long);
    let design = CftDesign;

    // 圧縮容量を大きく超える軸力を与え、ratio_axial = N/(cNc+sNc) を比較する。
    let forces = MemberForcesAt {
        n: -50_000_000.0,
        ..zero_forces()
    };
    let r_n = design.check(&forces, &sec, &mat_n, &ctx);
    let r_l = design.check(&forces, &sec, &mat_l, &ctx);
    assert!(
        r_l.ratio > r_n.ratio,
        "軽量1種は cNc 低減で検定比が大きいはず: normal={}, light={}",
        r_n.ratio,
        r_l.ratio
    );
}

#[test]
fn test_cft_fc_missing_skip() {
    let sec = cft_box_section(400.0, 300.0, 9.0);
    let mat = make_material_no_fc("SN400B");
    let ctx = ctx_column(LoadTerm::Long);
    let design = CftDesign;
    let result = design.check(&zero_forces(), &sec, &mat, &ctx);
    assert!(result.ok);
    assert_eq!(result.ratio, 0.0);
    assert!(result.basis.contains("Fc"));
}

#[test]
fn test_cft_shape_mismatch_skip() {
    let sec = Section {
        id: SectionId(0),
        name: "no-shape".to_string(),
        area: 1.0,
        iy: 1.0,
        iz: 1.0,
        j: 1.0,
        depth: 400.0,
        width: 400.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let mat = make_material(24.0, "SN400B");
    let ctx = ctx_column(LoadTerm::Long);
    let design = CftDesign;
    let result = design.check(&zero_forces(), &sec, &mat, &ctx);
    assert!(result.ok);
    assert!(result.basis.contains("断面形状不一致"));
}

// ------------------------------------------------------------------
// 地震時短期の設計用せん断力割増: CFT 柱
// ------------------------------------------------------------------

/// CFT 柱の設計用せん断力を QD2 = |QL| + n・|Q−QL| に置き換えると
/// （QL=0 のとき）せん断検定比が n 倍になることを確認する（method=Qd2）。
#[test]
fn test_cft_box_seismic_qd2_scales_shear_ratio_by_n() {
    use crate::{QdMethod, SeismicQd};

    let sec = cft_box_section(400.0, 300.0, 9.0);
    let mat = make_material(24.0, "SN400B");
    let design = CftDesign;

    let (_sa, _, _) = cft_box_steel_props(400.0, 300.0, 9.0);
    let f_value = steel_f_value_prefix("SN400B", 9.0).unwrap();
    let s_fs = steel_fs(f_value, LoadTerm::Short);
    let dw = 400.0 - 2.0 * 9.0;
    let s_aw = 2.0 * 9.0 * dw;
    let s_qa = s_aw * s_fs;

    let q_test = s_qa * 0.2;
    let forces = MemberForcesAt {
        qy: q_test,
        ..zero_forces()
    };

    let ctx_none = ctx_column(LoadTerm::Short);
    let r_none = design.check(&forces, &sec, &mat, &ctx_none);

    let n_factor = 1.5;
    let ctx_qd = DesignCtx {
        term: LoadTerm::Short,
        kind: crate::MemberKind::Column,
        seismic_qd: Some(SeismicQd {
            long_at: vec![(0.0, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])], // QL=0
            n_factor,
            clear_length: 4000.0,
            method: QdMethod::Qd2,
        }),
        ..Default::default()
    };
    let r_qd = design.check(&forces, &sec, &mat, &ctx_qd);

    assert!(
        (r_qd.ratio - n_factor * r_none.ratio).abs() / r_none.ratio < 1e-6,
        "ratio_none={}, ratio_qd={}, n={}",
        r_none.ratio,
        r_qd.ratio,
        n_factor
    );
}

/// QD1 = ΣcMy/h′（cMy = N-M 相互作用の Mu(N)、ΣcMy=2·Mu）が QD2 より
/// 小さい場合、method=Min では QD1 が設計用せん断力として採用される。
#[test]
fn test_cft_box_seismic_qd1_governs_when_smaller() {
    use crate::{QdMethod, SeismicQd};

    let (height, width, thick) = (400.0, 300.0, 9.0);
    let sec = cft_box_section(height, width, thick);
    let mat = make_material(24.0, "SN400B");
    let design = CftDesign;

    let f_value = steel_f_value_prefix("SN400B", thick).unwrap();
    let s_fs = steel_fs(f_value, LoadTerm::Short);
    let s_aw = 2.0 * thick * (height - 2.0 * thick);
    let s_qa = s_aw * s_fs;

    // 解析せん断を大きくして QD2 = n・Q を QD1 より大きくする。
    let q_test = s_qa * 0.6;
    let forces = MemberForcesAt {
        qy: q_test,
        ..zero_forces()
    };

    let n_factor = 1.5;
    let h_clear = 4000.0;
    let length = 4000.0;
    let ctx_qd = DesignCtx {
        term: LoadTerm::Short,
        kind: crate::MemberKind::Column,
        length,
        seismic_qd: Some(SeismicQd {
            long_at: vec![(0.0, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])], // QL=0
            n_factor,
            clear_length: h_clear,
            method: QdMethod::Min,
        }),
        ..Default::default()
    };
    let r_qd = design.check(&forces, &sec, &mat, &ctx_qd);

    // 期待値: QD1 = 2·Mu(N=0)/h′（強軸。qy に対応）。
    let shape = SectionShape::CftBox {
        height,
        width,
        thick,
    };
    let mu = crate::ultimate::cft_mu_nm(&shape, 24.0, f_value, 0.0, length, false).unwrap();
    let qd1 = 2.0 * mu / h_clear;
    assert!(
        qd1 < n_factor * q_test,
        "前提: QD1({qd1}) < QD2({})",
        n_factor * q_test
    );
    let expected_ratio = qd1 / s_qa;
    assert!(
        (r_qd.ratio - expected_ratio).abs() / expected_ratio < 1e-6,
        "ratio={}, expected={}",
        r_qd.ratio,
        expected_ratio
    );
}

/// ctx.seismic_qd が None のときは CFT も従来どおり解析せん断力の
/// ままとなる（回帰確認）。
#[test]
fn test_cft_box_seismic_qd_none_uses_raw_shear() {
    let sec = cft_box_section(400.0, 300.0, 9.0);
    let mat = make_material(24.0, "SN400B");
    let ctx = ctx_column(LoadTerm::Long);
    let design = CftDesign;

    let (_sa, _, _) = cft_box_steel_props(400.0, 300.0, 9.0);
    let f_value = steel_f_value_prefix("SN400B", 9.0).unwrap();
    let s_fs = steel_fs(f_value, LoadTerm::Long);
    let dw = 400.0 - 2.0 * 9.0;
    let s_aw = 2.0 * 9.0 * dw;
    let s_qa = s_aw * s_fs;

    let forces = MemberForcesAt {
        qy: s_qa * 0.4,
        ..zero_forces()
    };
    let r = design.check(&forces, &sec, &mat, &ctx);
    assert!((r.ratio - 0.4).abs() < 1e-3, "ratio={}", r.ratio);
}
