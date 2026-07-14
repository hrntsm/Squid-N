use super::*;
use test_support::build_symmetric_frame;

// ---- d_value ----
#[test]
fn test_d_value_rigid_beams_general() {
    // 梁が十分剛（ΣKb 大）→ k̄ 大 → a → 1 → D → Kc0
    let e = 1.0;
    let ic = 1.0;
    let h = 1.0;
    let kc0 = 12.0 * e * ic / (h * h * h);
    let d = d_value(e, ic, h, 1e9, false);
    assert!((d - kc0).abs() / kc0 < 1e-6, "a→1 で D→Kc0, got {d}");
}

#[test]
fn test_d_value_known_kbar() {
    // kc = Ic/h = 1, ΣKb = 4 → k̄ = 4/(2·1) = 2 → a = 2/(2+2) = 0.5
    // Kc0 = 12 → D = 0.5·12 = 6
    let d = d_value(1.0, 1.0, 1.0, 4.0, false);
    assert!((d - 6.0).abs() < 1e-9, "got {d}");
}

#[test]
fn test_d_value_first_story() {
    // 最下階: k̄ = 2 → a = (0.5+2)/(2+2) = 0.625 → D = 0.625·12 = 7.5
    let d = d_value(1.0, 1.0, 1.0, 4.0, true);
    assert!((d - 7.5).abs() < 1e-9, "got {d}");
}

#[test]
fn test_d_value_degenerate() {
    assert_eq!(d_value(1.0, 0.0, 1.0, 4.0, false), 0.0);
    assert_eq!(d_value(1.0, 1.0, 0.0, 4.0, false), 0.0);
}

// ---- center_of_rigidity（DoD §8.1）----
#[test]
fn test_center_of_rigidity_dod_example() {
    // 仕様 §5.2 の確定値: Dy=[100,300] @ x=[0,6000] → Xs = 4500
    let cols = vec![
        ColumnStiffness {
            pos: [0.0, 0.0],
            dx: 1.0,
            dy: 100.0,
        },
        ColumnStiffness {
            pos: [6000.0, 0.0],
            dx: 1.0,
            dy: 300.0,
        },
    ];
    let cr = center_of_rigidity(&cols);
    assert!((cr[0] - 4500.0).abs() < 1e-9, "Xs got {}", cr[0]);
}

#[test]
fn test_eccentricity_dod_example() {
    // 上の剛心に重心 Xg=3000 → ex = 1500（DoD §8.1）
    let cols = vec![
        ColumnStiffness {
            pos: [0.0, 0.0],
            dx: 1.0,
            dy: 100.0,
        },
        ColumnStiffness {
            pos: [6000.0, 0.0],
            dx: 1.0,
            dy: 300.0,
        },
    ];
    let cr = center_of_rigidity(&cols);
    let ecc = eccentricity(&cols, [3000.0, 0.0], cr);
    assert!((ecc.ex - 1500.0).abs() < 1e-9, "ex got {}", ecc.ex);
}

#[test]
fn test_eccentricity_symmetric_zero() {
    // 対称 4 本柱 → 剛心＝重心＝中央 → 偏心率 0
    let cols = vec![
        ColumnStiffness {
            pos: [0.0, 0.0],
            dx: 100.0,
            dy: 100.0,
        },
        ColumnStiffness {
            pos: [6000.0, 0.0],
            dx: 100.0,
            dy: 100.0,
        },
        ColumnStiffness {
            pos: [0.0, 6000.0],
            dx: 100.0,
            dy: 100.0,
        },
        ColumnStiffness {
            pos: [6000.0, 6000.0],
            dx: 100.0,
            dy: 100.0,
        },
    ];
    let cr = center_of_rigidity(&cols);
    assert!((cr[0] - 3000.0).abs() < 1e-9);
    assert!((cr[1] - 3000.0).abs() < 1e-9);
    let ecc = eccentricity(&cols, [3000.0, 3000.0], cr);
    assert!(ecc.re_x.abs() < 1e-9 && ecc.re_y.abs() < 1e-9);
}

#[test]
fn test_eccentricity_hand_calc() {
    // 手計算照合（X 加力時偏心率）。
    // 柱4本、すべて Dx=Dy=100 とし x=[0,0,6000,6000], y=[0,6000,0,6000]…ではなく
    // 剛心をずらすため右側を強くする: Dy=[100,100,300,300] @ x=[0,0,6000,6000]
    let cols = vec![
        ColumnStiffness {
            pos: [0.0, 0.0],
            dx: 100.0,
            dy: 100.0,
        },
        ColumnStiffness {
            pos: [0.0, 6000.0],
            dx: 100.0,
            dy: 100.0,
        },
        ColumnStiffness {
            pos: [6000.0, 0.0],
            dx: 100.0,
            dy: 300.0,
        },
        ColumnStiffness {
            pos: [6000.0, 6000.0],
            dx: 100.0,
            dy: 300.0,
        },
    ];
    let cr = center_of_rigidity(&cols);
    // Xs = (100·0+100·0+300·6000+300·6000)/(100+100+300+300) = 3,600,000/800 = 4500
    assert!((cr[0] - 4500.0).abs() < 1e-9, "Xs {}", cr[0]);
    // Ys = (Σ Dx·y)/ΣDx = 100·(0+6000+0+6000)/400 = 3000
    assert!((cr[1] - 3000.0).abs() < 1e-9, "Ys {}", cr[1]);

    // 重心は幾何中央 (3000, 3000) とする → ex = 1500, ey = 0
    let ecc = eccentricity(&cols, [3000.0, 3000.0], cr);
    assert!((ecc.ex - 1500.0).abs() < 1e-9);
    assert!(ecc.ey.abs() < 1e-9);

    // KR = Σ Dx·ȳ² + Σ Dy·x̄²
    //   x̄ = x-4500 = [-4500,-4500,1500,1500], ȳ = y-3000 = [-3000,3000,-3000,3000]
    //   Σ Dx·ȳ² = 100·(3000²·4) = 100·4·9e6 = 3.6e9
    //   Σ Dy·x̄² = 100·4500² + 100·4500² + 300·1500² + 300·1500²
    //           = 2·100·2.025e7 + 2·300·2.25e6 = 4.05e9 + 1.35e9 = 5.4e9
    //   KR = 3.6e9 + 5.4e9 = 9.0e9
    assert!((ecc.kr - 9.0e9).abs() / 9.0e9 < 1e-12, "KR {}", ecc.kr);
    // ΣDx = 400 → rex = √(9.0e9/400) = √2.25e7 = 4743.416...
    let rex = (9.0e9_f64 / 400.0).sqrt();
    assert!((ecc.rex - rex).abs() < 1e-6);
    // Rex = ey/rex = 0（ey=0）, Rey = ex/rey
    assert!(ecc.re_x.abs() < 1e-12);
    let sum_dy = 800.0;
    let rey = (9.0e9_f64 / sum_dy).sqrt();
    assert!((ecc.re_y - 1500.0 / rey).abs() < 1e-9, "Rey {}", ecc.re_y);
}

// ===== モデル自動算定テスト =====

/// テスト1: 対称フレーム → 偏心率 ≈ 0、剛心 ≈ [3000, 3000]。
#[test]
fn test_story_eccentricity_symmetric_zero() {
    let (model, s0) = build_symmetric_frame(None);
    let ecc = story_eccentricity(&model, s0);
    assert!(ecc.re_x.abs() < 1e-6, "re_x={} (should be ~0)", ecc.re_x);
    assert!(ecc.re_y.abs() < 1e-6, "re_y={} (should be ~0)", ecc.re_y);
    // 剛心確認
    let sc = story_centers(&model, s0);
    assert!(
        (sc.center_of_rigidity[0] - 3000.0).abs() < 1.0,
        "Xs={}",
        sc.center_of_rigidity[0]
    );
    assert!(
        (sc.center_of_rigidity[1] - 3000.0).abs() < 1.0,
        "Ys={}",
        sc.center_of_rigidity[1]
    );
}

/// テスト2: 右側柱 iz を 3 倍 → 剛心 x ≈ 4500、偏心距離 ex ≈ 1500。
/// 軸整合フレーム（柱軸=Z）では I_globalY = iz なので Dy ∝ iz。
/// 梁は全柱で対称なので a 補正は全柱一致 → Dy 比 = iz 比。
/// Xs = (1·0 + 1·0 + 3·6000 + 3·6000)/(1+1+3+3) = 4500。重心 = 3000 → ex=1500。
#[test]
fn test_story_eccentricity_biased_rigidity() {
    let (model, s0) = build_symmetric_frame(Some(3.0e6));
    let sc = story_centers(&model, s0);
    let xs = sc.center_of_rigidity[0];
    assert!((xs - 4500.0).abs() < 1.0, "Xs={} (expected ≈4500)", xs);
    let ecc = story_eccentricity(&model, s0);
    // 重心 x = 3000（等質量 4 点の中央）→ ex = |3000 - 4500| = 1500
    assert!(
        (ecc.ex - 1500.0).abs() < 1.0,
        "ex={} (expected ≈1500)",
        ecc.ex
    );
}

/// テスト3: 柱が無い層（story=S1 が存在するが柱の上端は S0）→ 空 Vec、剛心 [0,0]。
#[test]
fn test_story_eccentricity_empty_story() {
    let (model, _s0) = build_symmetric_frame(None);
    // S1 は存在しない（stories は S0 のみ）→ column_stiffnesses は空を返す。
    let s1 = StoryId(1);
    let cols = column_stiffnesses(&model, s1);
    assert!(cols.is_empty(), "S1 に柱が無いはず、got {} 本", cols.len());
    let cor = center_of_rigidity(&cols);
    assert_eq!(cor, [0.0, 0.0], "空時の剛心は [0,0]");
}

// ===== 雑壁の n 倍法 =====

#[test]
fn test_misc_wall_stiffness() {
    // Kw' = n·Aw'·ΣKc/ΣAc = 2·1000·400/400 = 2000
    assert!((misc_wall_stiffness(2.0, 1000.0, 400.0, 400.0) - 2000.0).abs() < 1e-12);
    // ΣAc = 0 → Kw' = 0（0 除算回避）
    assert_eq!(misc_wall_stiffness(2.0, 1000.0, 400.0, 0.0), 0.0);
}

#[test]
fn test_sum_column_area() {
    let (model, s0) = build_symmetric_frame(None);
    // 柱 4 本 × area 100
    assert!((sum_column_area(&model, s0) - 400.0).abs() < 1e-12);
}

#[test]
fn test_append_misc_wall_stiffnesses() {
    use squid_n_core::model::{MiscWall, MiscWallTransfer};
    let (mut model, s0) = build_symmetric_frame(None);
    model.stress_cfg.misc_wall_n = Some(2.0);
    // Y 方向の壁 @ x=6000（長さ 6000 × 厚 100 → Aw' = 6e5）、z_mid=1500 → S0 帰属。
    model.misc_walls.push(MiscWall {
        start: [6000.0, 0.0, 0.0],
        end: [6000.0, 6000.0, 0.0],
        height: 3000.0,
        weight_per_area: 1.0e-3,
        transfer: MiscWallTransfer::SelfStanding,
        thickness: Some(100.0),
    });
    // 帯域外の壁（z_mid = 4500 > elevation 3000）→ 無視される。
    model.misc_walls.push(MiscWall {
        start: [0.0, 0.0, 3000.0],
        end: [0.0, 6000.0, 3000.0],
        height: 3000.0,
        weight_per_area: 1.0e-3,
        transfer: MiscWallTransfer::SelfStanding,
        thickness: Some(100.0),
    });
    // 厚さ未設定の壁 → 無視される。
    model.misc_walls.push(MiscWall {
        start: [0.0, 0.0, 0.0],
        end: [0.0, 6000.0, 0.0],
        height: 3000.0,
        weight_per_area: 1.0e-3,
        transfer: MiscWallTransfer::SelfStanding,
        thickness: None,
    });

    let mut cols = vec![
        ColumnStiffness {
            pos: [0.0, 0.0],
            dx: 100.0,
            dy: 100.0,
        },
        ColumnStiffness {
            pos: [6000.0, 0.0],
            dx: 100.0,
            dy: 100.0,
        },
        ColumnStiffness {
            pos: [0.0, 6000.0],
            dx: 100.0,
            dy: 100.0,
        },
        ColumnStiffness {
            pos: [6000.0, 6000.0],
            dx: 100.0,
            dy: 100.0,
        },
    ];
    append_misc_wall_stiffnesses(&model, s0, &mut cols);
    assert_eq!(cols.len(), 5, "帯域内かつ厚さ有りの壁 1 枚のみ追加");
    let wall = cols[4];
    // Kw'y = n·Aw'·ΣKy/ΣAc = 2·6e5·400/400 = 1.2e6（cy=1 なので dy へ全量）
    assert!((wall.dy - 1.2e6).abs() < 1e-6, "Kw'y={}", wall.dy);
    assert!(wall.dx.abs() < 1e-12, "cx=0 なので dx は 0");
    assert_eq!(wall.pos, [6000.0, 3000.0]);

    // 剛心が壁側（x=6000）へ寄ることの確認。
    let cor = center_of_rigidity(&cols);
    assert!(cor[0] > 3000.0, "Xs={} は壁側へ寄る", cor[0]);

    // n 未指定なら追加されない。
    let mut model2 = model.clone();
    model2.stress_cfg.misc_wall_n = None;
    let mut cols2 = cols[..4].to_vec();
    append_misc_wall_stiffnesses(&model2, s0, &mut cols2);
    assert_eq!(cols2.len(), 4);
}
