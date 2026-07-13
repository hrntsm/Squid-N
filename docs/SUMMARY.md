# Summary

[はじめに](./introduction.md)

# 概要

- [アーキテクチャ](./architecture.md)

# 計算根拠（理論・出典）

- [計算根拠について](./calc_basis/README.md)
- [1. 荷重・外力](./calc_basis/01_荷重.md)
- [2. 材料構成則・材料強度](./calc_basis/02_材料.md)
- [3. 断面性能](./calc_basis/03_断面性能.md)
- [4. 要素の定式化・剛性](./calc_basis/04_要素剛性.md)
- [5. 構造解析](./calc_basis/05_構造解析.md)
- [6. 一次設計（許容応力度計算）](./calc_basis/06_一次設計.md)
- [7. 二次設計（保有水平耐力計算）](./calc_basis/07_二次設計.md)
- [8. 部材終局耐力](./calc_basis/08_部材終局耐力.md)
- [9. 限界耐力計算](./calc_basis/09_限界耐力計算.md)
- [10. 免震・制振](./calc_basis/10_免震制振.md)

# 設計仕様（specs）

- [仕様書について](./specs/README.md)
- [実装設計書（開発指示書）](./specs/構造計算一貫プログラム_実装設計書.md)
- [P0 基盤](./specs/P0_基盤.md)
- [P1 線形要素](./specs/P1_線形要素.md)
- [P1.5 板要素（MITC4 シェル）](./specs/P1.5_板要素.md)
- [P2 線形解析＋荷重](./specs/P2_線形解析と荷重.md)
- [P3 最小UI／設計](./specs/P3_最小UIと設計.md)
- [P4 材料・断面](./specs/P4_材料と断面.md)
- [P5 非線形](./specs/P5_非線形.md)
- [P5.5 壁・MS 要素](./specs/P5.5_壁とMS.md)
- [P6 動的（時刻歴応答）](./specs/P6_動的.md)
- [P7 二次設計](./specs/P7_二次設計.md)
- [P8 操作・連携](./specs/P8_操作と連携.md)
- [P9 仕上げ（V&V・配布）](./specs/P9_仕上げ.md)
- [P10 GPU 高速化](./specs/P10_GPU高速化.md)
- [P11 ML 断面提案](./specs/P11_ML断面提案.md)
- [P12 限界耐力計算](./specs/P12_限界耐力計算.md)
- [UI 設計（横断）](./specs/UI設計.md)
- [原典照合リスト](./specs/原典照合リスト.md)

# 検証（V&V）

- [V&V について](./v_and_v/README.md)
- [剛性計算 RESP-D 照合](./v_and_v/剛性計算_RESP-D照合.md)
- [応力解析 RESP-D 照合](./v_and_v/応力解析_RESP-D照合.md)
- [断面検定 RESP-D 照合](./v_and_v/断面検定_RESP-D照合.md)
- [終局検定 RESP-D 照合](./v_and_v/終局検定_RESP-D照合.md)
- [非線形モデル RESP-D 照合](./v_and_v/非線形モデル_RESP-D照合.md)
- [非線形動的解析 RESP-D 照合](./v_and_v/非線形動的解析_RESP-D照合.md)
- [荷重計算レビュー](./v_and_v/load_calculation_review.md)
- [P3 レビュー](./v_and_v/p3_review.md)
- [P4 レビュー](./v_and_v/p4_review.md)
- [P7 監査（保有水平耐力）](./v_and_v/p7_review.md)
- [P8 監査（MCP・ST-Bridge・GUI）](./v_and_v/p8_review.md)
- [P9 未完了項目](./v_and_v/pending_items.md)

# 開発運用

- [ロードマップ](./ROADMAP.md)
- [申し送り](./申し送り.md)
- [UI 導線改善](./UI導線改善.md)
- [トンマナガイド（TONMANUAL）](./TONMANUAL.md)
