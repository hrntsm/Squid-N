//! 地震・風の静的解析の設定型。

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SeismicDir {
    X,
    Y,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AiMode {
    Approx,
    SemiPrecise,
}

/// 地震静的解析(Ai分布)の設定。
#[derive(Debug, Clone, Copy)]
pub struct SeismicCfg {
    pub dir: SeismicDir,
    pub mode: AiMode,
    /// 地域係数 Z（令88条）。
    pub z: f64,
    /// 地盤種別（Tc の決定に使用）。
    pub soil: squid_n_load::ai::SoilClass,
    /// 標準せん断力係数 C0（一次設計 0.2、保有 1.0）。
    pub c0: f64,
}

impl Default for SeismicCfg {
    /// 既定の Ai 算定法は略算（`AiMode::Approx`）。略算周期 T = h(0.02+0.01α)
    /// （令88条・昭55建告1793号、h: 建築物の高さ、α: 鉄骨造比）で求め、
    /// 固有値解析を要しない。精算周期（`SemiPrecise`）は固有値解析の明示実行を
    /// 前提とするオプトイン設定。
    fn default() -> Self {
        Self {
            dir: SeismicDir::X,
            // 既定は略算 T（告示式）。固有値 T は呼び出し側が明示的に指定する。
            mode: AiMode::Approx,
            z: 1.0,
            soil: squid_n_load::ai::SoilClass::II,
            c0: 0.2,
        }
    }
}

/// 風荷重の静的解析（`wind_static`）の設定。
#[derive(Debug, Clone, Copy)]
pub struct WindStaticCfg {
    pub dir: SeismicDir,
    /// 基準風速 V0 [m/s]。
    pub v0: f64,
    /// 地表面粗度区分。
    pub roughness: squid_n_load::wind::TerrainRoughness,
    /// 内圧係数 Cpi（現行の `wind_forces` 実装では風上・風下合算で相殺され
    /// 結果に影響しない。将来の片面評価用に保持する）。
    pub cpi: f64,
    /// パラペット高さ [mm]（既定 0）。平12建告1454号の建築物の高さ H（建築物の
    /// 高さと軒の高さとの平均）= GLからPH階を除く最上階の床高さ + パラペット高さの半分、に
    /// 対応する。建物高さ H にはこの半分のみを算入するが、見付面積の算定では
    /// 最上層の負担区間上端をパラペット天端（最上階床高さ + `parapet_mm`）まで
    /// 延長する（実壁はパラペット天端まで存在するため）。
    pub parapet_mm: f64,
}
