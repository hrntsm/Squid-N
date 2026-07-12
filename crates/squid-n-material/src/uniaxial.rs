use std::fmt::Debug;

/// 一軸応力–ひずみ履歴則を示すトレイト（設計書 §7）。
/// trial/commit/revert パターンで非線形解析の試行収束に対応する。
///
/// 単位規約: ひずみは無次元、応力・接線剛性は [N/mm²]。
pub trait UniaxialMaterial: Send + Sync + Debug {
    /// 試行ひずみ strain に対する (応力 [N/mm²], 接線剛性 [N/mm²])。
    /// 状態は内部に試行値として保持。
    fn trial(&mut self, strain: f64) -> (f64, f64);
    /// 試行を確定（収束後にコミット）。
    fn commit(&mut self);
    /// 試行を破棄して直前のコミット状態へ戻す（リジェクト時）。
    fn revert(&mut self);
    /// ファイバ断面などで「ファイバごとに独立した状態インスタンス」を作るための複製。
    /// 非線形履歴では各ファイバが独自の履歴変数を持つ必要があるため、
    /// 共有状態だと履歴が混入して破綮する（設計書 §6.3）。
    fn clone_box(&self) -> Box<dyn UniaxialMaterial>;
    /// チェックポイント用: 材料の全状態をバイト列へ直列化
    fn serialize_state(&self) -> Vec<u8>;
    /// チェックポイント用: バイト列から材料状態を復元
    fn deserialize_state(&mut self, data: &[u8]);
    /// 降伏値（応力またはモーメント）を外部から更新するフック。
    /// N-M 相関により降伏面の大きさを解析中に変える要素（材端集中バネ等）が
    /// 用いる。対応しない材料は何もしない（既定実装）。
    fn set_yield(&mut self, _fy: f64) {}

    /// 塑性率（ductility）評価用の参照応力 σref（RESP-D「05 非線形モデル」）。
    /// 鋼材・鉄筋は降伏強度 fy、コンクリートはシリンダー強度 fc。重み付け平均
    /// 塑性率 Jm（Σσref·A·|ε|·μi）や降伏判定の分子重みに用いる。弾性材は 0。
    fn reference_stress(&self) -> f64 {
        0.0
    }
    /// 塑性率評価用の参照ひずみ εref（RESP-D「05 非線形モデル」）。
    /// 鋼材・鉄筋は降伏ひずみ fy/E、コンクリートは圧縮強度時ひずみ |εc0|。
    /// 各ファイバの塑性率 μi = |ε|/εref、降伏判定 |ε| ≥ εref に用いる。弾性材は 0。
    fn reference_strain(&self) -> f64 {
        0.0
    }

    /// コンクリート履歴の除荷則を解析種別で切替える（RESP-D「05 非線形モデル」:
    /// 静的解析は逆行型、動的解析は原点指向型）。`dynamic=true` で原点指向型、
    /// `false` で逆行型。対応しない材料（鋼材等）は何もしない（既定実装）。
    fn set_concrete_hysteresis(&mut self, _dynamic: bool) {}
}

// ──────────────────────────── バイリニア鋼材 ────────────────────────────

/// バイリニア鋼材（弾性＋線形硬化＝kinematic hardening）。
/// 降伏点 fy [N/mm²]、ヤング率 e [N/mm²]、hardening = ひずみ硬化比（降伏後接線 = hardening·e）。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Bilinear {
    pub e: f64,
    pub fy: f64,
    pub hardening: f64,
    committed: BilinearState,
    trial: BilinearState,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
#[allow(dead_code)]
struct BilinearState {
    strain: f64,
    stress: f64,
    tangent: f64,
    plastic_strain: f64,
}

impl Bilinear {
    pub fn new(e: f64, fy: f64, hardening: f64) -> Self {
        let init = BilinearState {
            strain: 0.0,
            stress: 0.0,
            tangent: e,
            plastic_strain: 0.0,
        };
        Self {
            e,
            fy,
            hardening,
            committed: init.clone(),
            trial: init,
        }
    }

    /// 1D 線形 kinematic hardening の塑性係数 Hp。
    /// 降伏後の接線 dσ/dε = e·Hp/(e+Hp) = hardening·e となるよう Hp を定める。
    fn hp(&self) -> f64 {
        if self.hardening >= 1.0 {
            f64::INFINITY
        } else {
            self.hardening * self.e / (1.0 - self.hardening)
        }
    }
}

impl UniaxialMaterial for Bilinear {
    fn set_yield(&mut self, fy: f64) {
        // kinematic hardening の背応力・塑性ひずみは維持したまま降伏面半径のみ更新
        self.fy = fy.max(1e-9);
    }

    fn reference_stress(&self) -> f64 {
        // 実質弾性（fy を極端に大きく設定した鋼材ダミー）は塑性率評価の対象外。
        if self.fy >= 1e18 {
            0.0
        } else {
            self.fy
        }
    }

    fn reference_strain(&self) -> f64 {
        if self.fy >= 1e18 || self.e <= 0.0 {
            0.0
        } else {
            self.fy / self.e
        }
    }

    fn trial(&mut self, strain: f64) -> (f64, f64) {
        let ep = self.committed.plastic_strain;
        let hp = self.hp();
        // 弾性予測（塑性ひずみは前回コミット値で固定）
        let sigma_tr = self.e * (strain - ep);
        // 降伏関数 f = |σ - α| - fy, α = Hp·ep（kinematic hardening の背応力）
        let alpha = hp * ep;
        let f_tr = (sigma_tr - alpha).abs() - self.fy;
        if f_tr <= 0.0 {
            // 弾性
            self.trial = BilinearState {
                strain,
                stress: sigma_tr,
                tangent: self.e,
                plastic_strain: ep,
            };
        } else {
            // 塑性戻し写像: Δep = f_tr/(e+Hp), 方向 = sign(σ_tr - α)
            let sgn = (sigma_tr - alpha).signum();
            let dep = f_tr / (self.e + hp);
            let ep_new = ep + sgn * dep;
            let stress = self.e * (strain - ep_new);
            let tangent = if hp.is_infinite() {
                0.0
            } else {
                self.e * hp / (self.e + hp)
            };
            self.trial = BilinearState {
                strain,
                stress,
                tangent,
                plastic_strain: ep_new,
            };
        }
        (self.trial.stress, self.trial.tangent)
    }

    fn commit(&mut self) {
        self.committed = self.trial.clone();
    }

    fn revert(&mut self) {
        self.trial = self.committed.clone();
    }

    fn clone_box(&self) -> Box<dyn UniaxialMaterial> {
        Box::new(self.clone())
    }

    fn serialize_state(&self) -> Vec<u8> {
        bincode::serialize(self).expect("material serialize")
    }

    fn deserialize_state(&mut self, data: &[u8]) {
        if let Ok(de) = bincode::deserialize::<Self>(data) {
            *self = de;
        }
    }
}

// ──────────────────────────── Menegotto–Pinto 鉄筋 ────────────────────────────

/// Menegotto–Pinto モデル（バウシンガー効果を滑らかに表現）。
/// 仕様書 §4 の正規化形。反転点 (εr,σr)・漸近線交点 (ε0,σ0)・ξ を状態に保持。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct MenegottoPinto {
    pub e: f64,
    pub fy: f64,
    pub b: f64,
    pub r0: f64,
    pub a1: f64,
    pub a2: f64,
    committed: MpState,
    trial: MpState,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct MpState {
    strain: f64,
    stress: f64,
    tangent: f64,
    /// 直前の反転点 (εr, σr)
    eps_r: f64,
    sig_r: f64,
    /// 漸近線の交点 (ε0, σ0)
    eps_0: f64,
    sig_0: f64,
    /// 反転後の塑性ひずみ振幅パラメータ ξ
    xi: f64,
    /// 直前の trial の進行方向（+1/-1/0）。反転検知に使う。
    direction: f64,
}

impl MenegottoPinto {
    pub fn new(e: f64, fy: f64) -> Self {
        Self::with_params(e, fy, 0.01, 20.0, 18.5, 0.15)
    }

    pub fn with_params(e: f64, fy: f64, b: f64, r0: f64, a1: f64, a2: f64) -> Self {
        let init = MpState {
            strain: 0.0,
            stress: 0.0,
            tangent: e,
            eps_r: 0.0,
            sig_r: 0.0,
            // 初期漸近線交点は弾性直線と降伏後直線の交点 = (εy, fy)
            eps_0: fy / e,
            sig_0: fy,
            xi: 0.0,
            direction: 0.0,
        };
        Self {
            e,
            fy,
            b,
            r0,
            a1,
            a2,
            committed: init.clone(),
            trial: init,
        }
    }

    /// 反転点 (εr,σr) と漸近線交点 (ε0,σ0) を更新する（標準 MP 手順）。
    /// 反転点 = 直前のコミット点。新しい漸近線交点は反転点を元の交点に対して鏡映。
    fn update_reversal(&self, state: &mut MpState, prev_strain: f64, prev_stress: f64) {
        let new_eps_r = prev_strain;
        let new_sig_r = prev_stress;
        // 元の漸近線交点に対する鏡映: (ε0',σ0') = 2·(εr,σr) - (ε0,σ0)
        let new_eps_0 = 2.0 * new_eps_r - state.eps_0;
        let new_sig_0 = 2.0 * new_sig_r - state.sig_0;
        // ξ: 反転後の塑性ひずみ振幅（Chang & Mander 2004 系の簡易形）。
        //   ξ = |εr - εr_prev| / εy,  εy = fy/E。単調増加を保証。
        let eps_y = self.fy / self.e;
        let xi_new = if eps_y.abs() > 1e-15 {
            ((new_eps_r - state.eps_r).abs() / eps_y).max(state.xi)
        } else {
            state.xi
        };
        state.eps_r = new_eps_r;
        state.sig_r = new_sig_r;
        state.eps_0 = new_eps_0;
        state.sig_0 = new_sig_0;
        state.xi = xi_new;
    }
}

impl UniaxialMaterial for MenegottoPinto {
    fn reference_stress(&self) -> f64 {
        self.fy
    }

    fn reference_strain(&self) -> f64 {
        if self.e > 0.0 {
            self.fy / self.e
        } else {
            0.0
        }
    }

    fn trial(&mut self, strain: f64) -> (f64, f64) {
        let c = &self.committed;
        // 進行方向の判定と反転検知（trial 内で行う＝標準 MP）
        let dir_new = (strain - c.strain).signum();
        let mut working = c.clone();
        if dir_new != 0.0 && c.direction != 0.0 && dir_new != c.direction {
            // 反転: 直前のコミット点を新しい反転点として採用
            self.update_reversal(&mut working, c.strain, c.stress);
            working.direction = dir_new;
        } else if dir_new != 0.0 {
            working.direction = dir_new;
        }

        let deps = working.eps_0 - working.eps_r;
        let dsig = working.sig_0 - working.sig_r;
        if deps.abs() < 1e-15 {
            // 漸近線が縮退: 弾性直線で評価
            let stress = working.sig_r + self.e * (strain - working.eps_r);
            working.strain = strain;
            working.stress = stress;
            working.tangent = self.e;
            self.trial = working;
            return (stress, self.e);
        }
        let eps_star = (strain - working.eps_r) / deps;
        let r = (self.r0 - self.a1 * working.xi / (self.a2 + working.xi)).max(1.0);
        let eps_star_abs = eps_star.abs();
        let denom = (1.0 + eps_star_abs.powf(r)).powf(1.0 / r);
        let sig_star = self.b * eps_star + (1.0 - self.b) * eps_star / denom;
        let stress = working.sig_r + dsig * sig_star;
        // dσ*/dε* = b + (1-b)·(1/denom - ε*·ddenom)
        let ddenom = if eps_star_abs > 1e-15 {
            eps_star_abs.powf(r - 1.0) * eps_star.signum()
                / (1.0 + eps_star_abs.powf(r)).powf(1.0 + 1.0 / r)
        } else {
            0.0
        };
        let dsig_star = self.b + (1.0 - self.b) * (1.0 / denom - eps_star * ddenom);
        let tangent = (dsig / deps) * dsig_star;
        let tangent = tangent.clamp(self.b * self.e, self.e);

        working.strain = strain;
        working.stress = stress;
        working.tangent = tangent;
        self.trial = working;
        (stress, tangent)
    }

    fn commit(&mut self) {
        self.committed = self.trial.clone();
    }

    fn revert(&mut self) {
        self.trial = self.committed.clone();
    }

    fn clone_box(&self) -> Box<dyn UniaxialMaterial> {
        Box::new(self.clone())
    }

    fn serialize_state(&self) -> Vec<u8> {
        bincode::serialize(self).expect("material serialize")
    }

    fn deserialize_state(&mut self, data: &[u8]) {
        if let Ok(de) = bincode::deserialize::<Self>(data) {
            *self = de;
        }
    }
}

// ──────────────────────────── コンクリートモデル ────────────────────────────

/// コンクリートの一軸履歴モデル（設計書 §7）。
/// 圧縮: 放物線上昇 → ピーク(-fc at ec0) → 線形軟化 → ecu で残留(-fc·residual)。
/// 引張: 弾性 → ひび割れ(ε_cr=ft/E0) → テンションスティフニング(指数減衰)。
/// 除荷・再載荷は原点指向（最大経験ひずみへの割線）。
///
/// 単位: fc/ft [N/mm²], ec0/ecu [ひずみ(負)], tension_stiffening/residual_ratio [無次元]。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Concrete {
    pub fc: f64,
    pub ec0: f64,
    pub ecu: f64,
    pub ft: f64,
    pub tension_stiffening: f64,
    pub residual_ratio: f64,
    committed: ConcreteState,
    trial: ConcreteState,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
#[allow(dead_code)]
struct ConcreteState {
    strain: f64,
    stress: f64,
    tangent: f64,
    /// 最大経験圧縮ひずみ（最も負の値）
    max_comp_strain: f64,
    /// 最大経験引張ひずみ（最も正の値）
    max_tens_strain: f64,
    is_cracked: bool,
}

impl Concrete {
    /// 係数は AIJ 系の代表値。製品経路では with_params + 外部データを用いること（§2.2）。
    pub fn new(fc: f64, ft: f64) -> Self {
        Self::with_params(fc, -0.002, -0.0035, ft, 0.5, 0.0)
    }

    pub fn with_params(
        fc: f64,
        ec0: f64,
        ecu: f64,
        ft: f64,
        tension_stiffening: f64,
        residual_ratio: f64,
    ) -> Self {
        let init = ConcreteState {
            strain: 0.0,
            stress: 0.0,
            tangent: 0.0,
            max_comp_strain: 0.0,
            max_tens_strain: 0.0,
            is_cracked: false,
        };
        Self {
            fc,
            ec0,
            ecu,
            ft,
            tension_stiffening,
            residual_ratio,
            committed: init.clone(),
            trial: init,
        }
    }

    /// 初期接線剛性 E0 = 2·fc/|ec0|（放物線の ε=0 での接線）。
    pub fn e0(&self) -> f64 {
        2.0 * self.fc / self.ec0.abs()
    }

    /// せん断弾性率 G0 = E0/(2(1+ν))。ν=0.2（コンクリート代表値）。
    pub fn e0_shear(&self) -> f64 {
        self.e0() / (2.0 * (1.0 + 0.2))
    }

    /// 圧縮包絡線（strain ≤ 0）。
    fn envelope_compression(&self, strain: f64) -> (f64, f64) {
        if strain >= self.ec0 {
            // 上昇域: 放物線 σ = -fc·(2r - r²), r = ε/εc0
            let r = strain / self.ec0;
            let stress = -self.fc * (2.0 * r - r * r);
            let tangent = -self.fc * (2.0 - 2.0 * r) / self.ec0;
            (stress, tangent)
        } else if strain >= self.ecu {
            // 軟化域: ec0 で -fc, ecu で -fc·residual への直線
            let slope = self.fc * (1.0 - self.residual_ratio) / (self.ecu - self.ec0);
            let stress = -self.fc + slope * (strain - self.ec0);
            (stress, slope)
        } else {
            // ecu 超過: 残留一定
            (-self.fc * self.residual_ratio, 0.0)
        }
    }

    /// 引張包絡線（strain ≥ 0）。
    fn envelope_tension(&self, strain: f64) -> (f64, f64) {
        let e0 = self.e0();
        let eps_cr = self.ft / e0;
        if strain <= eps_cr {
            (e0 * strain, e0)
        } else {
            let arg = -self.tension_stiffening * (strain / eps_cr - 1.0);
            let stress = self.ft * arg.exp();
            let tangent = -self.ft * (self.tension_stiffening / eps_cr) * arg.exp();
            (stress, tangent)
        }
    }
}

impl UniaxialMaterial for Concrete {
    fn reference_stress(&self) -> f64 {
        // コンクリートの参照応力は圧縮強度 fc（塑性率 Jm の重み・降伏判定に用いる）。
        self.fc
    }

    fn reference_strain(&self) -> f64 {
        // コンクリートの参照ひずみは圧縮強度時ひずみ |εc0|。
        self.ec0.abs()
    }

    fn trial(&mut self, strain: f64) -> (f64, f64) {
        let c = &self.committed;
        let e0 = self.e0();
        let eps_cr = self.ft / e0;

        let (stress, tangent, max_comp, max_tens, is_cracked) = if strain <= 0.0 {
            // 圧縮側
            let mut max_comp = c.max_comp_strain;
            let (s, t) = if strain < c.max_comp_strain {
                // 包絡線上（新最大圧縮ひずみ）
                max_comp = strain;
                self.envelope_compression(strain)
            } else {
                // 除荷・再載荷: 原点指向の割線剛性 σm/εm
                if c.max_comp_strain < 0.0 {
                    let (sig_m, _) = self.envelope_compression(c.max_comp_strain);
                    let ku = sig_m / c.max_comp_strain;
                    (ku * strain, ku)
                } else {
                    // 圧縮履歴なし: 初期接線で原点へ
                    (e0 * strain, e0)
                }
            };
            (s, t, max_comp, c.max_tens_strain, c.is_cracked)
        } else {
            // 引張側
            let mut max_tens = c.max_tens_strain;
            let mut cracked = c.is_cracked;
            let (s, t) = if !cracked && strain <= eps_cr {
                (e0 * strain, e0)
            } else if !cracked && strain > eps_cr {
                // 初期ひび割れ発生
                cracked = true;
                max_tens = strain;
                self.envelope_tension(strain)
            } else {
                // ひび割れ後
                if strain > c.max_tens_strain {
                    max_tens = strain;
                    self.envelope_tension(strain)
                } else {
                    // 除荷・再載荷: 原点指向割線
                    if c.max_tens_strain > 0.0 {
                        let (sig_m, _) = self.envelope_tension(c.max_tens_strain);
                        let kt = sig_m / c.max_tens_strain;
                        (kt * strain, kt)
                    } else {
                        (0.0, 0.0)
                    }
                }
            };
            (s, t, c.max_comp_strain, max_tens, cracked)
        };

        self.trial = ConcreteState {
            strain,
            stress,
            tangent,
            max_comp_strain: max_comp,
            max_tens_strain: max_tens,
            is_cracked,
        };
        (stress, tangent)
    }

    fn commit(&mut self) {
        self.committed = self.trial.clone();
    }

    fn revert(&mut self) {
        self.trial = self.committed.clone();
    }

    fn clone_box(&self) -> Box<dyn UniaxialMaterial> {
        Box::new(self.clone())
    }

    fn serialize_state(&self) -> Vec<u8> {
        bincode::serialize(self).expect("material serialize")
    }

    fn deserialize_state(&mut self, data: &[u8]) {
        if let Ok(de) = bincode::deserialize::<Self>(data) {
            *self = de;
        }
    }
}

// ──────────────────────────── 既存別名（後方互換） ────────────────────────────

pub type ElasticSteel = Bilinear;
pub type ElasticConcrete = Concrete;

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_bilinear_elastic() {
        let mut s = Bilinear::new(205000.0, 235.0, 0.01);
        let (stress, t) = s.trial(0.001);
        assert_relative_eq!(stress, 205.0, epsilon = 1.0);
        assert_relative_eq!(t, 205000.0, epsilon = 1.0);
    }

    #[test]
    fn test_bilinear_yield_monotonic() {
        let mut s = Bilinear::new(205000.0, 235.0, 0.01);
        let eps_y = 235.0 / 205000.0;
        let (stress, t) = s.trial(eps_y * 5.0);
        // 降伏後: stress = fy + hardening·e·(ε - εy)
        let expected = 235.0 + 0.01 * 205000.0 * (eps_y * 5.0 - eps_y);
        assert_relative_eq!(stress, expected, epsilon = 1.0);
        assert_relative_eq!(t, 0.01 * 205000.0, epsilon = 1.0);
    }

    #[test]
    fn test_bilinear_unload_elastic() {
        let mut s = Bilinear::new(205000.0, 235.0, 0.01);
        let eps_y = 235.0 / 205000.0;
        s.trial(eps_y * 5.0);
        s.commit();
        // 除荷: 弾性勾配 E
        let (stress, t) = s.trial(eps_y * 4.0);
        assert_relative_eq!(t, 205000.0, epsilon = 1.0);
        // 応力は直前コミット点から E·Δε だけ戻る
        let prev = 235.0 + 0.01 * 205000.0 * (eps_y * 5.0 - eps_y);
        assert_relative_eq!(stress, prev - 205000.0 * eps_y, epsilon = 1.0);
    }

    #[test]
    fn test_bilinear_kinematic_bauschinger() {
        // 引張降伏→圧縮再載荷で降伏点が負側へ移動（kinematic hardening）
        let mut s = Bilinear::new(205000.0, 235.0, 0.01);
        let eps_y = 235.0 / 205000.0;
        s.trial(eps_y * 5.0);
        s.commit();
        s.trial(-eps_y * 0.5);
        s.commit();
        let (stress, _) = s.trial(-eps_y * 5.0);
        // kinematic hardening では |σ| が fy を超えてから塑性。完全弾性戻りではない。
        assert!(stress.abs() > 235.0 * 0.9, "kinematic shift expected");
    }

    #[test]
    fn test_bilinear_commit_revert() {
        let mut s = Bilinear::new(205000.0, 235.0, 0.01);
        let (s1, _) = s.trial(0.001);
        s.commit();
        let (s2, _) = s.trial(0.002);
        assert!(s2.abs() > s1.abs());
        s.revert();
        let (s3, _) = s.trial(0.0005);
        assert!(s3.abs() < s1.abs());
    }

    #[test]
    fn test_menegotto_pinto_elastic() {
        let mut mp = MenegottoPinto::new(205000.0, 235.0);
        let (stress, _) = mp.trial(0.001);
        assert_relative_eq!(stress, 205.0, epsilon = 5.0);
    }

    #[test]
    fn test_menegotto_pinto_bauschinger_loop() {
        // 繰り返し履歴でバウシンガー効果（反転後の丸み）を確認
        let mut mp = MenegottoPinto::new(205000.0, 235.0);
        let eps_y = 235.0 / 205000.0;
        let mut peak = 0.0f64;
        // +4εy → -4εy → +4εy の履歴
        for &target in &[eps_y * 4.0, -eps_y * 4.0, eps_y * 4.0] {
            let n = 20;
            for i in 1..=n {
                let eps = target * (i as f64) / (n as f64);
                let (sig, _) = mp.trial(eps);
                mp.commit();
                peak = peak.max(sig.abs());
            }
        }
        // 反転後の曲率 R は ξ 増加で小さくなり、ループは漸近線に近づく。
        // ピーク応力は fy+硬化成分 に漸近し、fy を超えること（弾完全塑性ではない）
        assert!(peak > 235.0, "MP peak should exceed fy due to hardening");
    }

    #[test]
    fn test_concrete_compression_peak() {
        let mut c = Concrete::new(30.0, 2.0);
        let (stress, t) = c.trial(-0.002);
        assert_relative_eq!(stress, -30.0, epsilon = 1e-6);
        assert_relative_eq!(t, 0.0, epsilon = 1e-3);
    }

    #[test]
    fn test_concrete_compression_initial_tangent_positive() {
        let mut c = Concrete::new(30.0, 2.0);
        let (_, t) = c.trial(-1e-9);
        let e0 = 2.0 * 30.0 / 0.002;
        assert_relative_eq!(t, e0, epsilon = 1.0);
    }

    #[test]
    fn test_concrete_softening_direction() {
        // ピーク後ひずみ進行で応力は 0 側へ（|σ| は減少）
        let mut c = Concrete::new(30.0, 2.0);
        c.trial(-0.002);
        c.commit();
        let (s_mid, _) = c.trial(-0.003);
        let (s_u, _) = c.trial(-0.0035);
        assert!(
            s_mid > -30.0 && s_u > s_mid,
            "softening must reduce |stress|: mid={}, u={}",
            s_mid,
            s_u
        );
        assert_relative_eq!(s_u, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_concrete_continuity_at_ecu() {
        let c = Concrete::new(30.0, 2.0);
        let (s_soft, _) = c.envelope_compression(-0.0035);
        let (s_res, _) = c.envelope_compression(-0.0036);
        assert_relative_eq!(s_soft, s_res, epsilon = 1e-6);
    }

    #[test]
    fn test_concrete_tension_crack_detection() {
        let mut c = Concrete::new(30.0, 2.0);
        let e0 = 2.0 * 30.0 / 0.002;
        let eps_cr = 2.0 / e0;
        // ひび割れ前: 弾性
        let (s_el, _) = c.trial(eps_cr * 0.5);
        assert_relative_eq!(s_el, e0 * eps_cr * 0.5, epsilon = 1e-9);
        // ひび割れ点: σ = ft
        let (s_cr, _) = c.trial(eps_cr);
        assert_relative_eq!(s_cr, 2.0, epsilon = 1e-6);
        // ひび割れ後: テンションスティフニング（ft から指数減衰、ただし ft 未満）
        c.commit();
        let (s_ts, _) = c.trial(eps_cr * 3.0);
        assert!(s_ts > 0.0 && s_ts < 2.0, "tension stiffening: {}", s_ts);
    }

    #[test]
    fn test_concrete_commit_revert() {
        let mut c = Concrete::new(30.0, 2.0);
        c.trial(-0.001);
        c.commit();
        c.trial(-0.002);
        c.revert();
        let (stress, _) = c.trial(-0.0005);
        assert!(stress < 0.0);
    }
}
