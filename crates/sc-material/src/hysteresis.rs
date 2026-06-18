use crate::uniaxial::UniaxialMaterial;

/// 部材レベルの履歴則パラメータ（設計書 §7 / 仕様書 §5）。
/// ファイバ要素（P5）は一軸材料（uniaxial）の積分で履歴を作るので、
/// こちらは集中ばね（one/two-component）系で使う。
///
/// 状態（履歴変数）は `HysteresisMaterial` が保持する。本 enum は不変パラメータのみ。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum HysteresisRule {
    /// 武田モデル（剛性低下型トリリニア）。
    Takeda {
        /// ひび割れ点 (Mc, θc)
        crack: (f64, f64),
        /// 降伏点 (My, θy)
        yield_point: (f64, f64),
        /// 終局点 (Mu, θu)
        ultimate: (f64, f64),
        /// 除荷剛性低下指数 α（代表 0.4〜0.5。外部設定）
        alpha: f64,
    },
    /// 武田モデル劣化版。
    TakedaDegrading {
        crack: (f64, f64),
        yield_point: (f64, f64),
        ultimate: (f64, f64),
        alpha: f64,
        /// 劣化率（ピーク耐力の低下割合）
        degradation: f64,
    },
    /// 原点指向型（せん断）。バイリニアスケルトン、除荷は原点へ。
    OriginOriented {
        /// 降伏点 (Qy, δy)
        yield_point: (f64, f64),
        /// 終局点 (Qu, δu)
        ultimate: (f64, f64),
    },
    /// スリップ型（せん断）。再載荷が原点付近で寝る（ピンチ）。
    Slip {
        yield_point: (f64, f64),
        ultimate: (f64, f64),
        /// スリップ量係数（0..1、大きいほど原点付近で剛性が落ちる）
        slip_factor: f64,
    },
}

impl HysteresisRule {
    /// スケルトン包絡線上の (力 M/Q, 接線剛性 Kt) を返す。符号対称を仮定。
    /// 変形 theta は θ(M-θ) または δ(Q-δ)。単位は [rad] or [mm]、力は [N·mm] or [N]。
    pub fn skeleton(&self, theta: f64) -> (f64, f64) {
        match *self {
            HysteresisRule::Takeda {
                crack: (mc, tc),
                yield_point: (my, ty),
                ultimate: (mu, tu),
                ..
            }
            | HysteresisRule::TakedaDegrading {
                crack: (mc, tc),
                yield_point: (my, ty),
                ultimate: (mu, tu),
                ..
            } => trilinear_symmetric(theta, tc, mc, ty, my, tu, mu),
            HysteresisRule::OriginOriented {
                yield_point: (my, ty),
                ultimate: (mu, tu),
            }
            | HysteresisRule::Slip {
                yield_point: (my, ty),
                ultimate: (mu, tu),
                ..
            } => bilinear_symmetric(theta, ty, my, tu, mu),
        }
    }

    /// 降伏点 (My, θy)。力・変形の順。
    pub fn yield_point(&self) -> (f64, f64) {
        match *self {
            HysteresisRule::Takeda { yield_point, .. }
            | HysteresisRule::TakedaDegrading { yield_point, .. }
            | HysteresisRule::OriginOriented { yield_point, .. }
            | HysteresisRule::Slip { yield_point, .. } => yield_point,
        }
    }

    /// 降伏変形 θy。
    pub fn yield_deformation(&self) -> f64 {
        self.yield_point().1
    }

    /// 降伏耐力 My。
    pub fn yield_strength(&self) -> f64 {
        self.yield_point().0
    }

    /// ひび割れ点 (Mc, θc)。武田系のみ。無い場合は None。
    pub fn crack_point(&self) -> Option<(f64, f64)> {
        match *self {
            HysteresisRule::Takeda { crack, .. }
            | HysteresisRule::TakedaDegrading { crack, .. } => Some(crack),
            _ => None,
        }
    }

    /// 終局点 (Mu, θu)。
    pub fn ultimate_point(&self) -> (f64, f64) {
        match *self {
            HysteresisRule::Takeda { ultimate, .. }
            | HysteresisRule::TakedaDegrading { ultimate, .. }
            | HysteresisRule::OriginOriented { ultimate, .. }
            | HysteresisRule::Slip { ultimate, .. } => ultimate,
        }
    }

    /// 除荷剛性（降伏後）を計算する（仕様書 §5）。
    /// Ku = Ky · (θm / θy)^(−α),  Ky = My / θy,  θm = 最大経験変形
    pub fn unloading_stiffness(&self, max_deformation: f64) -> Option<f64> {
        match *self {
            HysteresisRule::Takeda {
                yield_point: (my, theta_y),
                alpha,
                ..
            }
            | HysteresisRule::TakedaDegrading {
                yield_point: (my, theta_y),
                alpha,
                ..
            } => {
                if theta_y.abs() < 1e-15 {
                    return None;
                }
                let ky = my / theta_y;
                let ratio = (max_deformation.abs() / theta_y).max(1.0);
                Some(ky * ratio.powf(-alpha))
            }
            HysteresisRule::OriginOriented { .. } | HysteresisRule::Slip { .. } => None,
        }
    }

    fn is_takeda(&self) -> bool {
        matches!(
            *self,
            HysteresisRule::Takeda { .. } | HysteresisRule::TakedaDegrading { .. }
        )
    }
}

/// トリリニア（0→crack→yield→ultimate）の符号対称スケルトン。
fn trilinear_symmetric(
    theta: f64,
    tc: f64,
    mc: f64,
    ty: f64,
    my: f64,
    tu: f64,
    mu: f64,
) -> (f64, f64) {
    let (t, sgn) = if theta >= 0.0 {
        (theta, 1.0)
    } else {
        (-theta, -1.0)
    };
    let (m, k) = if t <= tc {
        (mc * (t / tc).max(0.0), mc / tc)
    } else if t <= ty {
        let k = (my - mc) / (ty - tc);
        (mc + k * (t - tc), k)
    } else if t <= tu {
        let k = (mu - my) / (tu - ty);
        (my + k * (t - ty), k)
    } else {
        // 終局以降は耐力保持（軟化はスケルトンに無い前提。必要なら ultimate 超過で 0）
        (mu, 0.0)
    };
    (sgn * m, k)
}

/// バイリニア（0→yield→ultimate）の符号対称スケルトン。
fn bilinear_symmetric(theta: f64, ty: f64, my: f64, tu: f64, mu: f64) -> (f64, f64) {
    let (t, sgn) = if theta >= 0.0 {
        (theta, 1.0)
    } else {
        (-theta, -1.0)
    };
    let (m, k) = if t <= ty {
        (my * (t / ty).max(0.0), my / ty)
    } else if t <= tu {
        let k = (mu - my) / (tu - ty);
        (my + k * (t - ty), k)
    } else {
        (mu, 0.0)
    };
    (sgn * m, k)
}

/// 履歴則の内部状態（runtime。シリアライズ対象外）。
#[derive(Clone, Debug, Default)]
struct HystState {
    theta: f64,
    m: f64,
    kt: f64,
    /// 直前の移動方向 (+1/-1/0)
    dir: f64,
    /// 最大経験変形とその時の力（正側・負側）
    max_pos: (f64, f64),
    max_neg: (f64, f64),
    /// 現在の分岐
    branch: Branch,
    /// 反転点
    reversal: (f64, f64),
}

#[derive(Clone, Debug, Default, PartialEq)]
enum Branch {
    #[default]
    Skeleton,
    /// 降伏後の除荷: 反転点から原点方向へ、傾き Ku。
    Unloading { ku: f64 },
    /// 再載荷: 原点(または反転点)から反対側の最大経験点(または降伏点)へ向かう直線。
    Reloading {
        origin: (f64, f64),
        target: (f64, f64),
    },
}

/// 履歴則パラメータ + 状態を持つ `UniaxialMaterial`（設計書 §6.8 集中ばね用）。
/// `trial(theta) -> (M, Kt)`。`theta` は M-θ では回転角[rad]、Q-δ では変位[mm]。
#[derive(Clone, Debug)]
pub struct HysteresisMaterial {
    pub rule: HysteresisRule,
    committed: HystState,
    trial: HystState,
}

impl HysteresisMaterial {
    pub fn new(rule: HysteresisRule) -> Self {
        Self {
            rule,
            committed: HystState::default(),
            trial: HystState::default(),
        }
    }

    /// 反対側の目標点（再載荷先）。経験が無ければ降伏点。
    fn opposite_target(&self, dir: f64) -> (f64, f64) {
        let ty = self.rule.yield_deformation();
        let my = self.rule.yield_strength();
        if dir > 0.0 {
            // 正側へ再載荷: 正側の最大経験（>0）があればそこ、無ければ降伏点
            if self.committed.max_pos.0.abs() > 1e-15 {
                self.committed.max_pos
            } else {
                (ty, my)
            }
        } else {
            if self.committed.max_neg.0.abs() > 1e-15 {
                self.committed.max_neg
            } else {
                (-ty, -my)
            }
        }
    }

    /// 降伏したか（いずれかの側で |θ| >= θy を経験）。
    fn has_yielded(&self) -> bool {
        let ty = self.rule.yield_deformation();
        self.committed.max_pos.0.abs() >= ty || self.committed.max_neg.0.abs() >= ty
    }

    /// 最大経験変形（絶対値）。
    fn max_deformation(&self) -> f64 {
        self.committed
            .max_pos
            .0
            .abs()
            .max(self.committed.max_neg.0.abs())
    }

    /// 与えた θ での trial 状態を計算（committed から branch を遷移させつつ）。
    fn evaluate(&self, theta: f64) -> HystState {
        let c = &self.committed;
        let dir_new = (theta - c.theta).signum();
        let mut s = c.clone();

        if dir_new == 0.0 {
            return s;
        }

        // 方向反転の検知 → 分岐切り替え
        let reversed = c.dir != 0.0 && dir_new != c.dir;
        if reversed {
            s.reversal = (c.theta, c.m);
            let ty = self.rule.yield_deformation();
            let yielded = c.theta.abs() >= ty || self.has_yielded();
            if c.m.abs() < 1e-12 {
                // 原点付近で反転: 除荷ではなく反対側ピークへの再載荷
                let target = self.opposite_target(dir_new);
                s.branch = Branch::Reloading {
                    origin: (c.theta, c.m),
                    target,
                };
            } else if yielded {
                let ku = self.unloading_slope(c.theta, c.m);
                s.branch = Branch::Unloading { ku };
            } else {
                // 降伏前: スケルトンに沿って戻る（弾性）
                s.branch = Branch::Skeleton;
            }
            s.dir = dir_new;
        } else {
            s.dir = dir_new;
        }

        // θ での (M, Kt) と branch 遷移
        let (m, kt, branch_out) = self.eval_on_branch(theta, &s);
        s.theta = theta;
        s.m = m;
        s.kt = kt;
        s.branch = branch_out;

        // スケルトン上で新記録時に最大経験を更新
        if matches!(s.branch, Branch::Skeleton) {
            if theta > c.max_pos.0 && theta > 0.0 {
                s.max_pos = (theta, m);
            }
            if theta < c.max_neg.0 && theta < 0.0 {
                s.max_neg = (theta, m);
            }
        }

        s
    }

    /// 反転点 (tr, mr) からの除荷剛性。
    /// 武田系: Ku = Ky·(θm/θy)^(-α)（K1 上限）。原点指向/スリップ: 原点を通る割線 mr/tr。
    fn unloading_slope(&self, tr: f64, mr: f64) -> f64 {
        let ty = self.rule.yield_deformation();
        let my = self.rule.yield_strength();
        if tr.abs() < 1e-15 {
            return my / ty;
        }
        if self.rule.is_takeda() {
            let ku = self
                .rule
                .unloading_stiffness(self.max_deformation())
                .unwrap_or(mr / tr);
            let k1 = self
                .rule
                .crack_point()
                .map(|(mc, tc)| mc / tc)
                .unwrap_or(my / ty);
            ku.min(k1)
        } else {
            // 原点指向: 反転点と原点を結ぶ割線（自然に劣化）
            (mr / tr).abs()
        }
    }

    /// 現在の branch 上で θ を評価。branch 遷移もここで処理。
    fn eval_on_branch(&self, theta: f64, s: &HystState) -> (f64, f64, Branch) {
        match s.branch {
            Branch::Skeleton => {
                let (m, k) = self.rule.skeleton(theta);
                (m, k, Branch::Skeleton)
            }
            Branch::Unloading { ku } => {
                // 反転点 (tr,mr) から傾き ku (>0) で評価。M = mr + ku·(θ - tr)。
                let (tr, mr) = s.reversal;
                let m = mr + ku * (theta - tr);
                let crossed_zero = (mr > 0.0 && m <= 0.0) || (mr < 0.0 && m >= 0.0);
                if crossed_zero {
                    // M=0 に達した点から反対側ピークへ再載荷
                    let theta_zero = tr - mr / ku;
                    let origin = (theta_zero, 0.0);
                    let target = self.opposite_target(s.dir);
                    let (m2, k2) = reload_line(&self.rule, origin, target, theta);
                    (m2, k2, Branch::Reloading { origin, target })
                } else {
                    (m, ku, Branch::Unloading { ku })
                }
            }
            Branch::Reloading { origin, target } => {
                let reached = if target.0 >= origin.0 {
                    theta >= target.0
                } else {
                    theta <= target.0
                };
                if reached {
                    let (m, k) = self.rule.skeleton(theta);
                    (m, k, Branch::Skeleton)
                } else {
                    let (m, k) = reload_line(&self.rule, origin, target, theta);
                    (m, k, Branch::Reloading { origin, target })
                }
            }
        }
    }
}

/// 再載荷直線（原点→目標）。Slip 型は原点付近の剛性を slip_factor 倍に低下（ピンチ）。
fn reload_line(
    rule: &HysteresisRule,
    origin: (f64, f64),
    target: (f64, f64),
    theta: f64,
) -> (f64, f64) {
    let dt = target.0 - origin.0;
    if dt.abs() < 1e-15 {
        return (origin.1, 0.0);
    }
    if let HysteresisRule::Slip { slip_factor, .. } = rule {
        // ピリニア再載荷: origin → (origin + slip_factor·dt, target.1) → target
        let pinch_x = origin.0 + slip_factor * dt;
        let pinch_m = target.1;
        if (theta - origin.0).abs() <= (pinch_x - origin.0).abs() {
            let k = pinch_m / (pinch_x - origin.0);
            (origin.1 + k * (theta - origin.0), k)
        } else {
            let k = (target.1 - pinch_m) / (target.0 - pinch_x);
            (pinch_m + k * (theta - pinch_x), k)
        }
    } else {
        let k = (target.1 - origin.1) / dt;
        (origin.1 + k * (theta - origin.0), k)
    }
}

impl UniaxialMaterial for HysteresisMaterial {
    fn trial(&mut self, theta: f64) -> (f64, f64) {
        let s = self.evaluate(theta);
        self.trial = s;
        (self.trial.m, self.trial.kt)
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
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn takeda() -> HysteresisRule {
        HysteresisRule::Takeda {
            crack: (40.0, 0.002),
            yield_point: (100.0, 0.01),
            ultimate: (120.0, 0.05),
            alpha: 0.4,
        }
    }

    #[test]
    fn test_skeleton_monotonic_takeda() {
        let rule = takeda();
        // 原点-ひび割れ線形
        let (m, k) = rule.skeleton(0.001);
        assert_relative_eq!(m, 20.0, epsilon = 1e-6);
        assert_relative_eq!(k, 20000.0, epsilon = 1e-3);
        // ひび割れ点
        let (m, _) = rule.skeleton(0.002);
        assert_relative_eq!(m, 40.0, epsilon = 1e-6);
        // 降伏点
        let (m, _) = rule.skeleton(0.01);
        assert_relative_eq!(m, 100.0, epsilon = 1e-6);
        // 終局点
        let (m, _) = rule.skeleton(0.05);
        assert_relative_eq!(m, 120.0, epsilon = 1e-6);
        // 負側対称
        let (m, _) = rule.skeleton(-0.01);
        assert_relative_eq!(m, -100.0, epsilon = 1e-6);
    }

    #[test]
    fn test_hysteresis_monotonic_follows_skeleton() {
        let mut mat = HysteresisMaterial::new(takeda());
        for &theta in &[0.001, 0.002, 0.005, 0.01, 0.03] {
            let (m, _) = mat.trial(theta);
            let (ms, _) = takeda().skeleton(theta);
            assert_relative_eq!(m, ms, epsilon = 1e-6);
            mat.commit();
        }
    }

    #[test]
    fn test_takeda_unloading_stiffness_degraded() {
        // 降伏後(θ=0.03)で反転 → 除荷剛性 Ku = Ky*(θm/θy)^(-α) < K1
        let mut mat = HysteresisMaterial::new(takeda());
        mat.trial(0.03);
        mat.commit();
        let (m_r, _) = mat.trial(0.029);
        let (m2, _) = mat.trial(0.028);
        let ku = (m_r - m2) / (0.029 - 0.028);
        let ky = 100.0 / 0.01;
        let expected_ku: f64 = ky * (0.03_f64 / 0.01).powf(-0.4);
        let k1 = 40.0 / 0.002;
        assert!(
            ku < k1,
            "unloading stiffness ({}) must be below elastic K1 ({})",
            ku,
            k1
        );
        assert_relative_eq!(ku, expected_ku.min(k1), epsilon = expected_ku * 0.05);
    }

    #[test]
    fn test_takeda_cyclic_returns_near_peak() {
        let mut mat = HysteresisMaterial::new(takeda());
        // +0.03 → 0 → -0.03 → 0 → +0.03
        for &theta in &[0.03, 0.0, -0.03, 0.0, 0.03] {
            mat.trial(theta);
            mat.commit();
        }
        let (m, _) = mat.trial(0.03);
        // 再載荷で正側ピークに戻る（降伏後耐力はスケルトン上 ≈ 120 に近い、少なくとも 100 超）
        assert!(
            m > 100.0,
            "cyclic reload should return near positive peak, got M={}",
            m
        );
    }

    #[test]
    fn test_origin_oriented_unloads_to_origin() {
        let rule = HysteresisRule::OriginOriented {
            yield_point: (100.0, 0.01),
            ultimate: (120.0, 0.05),
        };
        let mut mat = HysteresisMaterial::new(rule);
        mat.trial(0.03);
        mat.commit();
        mat.trial(0.0);
        mat.commit();
        let (m, _) = mat.trial(0.0);
        assert_relative_eq!(m, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_slip_pinching() {
        let rule = HysteresisRule::Slip {
            yield_point: (100.0, 0.01),
            ultimate: (120.0, 0.05),
            slip_factor: 0.5,
        };
        let mut mat = HysteresisMaterial::new(rule);
        // 降伏→反転→原点付近で M が小さい（ピンチ）
        mat.trial(0.03);
        mat.commit();
        mat.trial(0.0);
        mat.commit();
        let (m_near_zero, _) = mat.trial(0.001);
        assert!(
            m_near_zero.abs() < 30.0,
            "slip should pinch near origin, got M={}",
            m_near_zero
        );
    }

    #[test]
    fn test_commit_revert() {
        let mut mat = HysteresisMaterial::new(takeda());
        mat.trial(0.01);
        mat.commit();
        let (m1, _) = mat.trial(0.02);
        mat.revert();
        let (m2, _) = mat.trial(0.005);
        assert!(m2.abs() < m1.abs());
    }
}
