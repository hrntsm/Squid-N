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
    /// 逆行型（RESP-D「07 非線形解析（動的解析）」履歴特性）。
    /// 「常にスケルトンカーブ上を進む」。除荷・再載荷ともスケルトンを可逆に辿り、
    /// 履歴ループ（エネルギー吸収）を生じない。トリリニアスケルトン。
    Retrograde {
        crack: (f64, f64),
        yield_point: (f64, f64),
        ultimate: (f64, f64),
    },
    /// 標準型（RESP-D「07 非線形解析（動的解析）」履歴特性）。
    /// 除荷履歴は Masing 則（相似則）により決定される。除荷開始時の剛性は
    /// 初期剛性 K1 となり、除荷後の第2・第3剛性は骨格曲線の剛性低下率と同様。
    /// トリリニアスケルトン（鋼材はひび割れ点を初期弾性線上に置きバイリニア相当）。
    Standard {
        crack: (f64, f64),
        yield_point: (f64, f64),
        ultimate: (f64, f64),
    },
    /// 最大点指向型（RESP-D「07 非線形解析（動的解析）」履歴特性）。
    /// |δ|<δy1 は原点を通る勾配 K1。±δy1/δy2/最大変形を超えるとスケルトンの
    /// 第2/第3勾配上を進み、除荷・再載荷は戻り点から反対側の最大経験変形点を
    /// 直線で目指す（Clough 系のピーク指向）。トリリニアスケルトン。
    MaxPointOriented {
        crack: (f64, f64),
        yield_point: (f64, f64),
        ultimate: (f64, f64),
    },
}

impl HysteresisRule {
    /// スケルトン包絡線上の (力 M/Q, 接線剛性 Kt) を返す。符号対称を仮定。
    /// 変形 theta は θ(M-θ) または δ(Q-δ)。単位は [rad] or [mm]、力は [N·mm] or [N]。
    /// `degrade_factor` は耐力低下係数（1.0=劣化なし。TakedaDegrading で <1.0）。
    pub fn skeleton_with_degradation(&self, theta: f64, degrade_factor: f64) -> (f64, f64) {
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
            }
            | HysteresisRule::Retrograde {
                crack: (mc, tc),
                yield_point: (my, ty),
                ultimate: (mu, tu),
            }
            | HysteresisRule::Standard {
                crack: (mc, tc),
                yield_point: (my, ty),
                ultimate: (mu, tu),
            }
            | HysteresisRule::MaxPointOriented {
                crack: (mc, tc),
                yield_point: (my, ty),
                ultimate: (mu, tu),
            } => {
                let (m, k) = trilinear_symmetric(theta, tc, mc, ty, my, tu, mu);
                (m * degrade_factor, k * degrade_factor)
            }
            HysteresisRule::OriginOriented {
                yield_point: (my, ty),
                ultimate: (mu, tu),
            }
            | HysteresisRule::Slip {
                yield_point: (my, ty),
                ultimate: (mu, tu),
                ..
            } => {
                let (m, k) = bilinear_symmetric(theta, ty, my, tu, mu);
                (m * degrade_factor, k * degrade_factor)
            }
        }
    }

    /// スケルトン包絡線（劣化なし）。
    pub fn skeleton(&self, theta: f64) -> (f64, f64) {
        self.skeleton_with_degradation(theta, 1.0)
    }

    /// 劣化版の劣化率（TakedaDegrading のみ <1.0、それ以外は 1.0）。
    pub fn degradation_rate(&self) -> f64 {
        match *self {
            HysteresisRule::TakedaDegrading { degradation, .. } => degradation,
            _ => 1.0,
        }
    }

    /// 降伏点 (My, θy)。力・変形の順。
    pub fn yield_point(&self) -> (f64, f64) {
        match *self {
            HysteresisRule::Takeda { yield_point, .. }
            | HysteresisRule::TakedaDegrading { yield_point, .. }
            | HysteresisRule::OriginOriented { yield_point, .. }
            | HysteresisRule::Slip { yield_point, .. }
            | HysteresisRule::Retrograde { yield_point, .. }
            | HysteresisRule::Standard { yield_point, .. }
            | HysteresisRule::MaxPointOriented { yield_point, .. } => yield_point,
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
            | HysteresisRule::TakedaDegrading { crack, .. }
            | HysteresisRule::Retrograde { crack, .. }
            | HysteresisRule::Standard { crack, .. }
            | HysteresisRule::MaxPointOriented { crack, .. } => Some(crack),
            _ => None,
        }
    }

    /// 終局点 (Mu, θu)。
    pub fn ultimate_point(&self) -> (f64, f64) {
        match *self {
            HysteresisRule::Takeda { ultimate, .. }
            | HysteresisRule::TakedaDegrading { ultimate, .. }
            | HysteresisRule::OriginOriented { ultimate, .. }
            | HysteresisRule::Slip { ultimate, .. }
            | HysteresisRule::Retrograde { ultimate, .. }
            | HysteresisRule::Standard { ultimate, .. }
            | HysteresisRule::MaxPointOriented { ultimate, .. } => ultimate,
        }
    }

    /// 除荷剛性（降伏後）を計算する（RESP-D「07 非線形解析（動的解析）」武田型）。
    /// Kd+ = K0 · |δmax / δy2|^(−ν)。ここで K0 = 初期勾配（0→ひび割れ点の勾配 Mc/θc）、
    /// δy2 = 第2折点（降伏）変形 θy、ν = 除荷剛性低下指数（本実装の `alpha`）、
    /// δmax = 最大経験変形。従来は基準を降伏割線 Ky=My/θy としていたが、原典に合わせ
    /// 初期勾配 K0 を基準に是正した。
    pub fn unloading_stiffness(&self, max_deformation: f64) -> Option<f64> {
        match *self {
            HysteresisRule::Takeda {
                crack: (mc, tc),
                yield_point: (_my, theta_y),
                alpha,
                ..
            }
            | HysteresisRule::TakedaDegrading {
                crack: (mc, tc),
                yield_point: (_my, theta_y),
                alpha,
                ..
            } => {
                if theta_y.abs() < 1e-15 || tc.abs() < 1e-15 {
                    return None;
                }
                let k0 = mc / tc;
                let ratio = (max_deformation.abs() / theta_y).max(1.0);
                Some(k0 * ratio.powf(-alpha))
            }
            HysteresisRule::OriginOriented { .. }
            | HysteresisRule::Slip { .. }
            | HysteresisRule::Retrograde { .. }
            | HysteresisRule::Standard { .. }
            | HysteresisRule::MaxPointOriented { .. } => None,
        }
    }

    fn is_takeda(&self) -> bool {
        matches!(
            *self,
            HysteresisRule::Takeda { .. } | HysteresisRule::TakedaDegrading { .. }
        )
    }

    /// 逆行型か（除荷・再載荷ともスケルトンを辿る）。
    fn is_retrograde(&self) -> bool {
        matches!(*self, HysteresisRule::Retrograde { .. })
    }

    /// 標準型（Masing 則）か。
    fn is_standard(&self) -> bool {
        matches!(*self, HysteresisRule::Standard { .. })
    }

    /// 最大点指向型か。
    fn is_max_point_oriented(&self) -> bool {
        matches!(*self, HysteresisRule::MaxPointOriented { .. })
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
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
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
    /// ピーク到達回数（正側・負側）。TakedaDegrading の耐力劣化に使用。
    peak_count_pos: u32,
    peak_count_neg: u32,
}

impl HystState {
    /// 現在の劣化係数。各側のピーク到達回数 n に対し degrade^n（指数的劣化）。
    fn degrade_factor(&self, degradation_rate: f64) -> f64 {
        let n = self.peak_count_pos.max(self.peak_count_neg);
        degradation_rate.powi(n as i32)
    }
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
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
    /// 内側ループ（武田の規則）: ある反転点 P1 から反対側の目標点 P2 へ向かう途中で
    /// 再反転した場合、新反転点 P3 から直前の反転点 P1 へ向かう直線を描く。
    /// target は P1（直前の反転点）。次に P1 を超えるか P2 側に達したら分岐を切り替える。
    InnerLoop {
        origin: (f64, f64),
        target: (f64, f64),
        /// 外側の目標点（反対側ピーク）。内側ループ脱出後に再指向する先。
        outer_target: (f64, f64),
    },
    /// 標準型（Masing 則）の除荷・再載荷枝。反転点 (θr,Qr) からスケルトンを
    /// 2 倍相似に拡大した曲線 Q(θ)=Qr − 2·sgn(θr−θ)·g(|θr−θ|/2) を辿る。
    /// スケルトンに到達した時点で `Skeleton` へ復帰する。
    Masing { reversal: (f64, f64) },
    /// 最大点指向型のピーク指向枝。戻り点 origin から反対側の最大経験点 target へ
    /// 直線で向かう。target 到達で `Skeleton` へ復帰する。
    PeakOriented {
        origin: (f64, f64),
        target: (f64, f64),
    },
}

/// 履歴則パラメータ + 状態を持つ `UniaxialMaterial`（設計書 §6.8 集中ばね用）。
/// `trial(theta) -> (M, Kt)`。`theta` は M-θ では回転角[rad]、Q-δ では変位[mm]。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
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
        self.opposite_target_degraded(dir, 1.0)
    }

    /// 反対側の目標点（劣化係数適用）。
    fn opposite_target_degraded(&self, dir: f64, degrade: f64) -> (f64, f64) {
        let ty = self.rule.yield_deformation();
        let my = self.rule.yield_strength();
        if dir > 0.0 {
            if self.committed.max_pos.0.abs() > 1e-15 {
                (self.committed.max_pos.0, self.committed.max_pos.1 * degrade)
            } else {
                (ty, my * degrade)
            }
        } else {
            if self.committed.max_neg.0.abs() > 1e-15 {
                (self.committed.max_neg.0, self.committed.max_neg.1 * degrade)
            } else {
                (-ty, -my * degrade)
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

        // 逆行型: 常にスケルトン上（除荷・再載荷ともスケルトンを可逆に辿る）。
        if self.rule.is_retrograde() {
            let (m, kt) = self.rule.skeleton(theta);
            s.theta = theta;
            s.m = m;
            s.kt = kt;
            s.dir = dir_new;
            s.branch = Branch::Skeleton;
            if theta > s.max_pos.0 {
                s.max_pos = (theta, m);
            }
            if theta < s.max_neg.0 {
                s.max_neg = (theta, m);
            }
            return s;
        }

        // 方向反転の検知 → 分岐切り替え
        let reversed = c.dir != 0.0 && dir_new != c.dir;
        if reversed {
            s.reversal = (c.theta, c.m);
            let ty = self.rule.yield_deformation();
            let yielded = c.theta.abs() >= ty || self.has_yielded();
            if self.rule.is_standard() {
                // 標準型: Masing 則の除荷・再載荷枝（除荷開始剛性 = K1）。
                s.branch = Branch::Masing {
                    reversal: (c.theta, c.m),
                };
            } else if self.rule.is_max_point_oriented() {
                // 最大点指向型: 降伏後は戻り点から反対側の最大経験点を直線で指向。
                s.branch = if yielded {
                    Branch::PeakOriented {
                        origin: (c.theta, c.m),
                        target: self.opposite_target(dir_new),
                    }
                } else {
                    Branch::Skeleton
                };
            } else if c.m.abs() < 1e-12 {
                // 原点付近で反転: 除荷ではなく反対側ピークへの再載荷
                let target = self.opposite_target(dir_new);
                s.branch = Branch::Reloading {
                    origin: (c.theta, c.m),
                    target,
                };
            } else if matches!(c.branch, Branch::Reloading { .. })
                || matches!(c.branch, Branch::InnerLoop { .. })
            {
                // 再載荷/内側ループ中の反転 → 内側ループ（武田のポリゴン則）
                // 直前の反転点（origin）を新たな target とし、今回の反転点から指向する。
                let (prev_origin, prev_target, outer) = match c.branch {
                    Branch::Reloading { origin, target } => (origin, target, target),
                    Branch::InnerLoop {
                        origin,
                        outer_target,
                        ..
                    } => (origin, outer_target, outer_target),
                    _ => ((0.0, 0.0), (0.0, 0.0), (0.0, 0.0)),
                };
                let _ = prev_target;
                s.branch = Branch::InnerLoop {
                    origin: (c.theta, c.m),
                    target: prev_origin,
                    outer_target: outer,
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

        // スケルトン上で新記録時に最大経験を更新。ピーク到達でカウント進行（劣化版用）。
        // 降伏後（|θ| > θy）のピーク到達・再訪でカウントし、劣化を進める。
        if matches!(s.branch, Branch::Skeleton) {
            let ty = self.rule.yield_deformation();
            let beyond_yield_pos = theta > ty;
            let beyond_yield_neg = theta < -ty;
            if theta > c.max_pos.0 && beyond_yield_pos {
                s.max_pos = (theta, m);
                s.peak_count_pos = c.peak_count_pos + 1;
            } else if beyond_yield_pos
                && (theta - c.max_pos.0).abs() < ty * 1e-3
                && c.max_pos.0 > ty
            {
                s.peak_count_pos = c.peak_count_pos + 1;
                s.max_pos = (theta, m);
            }
            if theta < c.max_neg.0 && beyond_yield_neg {
                s.max_neg = (theta, m);
                s.peak_count_neg = c.peak_count_neg + 1;
            } else if beyond_yield_neg
                && (theta - c.max_neg.0).abs() < ty * 1e-3
                && c.max_neg.0 < -ty
            {
                s.peak_count_neg = c.peak_count_neg + 1;
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
        let degrade = s.degrade_factor(self.rule.degradation_rate());
        match s.branch {
            Branch::Skeleton => {
                let (m, k) = self.rule.skeleton_with_degradation(theta, degrade);
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
                    let target = self.opposite_target_degraded(s.dir, degrade);
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
                    // スケルトン到達: ピークカウントを進める（劣化版用）
                    let (m, k) = self.rule.skeleton_with_degradation(theta, degrade);
                    (m, k, Branch::Skeleton)
                } else {
                    let (m, k) = reload_line(&self.rule, origin, target, theta);
                    (m, k, Branch::Reloading { origin, target })
                }
            }
            Branch::InnerLoop {
                origin,
                target,
                outer_target,
            } => {
                // target（直前の反転点）に達したら、そこから outer_target（反対側ピーク）
                // へ向かう再載荷に切り替え（target は次の反転で更新される）。
                let reached_target = if target.0 >= origin.0 {
                    theta >= target.0
                } else {
                    theta <= target.0
                };
                if reached_target {
                    // target 点から outer_target への再載荷
                    let (m, k) = reload_line(&self.rule, target, outer_target, theta);
                    (
                        m,
                        k,
                        Branch::Reloading {
                            origin: target,
                            target: outer_target,
                        },
                    )
                } else {
                    let (m, k) = reload_line(&self.rule, origin, target, theta);
                    (
                        m,
                        k,
                        Branch::InnerLoop {
                            origin,
                            target,
                            outer_target,
                        },
                    )
                }
            }
            Branch::Masing { reversal } => {
                // 標準型 Masing 則: 反転点 (tr,qr) からスケルトンを 2 倍相似に拡大した
                // 曲線 Q(θ)=Qr − 2·sgn(θr−θ)·g(|θr−θ|/2)。除荷開始勾配は g'(0)=K1、
                // 除荷後の第2・第3勾配は骨格の剛性低下率に一致する。反射点でスケルトンへ復帰。
                let (tr, qr) = reversal;
                let arg = (tr - theta).abs() / 2.0;
                let (g_mag, g_tan) = self.rule.skeleton(arg);
                let q = qr - (tr - theta).signum() * 2.0 * g_mag;
                let rejoined = s.dir * theta >= tr.abs();
                if rejoined {
                    let (m, k) = self.rule.skeleton(theta);
                    (m, k, Branch::Skeleton)
                } else {
                    (q, g_tan.max(1e-9), Branch::Masing { reversal })
                }
            }
            Branch::PeakOriented { origin, target } => {
                // 最大点指向型: 戻り点 origin から反対側の最大経験点 target へ直線で向かい、
                // target 到達でスケルトンへ復帰する。
                let reached = if target.0 >= origin.0 {
                    theta >= target.0
                } else {
                    theta <= target.0
                };
                if reached {
                    let (m, k) = self.rule.skeleton(theta);
                    (m, k, Branch::Skeleton)
                } else {
                    let dt = target.0 - origin.0;
                    if dt.abs() < 1e-15 {
                        (origin.1, 1e-9, Branch::PeakOriented { origin, target })
                    } else {
                        let k = (target.1 - origin.1) / dt;
                        let m = origin.1 + k * (theta - origin.0);
                        (m, k.max(1e-9), Branch::PeakOriented { origin, target })
                    }
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

    fn serialize_state(&self) -> Vec<u8> {
        bincode::serialize(self).expect("material serialize")
    }

    fn deserialize_state(&mut self, data: &[u8]) {
        if let Ok(de) = bincode::deserialize::<Self>(data) {
            *self = de;
        }
    }
}

// ──────────────────────────── 辻・山田モデル ────────────────────────────

/// 辻・山田モデル（RESP-D「07 非線形解析（動的解析）」履歴特性）。
/// バイリニア骨格 + β による等方硬化/移動硬化の混合硬化則。
///
/// 塑性増分応力 Δσ を等方硬化 `Δσ̄ = β|Δσ|`（降伏幅の膨張）と移動硬化
/// `Δᾱ = (1−β)|Δσ|`（降伏幅中心の移動）へ配分する。`β=1` で等方硬化（降伏耐力が
/// 正負同時に膨張）、`β=0` で移動硬化（標準型と同等のバウシンガー効果）となる。
///
/// 単位規約は他の `UniaxialMaterial` と同じ（変形＝ひずみ or 回転、力＝応力 or
/// モーメント、剛性＝力/変形）。JFE 二重鋼管座屈補剛ブレース等で用いられる。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct TsujiYamada {
    /// 初期剛性 K1（力/変形）。
    pub k1: f64,
    /// 降伏耐力 Qy（初期降伏面の半径）。
    pub qy: f64,
    /// 第2剛性 K2（降伏後接線。0 ≤ K2 < K1）。
    pub k2: f64,
    /// 移動/等方硬化の配分 β（0..1）。
    pub beta: f64,
    committed: TyState,
    trial: TyState,
}

#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize)]
struct TyState {
    strain: f64,
    stress: f64,
    tangent: f64,
    /// 塑性変形 dp。
    dp: f64,
    /// 背応力（移動硬化の降伏面中心）α。
    alpha: f64,
    /// 等方硬化による降伏面半径の増分 Riso（R = Qy + Riso）。
    r_iso: f64,
}

impl TsujiYamada {
    pub fn new(k1: f64, qy: f64, k2: f64, beta: f64) -> Self {
        // K2 は 0 ≤ K2 < K1 にクランプ（K2≥K1 は硬化係数 H が非有限になるため）。
        let k2 = k2.clamp(0.0, k1 * 0.999);
        let init = TyState {
            tangent: k1,
            ..Default::default()
        };
        Self {
            k1,
            qy: qy.max(1e-9),
            k2,
            beta: beta.clamp(0.0, 1.0),
            committed: init,
            trial: init,
        }
    }

    /// 硬化係数 H（塑性接線）: K2 = K1·H/(K1+H) より H = K1·K2/(K1−K2)。
    fn hardening(&self) -> f64 {
        let d = self.k1 - self.k2;
        if d <= 1e-12 {
            0.0
        } else {
            self.k1 * self.k2 / d
        }
    }
}

impl UniaxialMaterial for TsujiYamada {
    fn set_yield(&mut self, fy: f64) {
        self.qy = fy.max(1e-9);
    }

    fn reference_stress(&self) -> f64 {
        self.qy
    }

    fn reference_strain(&self) -> f64 {
        if self.k1 > 0.0 {
            self.qy / self.k1
        } else {
            0.0
        }
    }

    fn trial(&mut self, strain: f64) -> (f64, f64) {
        let c = self.committed;
        let h = self.hardening();
        let q_tr = self.k1 * (strain - c.dp);
        let r = self.qy + c.r_iso;
        let f = (q_tr - c.alpha).abs() - r;
        if f <= 0.0 {
            self.trial = TyState {
                strain,
                stress: q_tr,
                tangent: self.k1,
                ..c
            };
        } else {
            let s = (q_tr - c.alpha).signum();
            let d_dp = f / (self.k1 + h);
            let dp_new = c.dp + s * d_dp;
            // 移動硬化（背応力）と等方硬化（降伏面膨張）へ配分。
            let alpha_new = c.alpha + (1.0 - self.beta) * h * s * d_dp;
            let r_iso_new = c.r_iso + self.beta * h * d_dp;
            let stress = self.k1 * (strain - dp_new);
            let tangent = self.k1 * h / (self.k1 + h);
            self.trial = TyState {
                strain,
                stress,
                tangent,
                dp: dp_new,
                alpha: alpha_new,
                r_iso: r_iso_new,
            };
        }
        (self.trial.stress, self.trial.tangent)
    }

    fn commit(&mut self) {
        self.committed = self.trial;
    }

    fn revert(&mut self) {
        self.trial = self.committed;
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

// ──────────────────────────── 鉄骨大梁の座屈考慮履歴 ────────────────────────────

/// 横座屈で耐力が決まる H 形鋼梁の最大曲げ耐力比 `Mu/Mp`
/// （RESP-D「07 非線形解析（動的解析）」鉄骨大梁の座屈を考慮した履歴、
/// 井戸田ほか 2015 の式）。
///
/// - `lambda_b`: 横座屈細長比 λb（基準化）。
/// - `kappa`: 曲げモーメント勾配（端部モーメント比、−1≤κ≤0 で複曲率〜片持ち）。
/// - `w_f`: フランジ幅厚比パラメータ（`WF`）。
/// - `e_lambda_b`: 弾性限界細長比 `eλb`。
///
/// 係数は原典既定: `cres=0.0`（残留応力）、`f=1.0`（形状係数）、`kres=0.3`、`kdef=1.0`。
pub fn lateral_buckling_mu_ratio(lambda_b: f64, kappa: f64, w_f: f64, e_lambda_b: f64) -> f64 {
    let lambda_b = lambda_b.max(1e-6);
    let e_lambda_b = e_lambda_b.max(1e-6);
    let kappa = kappa.clamp(-1.0, 1.0);
    const C_RES: f64 = 0.0;
    const K_DEF: f64 = 1.0;
    // qκ・r・αΛ（原典の区分式）。
    let q_kappa = if kappa <= 0.0 {
        -0.1 * kappa + 0.065
    } else {
        0.065
    };
    let r = if kappa <= 0.0 { 0.5 * kappa + 1.0 } else { 1.0 };
    let alpha_lambda = -0.2 * kappa - 0.25;
    // 変形性能指標 Λc' = ((λb/eλb) + WF/3)^(1/3)。
    let lambda_c = ((lambda_b / e_lambda_b) + w_f / 3.0).max(0.0).cbrt();
    // 歪硬化による耐力上昇率 h0。
    let h0 = if lambda_c <= 1.25 {
        alpha_lambda * lambda_c * (lambda_c - 1.25) + 1.0
    } else {
        1.0
    };
    // 初期たわみ係数 cdef = qκ·kdef·r。
    let c_def = q_kappa * K_DEF * r;
    let a = 1.0 + c_def * lambda_b + (1.0 + C_RES) * lambda_b * lambda_b;
    let disc = (a * a - 4.0 * lambda_b * lambda_b * (1.0 + C_RES * lambda_b * lambda_b)).max(0.0);
    let denom = a + disc.sqrt();
    if denom <= 1e-12 {
        1.0
    } else {
        (2.0 * h0 / denom).clamp(0.05, 5.0)
    }
}

/// 鉄骨大梁の座屈を考慮した履歴則（RESP-D「07」）。曲げモーメント–回転角 `M–θ`。
///
/// 骨格は 弾性 → 全塑性 `Mp` → 歪硬化で最大耐力 `Mu`（`Mu/Mp=mu_ratio`）→
/// 劣化開始 `θ_static` から負勾配で残留耐力 `Mu·mu_res` へ低下、の耐力劣化型。
/// 除荷は孟・大井・高梨の **RO モデル**（γ=5, Φ=0.5）で表す（反転点から初期剛性 `k1`
/// で立ち上がり、RO 式で滑らかに軟化）。再載荷は経験最大点指向で骨格へ復帰する
/// （原典の完全な繰返し則の簡略化）。
///
/// 局部座屈／横座屈／連成座屈で `Mu`・`θ_static` が異なるが、本モデルはそれらを
/// パラメータとして受け取る（`Mu` は [`lateral_buckling_mu_ratio`] 等で算定）。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct SteelBuckling {
    /// 初期（弾性）剛性 k1 [力/回転角]。
    pub k1: f64,
    /// 全塑性耐力 Mp。
    pub mp: f64,
    /// 最大耐力 Mu = mu_ratio·Mp（mu_ratio≥1）。
    pub mu: f64,
    /// 最大耐力に至る回転角 θu。
    pub theta_u: f64,
    /// 耐力劣化開始の回転角 θ_static（≥θu）。
    pub theta_static: f64,
    /// 残留耐力に至る回転角 θ_res（>θ_static）。
    pub theta_res: f64,
    /// 残留耐力比（Mu に対する。0<mu_res≤1）。
    pub mu_res: f64,
    committed: SbState,
    trial: SbState,
}

#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize)]
struct SbState {
    theta: f64,
    m: f64,
    tangent: f64,
    /// 経験最大回転角（正）。
    theta_max_pos: f64,
    /// 経験最大回転角（負）。
    theta_max_neg: f64,
    /// 直近の反転点。
    theta_r: f64,
    m_r: f64,
    /// 直近の進行方向（+1/-1/0）。
    dir: f64,
    /// 骨格上にいるか（除荷・再載荷中でない）。
    on_backbone: bool,
}

impl SteelBuckling {
    /// RO モデル諸元（原典既定 γ=5, Φ=0.5）。
    const RO_GAMMA: f64 = 5.0;
    const RO_PHI: f64 = 0.5;

    /// 詳細指定のコンストラクタ。
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        k1: f64,
        mp: f64,
        mu_ratio: f64,
        theta_u: f64,
        theta_static: f64,
        theta_res: f64,
        mu_res: f64,
    ) -> Self {
        let k1 = k1.max(1e-9);
        let mp = mp.max(1e-9);
        let mu = mp * mu_ratio.max(1.0);
        let theta_y = mp / k1;
        // θy < θu ≤ θ_static < θ_res を保証。
        let theta_u = theta_u.max(theta_y * 1.0001);
        let theta_static = theta_static.max(theta_u);
        let theta_res = theta_res.max(theta_static * 1.0001);
        let init = SbState {
            tangent: k1,
            on_backbone: true,
            ..Default::default()
        };
        Self {
            k1,
            mp,
            mu,
            theta_u,
            theta_static,
            theta_res,
            mu_res: mu_res.clamp(0.05, 1.0),
            committed: init,
            trial: init,
        }
    }

    /// 既定諸元（θu=2θy, θ_static=4θy, θ_res=10θy, mu_res=0.5）。
    pub fn with_defaults(k1: f64, mp: f64, mu_ratio: f64) -> Self {
        let theta_y = mp.max(1e-9) / k1.max(1e-9);
        Self::new(
            k1,
            mp,
            mu_ratio,
            2.0 * theta_y,
            4.0 * theta_y,
            10.0 * theta_y,
            0.5,
        )
    }

    /// 骨格（奇対称）。回転角 θ に対する (M, 接線)。
    fn envelope(&self, theta: f64) -> (f64, f64) {
        let s = theta.signum();
        let t = theta.abs();
        let theta_y = self.mp / self.k1;
        let (m, k) = if t <= theta_y {
            (self.k1 * t, self.k1)
        } else if t <= self.theta_u {
            // 歪硬化: Mp → Mu。
            let kh = (self.mu - self.mp) / (self.theta_u - theta_y);
            (self.mp + kh * (t - theta_y), kh)
        } else if t <= self.theta_static {
            // 最大耐力で頭打ち（プラトー）。
            (self.mu, 0.0)
        } else if t <= self.theta_res {
            // 耐力劣化: Mu → Mu·mu_res。
            let kdeg = (self.mu_res * self.mu - self.mu) / (self.theta_res - self.theta_static);
            (self.mu + kdeg * (t - self.theta_static), kdeg)
        } else {
            (self.mu_res * self.mu, 0.0)
        };
        (s * m, k)
    }

    /// 反転点 (θr, Mr) からの RO 除荷枝。回転角 θ に対する (M, 接線)。
    /// RO: Δθ = (ΔM/k1)·(1 + Φ·|ΔM/Mp|^(γ−1))。ΔM を Newton で解く。
    fn ro_branch(&self, theta: f64, theta_r: f64, m_r: f64) -> (f64, f64) {
        let dtheta = theta - theta_r;
        let mut dm = self.k1 * dtheta; // 線形初期推定。
        let g = Self::RO_GAMMA;
        let phi = Self::RO_PHI;
        for _ in 0..30 {
            let ratio = (dm / self.mp).abs();
            let f = (dm / self.k1) * (1.0 + phi * ratio.powf(g - 1.0)) - dtheta;
            let fp = (1.0 / self.k1) * (1.0 + phi * g * ratio.powf(g - 1.0));
            if fp.abs() < 1e-30 {
                break;
            }
            let step = f / fp;
            dm -= step;
            if step.abs() < 1e-9 * self.mp.max(1.0) {
                break;
            }
        }
        let ratio = (dm / self.mp).abs();
        let tangent = self.k1 / (1.0 + phi * g * ratio.powf(g - 1.0));
        (m_r + dm, tangent.max(1e-6))
    }
}

impl UniaxialMaterial for SteelBuckling {
    fn set_yield(&mut self, fy: f64) {
        // Mp 更新に伴い Mu も比率を保って更新。
        let ratio = if self.mp > 0.0 {
            self.mu / self.mp
        } else {
            1.0
        };
        self.mp = fy.max(1e-9);
        self.mu = self.mp * ratio;
    }

    fn reference_stress(&self) -> f64 {
        self.mp
    }

    fn reference_strain(&self) -> f64 {
        if self.k1 > 0.0 {
            self.mp / self.k1
        } else {
            0.0
        }
    }

    fn trial(&mut self, theta: f64) -> (f64, f64) {
        let c = self.committed;
        let dir = (theta - c.theta).signum();
        if dir == 0.0 {
            self.trial = c;
            return (c.m, c.tangent);
        }
        // 骨格更新の判定: その方向の経験最大を超えて進む → 骨格上。
        let beyond_pos = dir > 0.0 && theta >= c.theta_max_pos;
        let beyond_neg = dir < 0.0 && theta <= c.theta_max_neg;
        let mut st = c;
        st.dir = dir;
        if beyond_pos || beyond_neg {
            // 骨格。
            let (m, k) = self.envelope(theta);
            st.m = m;
            st.tangent = k;
            st.theta = theta;
            st.on_backbone = true;
            st.theta_max_pos = st.theta_max_pos.max(theta);
            st.theta_max_neg = st.theta_max_neg.min(theta);
            self.trial = st;
            return (m, k);
        }
        // 除荷・再載荷: 反転直後（骨格からの離脱／方向反転）に反転点を更新。
        if c.on_backbone || (c.dir != 0.0 && dir != c.dir) {
            st.theta_r = c.theta;
            st.m_r = c.m;
        }
        st.on_backbone = false;
        // 反転点からの RO 除荷・再載荷枝。経験最大点への復帰は上の beyond_pos/neg 判定が
        // 担う（θ がその方向の経験最大を超えると骨格へ戻る）ため、ここでは骨格クランプは
        // 行わない（プラトー骨格からの除荷が誤って骨格へ張り付くのを避ける）。
        let (m, k) = self.ro_branch(theta, st.theta_r, st.m_r);
        st.m = m;
        st.tangent = k;
        st.theta = theta;
        self.trial = st;
        (m, k)
    }

    fn commit(&mut self) {
        self.committed = self.trial;
    }

    fn revert(&mut self) {
        self.trial = self.committed;
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
        // RESP-D 武田型: Kd+ = K0·|δmax/δy2|^(−ν), K0 = Mc/θc（初期勾配）, δy2 = θy, ν = α。
        let k0 = 40.0 / 0.002;
        let expected_ku: f64 = k0 * (0.03_f64 / 0.01).powf(-0.4);
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

    #[test]
    fn test_takeda_inner_loop_polygon() {
        // 外側ループ途中で反転 → 内側ループ → 再反転でポリゴン則。
        let mut mat = HysteresisMaterial::new(takeda());
        // +0.03（降伏後ピーク）→ 0.01（除荷途中・M=0 未満）→ -0.005（内側反転）
        mat.trial(0.03);
        mat.commit();
        mat.trial(0.01);
        mat.commit();
        // 内側ループに入る反転
        let (m_inner, _) = mat.trial(-0.005);
        mat.commit();
        // 再反転で target（直前の反転点 θ=0.01 側）へ向かう直線
        let (m_back, _) = mat.trial(0.008);
        // 内側ループの戻りは、ピーク直前の反転点付近の M に近い値を取る
        assert!(
            m_back > m_inner,
            "inner loop should return toward previous reversal point: inner={}, back={}",
            m_inner,
            m_back
        );
    }

    #[test]
    fn test_takeda_degrading_peak_reduction() {
        // TakedaDegrading: ピーク到達ごとに耐力が劣化する。
        let rule = HysteresisRule::TakedaDegrading {
            crack: (40.0, 0.002),
            yield_point: (100.0, 0.01),
            ultimate: (120.0, 0.05),
            alpha: 0.4,
            degradation: 0.9, // 1ピークごとに 10% 低下
        };
        let mut mat = HysteresisMaterial::new(rule);
        // 1 サイクル目: +0.03 → -0.03（ピーク到達 2 回）
        mat.trial(0.03);
        mat.commit();
        mat.trial(0.0);
        mat.commit();
        mat.trial(-0.03);
        mat.commit();
        // 2 サイクル目の正側ピーク再到達で耐力が低下しているか
        mat.trial(0.0);
        mat.commit();
        let (m_peak2, _) = mat.trial(0.03);
        // 初回ピークは 110（skeleton at 0.03）。劣化後は 110*0.9^n 系で低下。
        assert!(
            m_peak2 < 110.0,
            "degrading model should reduce peak on 2nd cycle: m={}",
            m_peak2
        );
    }

    fn retrograde() -> HysteresisRule {
        HysteresisRule::Retrograde {
            crack: (40.0, 0.002),
            yield_point: (100.0, 0.01),
            ultimate: (120.0, 0.05),
        }
    }

    fn standard() -> HysteresisRule {
        HysteresisRule::Standard {
            crack: (40.0, 0.002),
            yield_point: (100.0, 0.01),
            ultimate: (120.0, 0.05),
        }
    }

    fn max_point() -> HysteresisRule {
        HysteresisRule::MaxPointOriented {
            crack: (40.0, 0.002),
            yield_point: (100.0, 0.01),
            ultimate: (120.0, 0.05),
        }
    }

    #[test]
    fn test_retrograde_traces_skeleton_both_ways() {
        // 逆行型: 除荷・再載荷ともスケルトンを可逆に辿る（履歴ループなし）。
        let mut mat = HysteresisMaterial::new(retrograde());
        mat.trial(0.03);
        mat.commit();
        // 除荷: 力はスケルトン値に一致（除荷枝を描かない）。
        let (m, _) = mat.trial(0.02);
        let (ms, _) = retrograde().skeleton(0.02);
        assert_relative_eq!(m, ms, epsilon = 1e-6);
        mat.commit();
        // 原点まで戻れば力は 0（エネルギー吸収なし）。
        let (m0, _) = mat.trial(0.0);
        assert_relative_eq!(m0, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_standard_masing_unload_starts_at_initial_stiffness() {
        // 標準型: 除荷開始剛性は初期剛性 K1 = Mc/θc = 20000。
        let mut mat = HysteresisMaterial::new(standard());
        mat.trial(0.03); // スケルトン上 (110, 0.03)
        mat.commit();
        let peak = 110.0_f64;
        let (m1, k1) = mat.trial(0.0299); // 反転 → Masing 枝
        let k1_expected = 40.0 / 0.002;
        let slope = (peak - m1) / (0.03 - 0.0299);
        assert_relative_eq!(slope, k1_expected, epsilon = k1_expected * 0.05);
        assert_relative_eq!(k1, k1_expected, epsilon = k1_expected * 0.1);
    }

    #[test]
    fn test_standard_masing_reaches_opposite_skeleton() {
        // 標準型: 反射点（反対側 |θ|≥|反転点|）でスケルトンへ復帰する。
        let mut mat = HysteresisMaterial::new(standard());
        mat.trial(0.03);
        mat.commit();
        for &t in &[0.02, 0.0, -0.02, -0.03] {
            mat.trial(t);
            mat.commit();
        }
        let (m, _) = mat.trial(-0.03);
        let (ms, _) = standard().skeleton(-0.03);
        assert_relative_eq!(m, ms, epsilon = 2.0);
        // 途中（θ=0）は履歴枝上にあり、原点指向型のように 0 にはならない。
        let mut mat2 = HysteresisMaterial::new(standard());
        mat2.trial(0.03);
        mat2.commit();
        let (m_zero, _) = mat2.trial(0.0);
        assert!(
            m_zero < -1.0,
            "Masing loop should carry negative force at θ=0, got {}",
            m_zero
        );
    }

    #[test]
    fn test_max_point_oriented_targets_opposite_peak() {
        // 最大点指向型: 戻り点から反対側の最大経験点を直線で指向する。
        let mut mat = HysteresisMaterial::new(max_point());
        mat.trial(0.03); // +ピーク (110, 0.03)
        mat.commit();
        for &t in &[0.0, -0.01, -0.025] {
            mat.trial(t);
            mat.commit();
        }
        // -0.025 から再載荷（反転）→ +ピークを直線で指向。
        mat.trial(-0.02);
        mat.commit();
        let (m_mid, _) = mat.trial(0.0);
        let (m_end, _) = mat.trial(0.03);
        assert!(m_end > 100.0, "should reach positive peak, got {}", m_end);
        assert!(
            m_mid > -110.0 && m_mid < m_end,
            "peak-oriented interpolation mid={}",
            m_mid
        );
    }

    #[test]
    fn test_new_rules_skeleton_matches_takeda_trilinear() {
        // 新規則のスケルトンは武田と同じトリリニア（骨格は履歴則に依存しない）。
        for rule in [retrograde(), standard(), max_point()] {
            let (m, _) = rule.skeleton(0.01);
            assert_relative_eq!(m, 100.0, epsilon = 1e-6);
            let (m2, _) = rule.skeleton(-0.002);
            assert_relative_eq!(m2, -40.0, epsilon = 1e-6);
        }
    }

    #[test]
    fn test_tsuji_yamada_monotonic_bilinear() {
        // 単調載荷は K2 のバイリニア骨格を辿る。K1=1000, Qy=100, K2=100, δy=0.1。
        let mut m = TsujiYamada::new(1000.0, 100.0, 100.0, 0.5);
        let (s_el, t_el) = m.trial(0.05);
        assert_relative_eq!(s_el, 50.0, epsilon = 1e-6);
        assert_relative_eq!(t_el, 1000.0, epsilon = 1e-6);
        m.commit();
        let (s, t) = m.trial(0.3);
        // Qy + K2·(δ − δy) = 100 + 100·0.2 = 120。
        assert_relative_eq!(s, 120.0, epsilon = 1e-6);
        assert_relative_eq!(t, 100.0, epsilon = 1e-6);
    }

    #[test]
    fn test_tsuji_yamada_isotropic_grows_beta1() {
        // β=1（等方硬化）: 大振幅を経験すると降伏耐力が正負同時に膨張し、
        // 同一変形での再載荷応力が初回より増大する。
        let mut m = TsujiYamada::new(1000.0, 100.0, 100.0, 1.0);
        let (s1, _) = m.trial(0.3);
        m.commit();
        for &d in &[0.0, -0.3, 0.0] {
            m.trial(d);
            m.commit();
        }
        let (s2, _) = m.trial(0.3);
        assert!(
            s2 > s1 + 10.0,
            "isotropic hardening should raise reload force: first={s1}, second={s2}"
        );
    }

    #[test]
    fn test_tsuji_yamada_kinematic_bauschinger_beta0() {
        // β=0（移動硬化）: 降伏面は膨張せず中心（背応力 α）が移動する。
        // +方向降伏（δ=0.3, α=20）後、除荷は弾性域 2Qy/K1=0.20 を経て δ=0.10 で
        // 逆方向降伏する（バウシンガー効果）。
        let mut m = TsujiYamada::new(1000.0, 100.0, 100.0, 0.0);
        m.trial(0.3);
        m.commit();
        // δ=0.15 はまだ弾性（初期剛性）。
        let (_s, t_mid) = m.trial(0.15);
        assert_relative_eq!(t_mid, 1000.0, epsilon = 1e-6);
        // δ=0.05 では逆方向に塑性化（第2剛性・圧縮応力）。
        let (s_rev, t_rev) = m.trial(0.05);
        assert_relative_eq!(t_rev, 100.0, epsilon = 1e-6);
        assert!(s_rev < 0.0, "reverse plastic in compression: {s_rev}");
    }

    #[test]
    fn test_tsuji_yamada_commit_revert() {
        let mut m = TsujiYamada::new(1000.0, 100.0, 100.0, 0.5);
        m.trial(0.2);
        m.commit();
        let (s1, _) = m.trial(0.4);
        m.revert();
        let (s2, _) = m.trial(0.25);
        assert!(s2 < s1);
    }

    #[test]
    fn test_lateral_buckling_mu_ratio_slender_reduces() {
        // 細長比が大きいほど Mu/Mp は小さくなる（横座屈で耐力低下）。
        let stocky = lateral_buckling_mu_ratio(0.3, 0.0, 0.5, 0.3);
        let slender = lateral_buckling_mu_ratio(1.5, 0.0, 0.5, 0.3);
        assert!(stocky > slender, "stocky={stocky}, slender={slender}");
        assert!(slender > 0.0 && stocky <= 5.0);
    }

    #[test]
    fn test_steel_buckling_backbone_peak_then_degrade() {
        // 骨格: 弾性→硬化→最大耐力 Mu→劣化。単調載荷で Mu 到達後に耐力低下。
        let k1 = 1000.0;
        let mp = 100.0;
        let mut m = SteelBuckling::with_defaults(k1, mp, 1.3);
        let theta_y = mp / k1;
        // 弾性点。
        let (m_el, _) = m.trial(0.5 * theta_y);
        m.commit();
        assert_relative_eq!(m_el, 0.5 * mp, epsilon = 1e-6);
        // ピーク（θu=2θy 付近）。
        let (m_peak, _) = m.trial(2.0 * theta_y);
        m.commit();
        assert_relative_eq!(m_peak, 1.3 * mp, epsilon = 1e-3);
        // 劣化域（θ_res=10θy 手前）。耐力が Mu より低下。
        let (m_deg, _) = m.trial(8.0 * theta_y);
        m.commit();
        assert!(
            m_deg < m_peak && m_deg > 0.5 * 1.3 * mp * 0.99,
            "degradation: peak={m_peak}, deg={m_deg}"
        );
    }

    #[test]
    fn test_steel_buckling_ro_unload_initial_stiffness() {
        // RO 除荷は反転点で初期剛性 k1 から立ち上がる。
        let k1 = 1000.0;
        let mp = 100.0;
        let mut m = SteelBuckling::with_defaults(k1, mp, 1.2);
        let theta_y = mp / k1;
        m.trial(3.0 * theta_y);
        m.commit();
        // 反転直後の微小除荷: 接線 ≈ k1。
        let (_, k) = m.trial(3.0 * theta_y - 1e-6 * theta_y);
        assert_relative_eq!(k, k1, epsilon = k1 * 0.02);
    }

    #[test]
    fn test_steel_buckling_hysteretic_energy_positive() {
        // 1 サイクルで履歴ループ面積（散逸エネルギー）が正。
        let k1 = 1000.0;
        let mp = 100.0;
        let mut m = SteelBuckling::with_defaults(k1, mp, 1.2);
        let theta_y = mp / k1;
        let amp = 3.0 * theta_y;
        let path: Vec<f64> = (0..=80)
            .map(|i| {
                let phase = i as f64 / 20.0 * std::f64::consts::PI;
                amp * phase.sin()
            })
            .collect();
        let mut energy = 0.0;
        let mut prev = (0.0, 0.0);
        for &th in &path {
            let (mm, _) = m.trial(th);
            m.commit();
            energy += 0.5 * (prev.1 + mm) * (th - prev.0);
            prev = (th, mm);
        }
        assert!(
            energy > 0.0,
            "dissipated energy should be positive: {energy}"
        );
    }

    #[test]
    fn test_steel_buckling_commit_revert() {
        // 硬化域（θy=0.1 < θ < θu=0.2）で単調に耐力増加する区間を用いる。
        let mut m = SteelBuckling::with_defaults(1000.0, 100.0, 1.2);
        m.trial(0.12);
        m.commit();
        let (s1, _) = m.trial(0.18);
        m.revert();
        let (s2, _) = m.trial(0.13);
        assert!(
            s2 < s1,
            "revert then smaller θ → smaller M: s1={s1}, s2={s2}"
        );
    }
}
