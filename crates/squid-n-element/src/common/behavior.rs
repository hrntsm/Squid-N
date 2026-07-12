use smallvec::SmallVec;
use squid_n_core::dof::DofMap;
use squid_n_core::model::Model;
use std::any::Any;

pub struct LocalMat {
    pub n: usize,
    pub data: Vec<f64>,
}

pub struct LocalVec {
    pub data: SmallVec<[f64; 24]>,
}

pub struct Ctx<'a> {
    pub model: &'a Model,
}

#[derive(Clone, Debug, Default)]
pub struct ElemState {}

#[derive(Clone, Copy)]
pub enum MassOption {
    Lumped,
    Consistent,
}

/// 塑性率（ductility）評価用の危険断面プローブ（RESP-D「05 非線形モデル」
/// ファイバーモデルの塑性率）。ファイバー要素が最大曲率のガウス点（危険断面）
/// について現在のひずみ状態を集約して返す。プッシュオーバー解析
/// （`squid_n_solver::pushover`）が各ステップで参照し、塑性率基点曲率と
/// 最大応答曲率から部材塑性率 μ を算定する。
#[derive(Clone, Copy, Debug, Default)]
pub struct DuctilityProbe {
    /// 危険断面の曲率の大きさ |κ| = √(κy²+κz²) [1/mm]。
    pub curvature: f64,
    /// 断面内の最大引張ひずみ（正）。
    pub max_tension_strain: f64,
    /// 断面内の最大圧縮ひずみの大きさ（正で返す）。
    pub max_compression_strain: f64,
    /// 各ファイバの塑性率 μi=|ε|/εref の最大値（≥1 で降伏＝塑性率基点方式(3)）。
    pub max_yield_ratio: f64,
    /// 重み付け平均塑性率 Jm = Σσref·A·|ε|·μi / Σσref·A·|ε|（≥1 で基点＝方式(2)）。
    pub jm: f64,
}

impl LocalMat {
    pub fn zeros(n: usize) -> Self {
        Self {
            n,
            data: vec![0.0; n * n],
        }
    }

    pub fn get(&self, i: usize, j: usize) -> f64 {
        self.data[i * self.n + j]
    }

    pub fn set(&mut self, i: usize, j: usize, v: f64) {
        self.data[i * self.n + j] = v;
    }

    pub fn to_triplets(&self, gdofs: &[usize]) -> Vec<squid_n_math::sparse::Triplet> {
        let mut out = Vec::with_capacity(self.n * self.n);
        for i in 0..self.n {
            let gi = gdofs[i];
            if gi == usize::MAX {
                continue;
            }
            for (j, &gj) in gdofs.iter().enumerate().take(self.n) {
                if gj == usize::MAX {
                    continue;
                }
                let v = self.get(i, j);
                if v != 0.0 {
                    out.push(squid_n_math::sparse::Triplet {
                        row: gi,
                        col: gj,
                        val: v,
                    });
                }
            }
        }
        out
    }
}

pub trait ElementBehavior {
    fn n_dof(&self) -> usize;
    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]>;
    fn tangent_stiffness(&self, state: &ElemState, ctx: &Ctx) -> LocalMat;
    fn internal_force(&self, state: &ElemState, ctx: &Ctx) -> LocalVec;
    fn update_state(&mut self, _du: &LocalVec, _commit: bool, _ctx: &Ctx) {}
    fn mass_matrix(&self, opt: MassOption) -> LocalMat;
    fn recover_forces(&self, _u_elem: &[f64]) -> Option<crate::beam::MemberForces> {
        None
    }
    /// T7: 線形化幾何剛性 Kg（P-Δ）。軸力 N（引張正）。デフォルトはゼロ。
    fn geometric_stiffness(&self, _n: f64) -> LocalMat {
        LocalMat::zeros(12)
    }
    /// T4: 全材料の committed 状態をスナップショット
    fn snapshot_state(&self) -> Box<dyn Any> {
        Box::new(())
    }
    /// T4: スナップショットから状態を復元
    fn restore_state(&mut self, _state: &dyn Any) {}
    /// T4: 全材料の trial を committed に確定
    fn commit_state(&mut self) {}
    /// T4: 全材料の trial を committed に戻す（rollback）
    fn revert_state(&mut self) {}
    /// チェックポイント用: 要素の全状態をバイト列へ直列化
    fn serialize_checkpoint(&self) -> Vec<u8> {
        vec![]
    }
    /// チェックポイント用: バイト列から要素状態を復元
    fn deserialize_checkpoint(&mut self, _data: &[u8]) {}
    /// 塑性率評価用の危険断面プローブ（ファイバー要素のみ実装。既定は None）。
    /// RESP-D「05 非線形モデル」ファイバーモデルの塑性率算定に用いる。
    fn ductility_probe(&self) -> Option<DuctilityProbe> {
        None
    }
    /// コンクリート履歴の除荷則を解析種別で切替える（RESP-D「05 非線形モデル」:
    /// 静的=逆行型／動的=原点指向型）。`dynamic=true` で原点指向型。
    /// ファイバー要素がコンクリート材料へ伝播する（既定は何もしない）。
    fn set_concrete_hysteresis(&mut self, _dynamic: bool) {}
}
