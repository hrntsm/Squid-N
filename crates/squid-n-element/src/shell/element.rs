//! 4節点シェル要素の本体と生成。
//!
//! - [`ShellElement`] — 節点・材料・断面・フレームを保持する要素構造体
//! - [`ShellElement::new`] — モデルデータから要素を構築（剛床判定含む）
//! - [`ShellElement::local_coords`] — 節点座標をローカル面内 2D 座標へ射影

use super::frame::ShellFrame;
use super::DEFAULT_DRILLING_FACTOR;
use squid_n_core::ids::NodeId;
use squid_n_core::model::Model;

// ---------------------------------------------------------------------------
// ShellElement
// ---------------------------------------------------------------------------
#[derive(Clone)]
pub struct ShellElement {
    pub nodes: [NodeId; 4],
    pub coords: [[f64; 3]; 4],
    pub t: f64,
    pub e: f64,
    pub nu: f64,
    pub density: f64,
    pub frame: ShellFrame,
    pub drilling_factor: f64,
    pub membrane_active: bool,
    /// 確定変位（4 節点 24 自由度、グローバル系）。commit_state で trial から確定。
    pub committed_disp: [f64; 24],
    /// トライアル変位（グローバル系）。Newton 反復中も蓄積され、
    /// internal_force はこちらを参照する（beam/behavior.rs と同じトライアル追従規約）。
    pub trial_disp: [f64; 24],
}

impl ShellElement {
    pub fn new(data: &squid_n_core::model::ElementData, model: &Model) -> Self {
        let nids = [data.nodes[0], data.nodes[1], data.nodes[2], data.nodes[3]];
        let coords = [
            model.nodes[nids[0].index()].coord,
            model.nodes[nids[1].index()].coord,
            model.nodes[nids[2].index()].coord,
            model.nodes[nids[3].index()].coord,
        ];
        let frame = ShellFrame::from_nodes(coords);

        let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
        let t = sec.and_then(|s| s.thickness).unwrap_or(100.0);

        let mat = data
            .material
            .and_then(|mid| model.materials.get(mid.index()));
        let e = mat.map(|m| m.young).unwrap_or(205000.0);
        let nu = mat.map(|m| m.poisson).unwrap_or(0.3);

        // Determine membrane_active: true unless every node is part of a rigid diaphragm
        let membrane_active = {
            let node_in_rigid_diaphragm = |nid: NodeId| -> bool {
                model
                    .nodes
                    .get(nid.index())
                    .and_then(|n| n.story)
                    .and_then(|sid| model.stories.get(sid.index()))
                    .map(|s| {
                        s.diaphragms
                            .iter()
                            .any(|d| d.rigid && (d.master == nid || d.slaves.contains(&nid)))
                    })
                    .unwrap_or(false)
            };
            !nids.iter().all(|&n| node_in_rigid_diaphragm(n))
        };

        ShellElement {
            nodes: nids,
            coords,
            t,
            e,
            nu,
            density: mat.map(|m| m.density).unwrap_or(0.0),
            frame,
            drilling_factor: DEFAULT_DRILLING_FACTOR,
            membrane_active,
            committed_disp: [0.0; 24],
            trial_disp: [0.0; 24],
        }
    }

    /// 節点座標を要素ローカル面内 2D 座標（e1,e2 への射影）へ変換する。
    /// B 行列・ヤコビアンはこのローカル座標で評価しなければならない
    /// （`to_global` でフレーム回転を掛けるため、座標も同じフレームに揃える）。
    /// グローバル x,y を直接使うと、第1辺がグローバル x に沿わない要素で
    /// 二重回転になりパッチテストが破綻する。
    pub(crate) fn local_coords(&self) -> [[f64; 3]; 4] {
        let f = &self.frame;
        let mut lc = [[0.0; 3]; 4];
        for i in 0..4 {
            let c = self.coords[i];
            lc[i][0] = c[0] * f.e1[0] + c[1] * f.e1[1] + c[2] * f.e1[2];
            lc[i][1] = c[0] * f.e2[0] + c[1] * f.e2[1] + c[2] * f.e2[2];
            lc[i][2] = 0.0;
        }
        lc
    }
}
