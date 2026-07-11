use squid_n_element::behavior::ElementBehavior;
use std::any::Any;

/// 全要素の確定状態のスナップショット
pub struct StateSnapshot {
    pub states: Vec<Box<dyn Any>>,
}

impl StateSnapshot {
    /// 現在の全要素の状態をキャプチャ
    pub fn capture(behaviors: &[Box<dyn ElementBehavior>]) -> Self {
        StateSnapshot {
            states: behaviors.iter().map(|b| b.snapshot_state()).collect(),
        }
    }
}

/// 状態管理トレイト
pub trait StatefulModel {
    fn snapshot(&self, behaviors: &[Box<dyn ElementBehavior>]) -> StateSnapshot;
    fn restore(&mut self, snap: &StateSnapshot, behaviors: &mut [Box<dyn ElementBehavior>]);
    fn commit_all(&mut self, behaviors: &mut [Box<dyn ElementBehavior>]);
    fn revert_all(&mut self, behaviors: &mut [Box<dyn ElementBehavior>]);
}

impl StatefulModel for squid_n_core::model::Model {
    fn snapshot(&self, behaviors: &[Box<dyn ElementBehavior>]) -> StateSnapshot {
        StateSnapshot::capture(behaviors)
    }
    fn restore(&mut self, snap: &StateSnapshot, behaviors: &mut [Box<dyn ElementBehavior>]) {
        for (b, s) in behaviors.iter_mut().zip(&snap.states) {
            b.restore_state(s.as_ref());
        }
    }
    fn commit_all(&mut self, behaviors: &mut [Box<dyn ElementBehavior>]) {
        for b in behaviors.iter_mut() {
            b.commit_state();
        }
    }
    fn revert_all(&mut self, behaviors: &mut [Box<dyn ElementBehavior>]) {
        for b in behaviors.iter_mut() {
            b.revert_state();
        }
    }
}
