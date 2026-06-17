use sc_core::ids::LoadCaseId;
use sc_core::model::LoadCombination;

pub fn auto_combinations(
    dl_case: LoadCaseId,
    ll_case: LoadCaseId,
    seismic_x: Option<LoadCaseId>,
    seismic_y: Option<LoadCaseId>,
    snow_case: Option<LoadCaseId>,
) -> Vec<LoadCombination> {
    let mut combos = Vec::new();

    // Long-term (長期): DL + LL
    combos.push(LoadCombination {
        name: "G + P".into(),
        terms: vec![(dl_case, 1.0), (ll_case, 1.0)],
    });

    // Short-term (短期): DL + LL + Seismic (or Snow)
    if let Some(sx) = seismic_x {
        combos.push(LoadCombination {
            name: "G + P + Kx".into(),
            terms: vec![(dl_case, 1.0), (ll_case, 1.0), (sx, 1.0)],
        });
    }
    if let Some(sy) = seismic_y {
        combos.push(LoadCombination {
            name: "G + P + Ky".into(),
            terms: vec![(dl_case, 1.0), (ll_case, 1.0), (sy, 1.0)],
        });
    }
    if let Some(snow) = snow_case {
        combos.push(LoadCombination {
            name: "G + P + S".into(),
            terms: vec![(dl_case, 1.0), (ll_case, 1.0), (snow, 1.0)],
        });
    }

    combos
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_auto_combos() {
        let combos = auto_combinations(
            LoadCaseId(1),
            LoadCaseId(2),
            Some(LoadCaseId(3)),
            Some(LoadCaseId(4)),
            None,
        );
        assert!(combos.len() >= 3);
        assert_eq!(combos[0].name, "G + P");
        assert_eq!(combos[1].name, "G + P + Kx");
    }
}
