use squid_n_core::ids::LoadCaseId;
use squid_n_core::model::LoadCombination;

/// [`standard_combinations`] への入力ケース指定。
pub struct ComboInput {
    pub dl: LoadCaseId,
    pub ll: LoadCaseId,
    pub seismic_x: Option<LoadCaseId>,
    pub seismic_y: Option<LoadCaseId>,
    pub wind_x: Option<LoadCaseId>,
    pub wind_y: Option<LoadCaseId>,
    pub snow: Option<LoadCaseId>,
    /// 多雪区域か否か（建築基準法施行令86条・同82条）。
    /// `true` の場合、長期に `0.7S` を加算し、短期地震・短期暴風に
    /// `0.35S` を加算した組合せも追加で生成する。
    pub heavy_snow_zone: bool,
}

fn push_gp(combos: &mut Vec<LoadCombination>, dl: LoadCaseId, ll: LoadCaseId) {
    combos.push(LoadCombination {
        name: "G + P".into(),
        terms: vec![(dl, 1.0), (ll, 1.0)],
    });
}

/// 建築基準法施行令82条の標準荷重組合せを生成する。
///
/// - 長期: `G+P`。多雪区域はさらに `G+P+0.7S`。
/// - 短期積雪: `G+P+S`（`snow` が指定されている場合）。
/// - 短期地震: `G+P±Kx`・`G+P±Ky`（±両方向）。多雪区域はさらに
///   `G+P+0.35S±K`（X・Y 各方向）。
/// - 短期暴風: `G+P±Wx`・`G+P±Wy`（±両方向）。多雪区域はさらに
///   `G+P+0.35S±W`（X・Y 各方向）。
///
/// 各ケースは `seismic_x`/`seismic_y`/`wind_x`/`wind_y`/`snow` が
/// `Some` の場合のみ生成される（レビュー §1.10）。
pub fn standard_combinations(input: &ComboInput) -> Vec<LoadCombination> {
    let mut combos = Vec::new();
    let dl = input.dl;
    let ll = input.ll;

    // 長期: G+P
    push_gp(&mut combos, dl, ll);

    // 多雪区域の長期: G+P+0.7S
    if input.heavy_snow_zone {
        if let Some(snow) = input.snow {
            combos.push(LoadCombination {
                name: "G + P + 0.7S".into(),
                terms: vec![(dl, 1.0), (ll, 1.0), (snow, 0.7)],
            });
        }
    }

    // 短期積雪: G+P+S
    if let Some(snow) = input.snow {
        combos.push(LoadCombination {
            name: "G + P + S".into(),
            terms: vec![(dl, 1.0), (ll, 1.0), (snow, 1.0)],
        });
    }

    // 短期地震（±両方向、多雪区域は 0.35S 付きも追加）。
    push_directional(
        &mut combos,
        dl,
        ll,
        input.seismic_x,
        "Kx",
        input.snow,
        input.heavy_snow_zone,
    );
    push_directional(
        &mut combos,
        dl,
        ll,
        input.seismic_y,
        "Ky",
        input.snow,
        input.heavy_snow_zone,
    );

    // 短期暴風（±両方向、多雪区域は 0.35S 付きも追加）。
    push_directional(
        &mut combos,
        dl,
        ll,
        input.wind_x,
        "Wx",
        input.snow,
        input.heavy_snow_zone,
    );
    push_directional(
        &mut combos,
        dl,
        ll,
        input.wind_y,
        "Wy",
        input.snow,
        input.heavy_snow_zone,
    );

    combos
}

/// 地震・暴風いずれかの片方向（Kx/Ky/Wx/Wy）について、`G+P±X` と
/// 多雪区域なら `G+P+0.35S±X` を追加する共通ヘルパー。
fn push_directional(
    combos: &mut Vec<LoadCombination>,
    dl: LoadCaseId,
    ll: LoadCaseId,
    case: Option<LoadCaseId>,
    label: &str,
    snow: Option<LoadCaseId>,
    heavy_snow_zone: bool,
) {
    let Some(case) = case else {
        return;
    };
    combos.push(LoadCombination {
        name: format!("G + P + {label}"),
        terms: vec![(dl, 1.0), (ll, 1.0), (case, 1.0)],
    });
    combos.push(LoadCombination {
        name: format!("G + P - {label}"),
        terms: vec![(dl, 1.0), (ll, 1.0), (case, -1.0)],
    });
    if heavy_snow_zone {
        if let Some(snow) = snow {
            combos.push(LoadCombination {
                name: format!("G + P + 0.35S + {label}"),
                terms: vec![(dl, 1.0), (ll, 1.0), (snow, 0.35), (case, 1.0)],
            });
            combos.push(LoadCombination {
                name: format!("G + P + 0.35S - {label}"),
                terms: vec![(dl, 1.0), (ll, 1.0), (snow, 0.35), (case, -1.0)],
            });
        }
    }
}

/// 旧API（後方互換）。地震± 方向・暴風・多雪区域を扱わない単純版が必要な
/// 呼び出し元向けに残す。内部では [`standard_combinations`] に委譲する
/// （地震の ± 両方向・暴風・多雪区域の組合せが追加される点が旧実装からの
/// 拡張＝レビュー §1.10 の是正）。
pub fn auto_combinations(
    dl_case: LoadCaseId,
    ll_case: LoadCaseId,
    seismic_x: Option<LoadCaseId>,
    seismic_y: Option<LoadCaseId>,
    snow_case: Option<LoadCaseId>,
) -> Vec<LoadCombination> {
    let input = ComboInput {
        dl: dl_case,
        ll: ll_case,
        seismic_x,
        seismic_y,
        wind_x: None,
        wind_y: None,
        snow: snow_case,
        heavy_snow_zone: false,
    };
    standard_combinations(&input)
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

    #[test]
    fn test_auto_combos_no_snow_matches_legacy_shape() {
        // 多雪区域=false・風=None の従来相当構成では、長期1 + 短期積雪0
        // + 地震(±Kx,±Ky)=4 の計 5 ケース。
        let combos = auto_combinations(
            LoadCaseId(1),
            LoadCaseId(2),
            Some(LoadCaseId(3)),
            Some(LoadCaseId(4)),
            None,
        );
        assert_eq!(combos.len(), 5);
        let names: Vec<&str> = combos.iter().map(|c| c.name.as_str()).collect();
        assert_eq!(
            names,
            vec![
                "G + P",
                "G + P + Kx",
                "G + P - Kx",
                "G + P + Ky",
                "G + P - Ky"
            ]
        );
    }

    #[test]
    fn test_standard_combinations_all_cases_heavy_snow() {
        let input = ComboInput {
            dl: LoadCaseId(1),
            ll: LoadCaseId(2),
            seismic_x: Some(LoadCaseId(3)),
            seismic_y: Some(LoadCaseId(4)),
            wind_x: Some(LoadCaseId(5)),
            wind_y: Some(LoadCaseId(6)),
            snow: Some(LoadCaseId(7)),
            heavy_snow_zone: true,
        };
        let combos = standard_combinations(&input);
        // G+P(1) + G+P+0.7S(1) + G+P+S(1)
        // + Kx系4 + Ky系4 + Wx系4 + Wy系4 = 3 + 16 = 19
        assert_eq!(combos.len(), 19);

        let by_name = |n: &str| {
            combos
                .iter()
                .find(|c| c.name == n)
                .unwrap_or_else(|| panic!("missing combo {n}"))
        };

        assert_eq!(
            by_name("G + P").terms,
            vec![(LoadCaseId(1), 1.0), (LoadCaseId(2), 1.0)]
        );
        assert_eq!(
            by_name("G + P + 0.7S").terms,
            vec![
                (LoadCaseId(1), 1.0),
                (LoadCaseId(2), 1.0),
                (LoadCaseId(7), 0.7)
            ]
        );
        assert_eq!(
            by_name("G + P + S").terms,
            vec![
                (LoadCaseId(1), 1.0),
                (LoadCaseId(2), 1.0),
                (LoadCaseId(7), 1.0)
            ]
        );
        assert_eq!(
            by_name("G + P + Kx").terms,
            vec![
                (LoadCaseId(1), 1.0),
                (LoadCaseId(2), 1.0),
                (LoadCaseId(3), 1.0)
            ]
        );
        assert_eq!(
            by_name("G + P - Kx").terms,
            vec![
                (LoadCaseId(1), 1.0),
                (LoadCaseId(2), 1.0),
                (LoadCaseId(3), -1.0)
            ]
        );
        assert_eq!(
            by_name("G + P + 0.35S + Kx").terms,
            vec![
                (LoadCaseId(1), 1.0),
                (LoadCaseId(2), 1.0),
                (LoadCaseId(7), 0.35),
                (LoadCaseId(3), 1.0)
            ]
        );
        assert_eq!(
            by_name("G + P + 0.35S - Ky").terms,
            vec![
                (LoadCaseId(1), 1.0),
                (LoadCaseId(2), 1.0),
                (LoadCaseId(7), 0.35),
                (LoadCaseId(4), -1.0)
            ]
        );
        assert_eq!(
            by_name("G + P + Wx").terms,
            vec![
                (LoadCaseId(1), 1.0),
                (LoadCaseId(2), 1.0),
                (LoadCaseId(5), 1.0)
            ]
        );
        assert_eq!(
            by_name("G + P - Wy").terms,
            vec![
                (LoadCaseId(1), 1.0),
                (LoadCaseId(2), 1.0),
                (LoadCaseId(6), -1.0)
            ]
        );
        assert_eq!(
            by_name("G + P + 0.35S + Wy").terms,
            vec![
                (LoadCaseId(1), 1.0),
                (LoadCaseId(2), 1.0),
                (LoadCaseId(7), 0.35),
                (LoadCaseId(6), 1.0)
            ]
        );
    }

    #[test]
    fn test_standard_combinations_no_heavy_snow_no_wind() {
        let input = ComboInput {
            dl: LoadCaseId(1),
            ll: LoadCaseId(2),
            seismic_x: Some(LoadCaseId(3)),
            seismic_y: Some(LoadCaseId(4)),
            wind_x: None,
            wind_y: None,
            snow: Some(LoadCaseId(5)),
            heavy_snow_zone: false,
        };
        let combos = standard_combinations(&input);
        // G+P(1) + G+P+S(1) + Kx系2 + Ky系2 = 6（多雪でないので 0.7S・0.35S 系は無し）
        assert_eq!(combos.len(), 6);
        assert!(combos.iter().all(|c| !c.name.contains("0.35S")));
        assert!(combos.iter().all(|c| !c.name.contains("0.7S")));
        assert!(combos.iter().all(|c| !c.name.contains('W')));
    }

    #[test]
    fn test_standard_combinations_empty_optional_cases() {
        let input = ComboInput {
            dl: LoadCaseId(1),
            ll: LoadCaseId(2),
            seismic_x: None,
            seismic_y: None,
            wind_x: None,
            wind_y: None,
            snow: None,
            heavy_snow_zone: false,
        };
        let combos = standard_combinations(&input);
        assert_eq!(combos.len(), 1);
        assert_eq!(combos[0].name, "G + P");
    }
}
