//! モデルの照会（JSON 出力・クエリ・解析）関数。

use super::*;

pub fn get_model_json(state: &ServerState) -> String {
    serde_json::to_string(&state.model).unwrap_or_default()
}

/// `model.query` の中核ロジック（feature 非依存・テスト可能）。
///
/// `kind` で `node`/`member`(=element)/`section` を選び、各要素を JSON 化して返す。
/// `member`/`elements` では、`Model::member_detail` に付帯情報（ハンチ・継手位置。
/// 剛性には影響しない）が登録されている部材について `haunch_i`/`haunch_j`
/// （`length`/`depth_increase`/`width_increase`）と `joints`（`distance`/`kind`）
/// を追加で含める（付帯情報が無い部材は従来どおりのフィールドのみ）。
/// `filter` が与えられたときは、各 JSON を文字列化した中に部分一致するものだけを残す
/// （簡易フィルタ。名前・ID 等での絞り込み用）。MCP ツール `model_query` はこれを呼ぶ。
pub fn query_model(model: &Model, kind: &str, filter: Option<&str>) -> Vec<serde_json::Value> {
    use serde_json::json;
    let items: Vec<serde_json::Value> = match kind {
        "node" | "nodes" => model
            .nodes
            .iter()
            .map(|n| {
                json!({
                    "id": n.id.0,
                    "coord": n.coord,
                    "story": n.story.map(|s| s.0),
                })
            })
            .collect(),
        "member" | "members" | "element" | "elements" => model
            .elements
            .iter()
            .map(|e| {
                let mut v = json!({
                    "id": e.id.0,
                    "kind": format!("{:?}", e.kind),
                    "nodes": e.nodes.iter().map(|n| n.0).collect::<Vec<_>>(),
                    "section": e.section.map(|s| s.0),
                    "material": e.material.map(|m| m.0),
                });
                // 付帯情報（ハンチ・継手位置。剛性には影響しない）があれば併記する
                // （側テーブルが無い/空の部材は従来どおりのフィールドのみ）。
                if let Some(detail) = model.member_detail(e.id) {
                    let haunch_json = |h: &squid_n_core::model::Haunch| {
                        json!({
                            "length": h.length,
                            "depth_increase": h.depth_increase,
                            "width_increase": h.width_increase,
                        })
                    };
                    let obj = v.as_object_mut().expect("json!({...}) is always an object");
                    if let Some(h) = &detail.haunch_i {
                        obj.insert("haunch_i".to_string(), haunch_json(h));
                    }
                    if let Some(h) = &detail.haunch_j {
                        obj.insert("haunch_j".to_string(), haunch_json(h));
                    }
                    if !detail.joints.is_empty() {
                        obj.insert(
                            "joints".to_string(),
                            json!(detail
                                .joints
                                .iter()
                                .map(|j| json!({
                                    "distance": j.distance,
                                    "kind": format!("{:?}", j.kind),
                                }))
                                .collect::<Vec<_>>()),
                        );
                    }
                }
                v
            })
            .collect(),
        "section" | "sections" => model
            .sections
            .iter()
            .map(|s| {
                json!({
                    "id": s.id.0,
                    "name": s.name,
                    "area": s.area,
                    "iy": s.iy,
                    "iz": s.iz,
                })
            })
            .collect(),
        _ => vec![],
    };
    match filter {
        Some(f) if !f.is_empty() => items
            .into_iter()
            .filter(|v| v.to_string().contains(f))
            .collect(),
        _ => items,
    }
}

/// 数量積算（feature 非依存・テスト可能）。MCP ツール `quantity_takeoff` の中核。
///
/// 部位別の概算数量（`squid_n_design_jp::quantity`）を JSON で返す。
/// `group_by` は `category`（部位別、既定）/`story`（階別）/`steel`（鉄骨種類別）/
/// `rebar`（鉄筋径別）/`detail`（明細）。
pub fn quantity_takeoff_json(model: &Model, group_by: Option<&str>) -> serde_json::Value {
    use serde_json::json;
    use squid_n_design_jp::quantity::{compute_quantity_takeoff, QuantityCfg, QuantityTotals};

    let q = compute_quantity_takeoff(model, &QuantityCfg::default());
    let totals_json = |t: &QuantityTotals| {
        json!({
            "concrete_m3": t.concrete_m3,
            "formwork_m2": t.formwork_m2,
            "rebar_t": t.rebar_t,
            "steel_t": t.steel_t,
            "rebar_joints": t.rebar_joints,
        })
    };
    let rows: Vec<serde_json::Value> = match group_by.unwrap_or("category") {
        "story" | "stories" => q
            .totals_by_story()
            .iter()
            .map(|(story, t)| {
                let mut v = totals_json(t);
                v["story"] = json!(story);
                v
            })
            .collect(),
        "steel" => q
            .steel_by_section()
            .iter()
            .map(|s| {
                json!({
                    "section": s.section_name,
                    "length_m": s.length_m,
                    "weight_t": s.weight_t,
                })
            })
            .collect(),
        "rebar" => q
            .rebar_by_dia()
            .iter()
            .map(|(dia, len, w)| {
                json!({
                    "dia_mm": dia,
                    "length_m": len,
                    "weight_t": w,
                })
            })
            .collect(),
        "detail" | "items" => q
            .items
            .iter()
            .map(|it| {
                json!({
                    "elem": it.elem.map(|e| e.0),
                    "slab": it.slab.map(|s| s.0),
                    "label": it.label,
                    "story": it.story,
                    "category": it.category.label(),
                    "structure": it.structure.label(),
                    "concrete_m3": it.concrete_m3,
                    "formwork_m2": it.formwork_m2,
                    "rebar_t": it.rebar_weight_t(),
                    "steel_t": it.steel_weight_t(),
                    "rebar_joints": it.rebar_joints,
                })
            })
            .collect(),
        // 既定: 部位別小計。
        _ => q
            .totals_by_category()
            .iter()
            .map(|(cat, t)| {
                let mut v = totals_json(t);
                v["category"] = json!(cat.label());
                v
            })
            .collect(),
    };
    json!({
        "rows": rows,
        "totals": totals_json(&q.totals()),
        "notes": q.notes,
    })
}

/// 解析の実処理（feature 非依存・テスト可能）。`Model` の参照だけを受け取るため、
/// `ServerState` のロックを取らずに（= ロック解放後に）呼び出せる。
/// 現状は先頭の荷重ケースに対する線形静的解析のみ（他ジョブ種別は将来対応）。
///
/// `analysis_run`（MCP ツール）は `state.model.clone()` を取ってロックを落としてから
/// `spawn_blocking` 内でこの関数を呼ぶことで、CPU バウンドな解析中も `ServerState` の
/// ミューテックスを他ツール呼び出しのためにブロックしない。
pub fn analyze_model(model: &Model) -> Result<String, String> {
    // 解析前に剛域を自動算定してモデルへ反映する（設計書 §6.2.1「剛域」は標準実装）。
    let mut model = model.clone();
    squid_n_element::beam::apply_auto_rigid_zones(
        &mut model,
        &squid_n_element::beam::RigidZoneRule::default(),
    );
    let model = &model;
    let analysis = squid_n_solver::analysis::Analysis::prepare(model)
        .map_err(|e| format!("prepare failed: {e}"))?;
    if let Some(lc) = model.load_cases.first() {
        let result = analysis
            .linear_static(lc.id)
            .map_err(|e| format!("solve failed: {e}"))?;
        Ok(serde_json::to_string(&result.disp).unwrap_or_default())
    } else {
        Err("no load cases".into())
    }
}

/// `analyze_model` の `ServerState` 経由の薄いラッパ（後方互換用）。
pub fn analyze(state: &mut ServerState) -> Result<String, String> {
    analyze_model(&state.model)
}

// ============================================================================
// 全 JobKind の実処理（feature 非依存・テスト可能）。
//
// `analysis_run`（MCP ツール、mod server）は「ロック保持中にモデルを複製 →
// ロック解放 → spawn_blocking でこの節の compute_* を呼ぶ → 再度ロックして
// 結果ストアへ永続化 + ジョブ状態更新」という流れを取る（P8 の既存方針を踏襲）。
// compute_* はいずれも GUI（squid-n-app）非依存の純関数（&Model か Model の
// クローンだけで完結）とし、squid-n-app の同等ロジック（compute_pushover /
// compute_time_history / sample_wave / run_design_check）と重複する箇所は
// コメントで明記する（squid-n-mcp は squid-n-app に依存しないため複製が必要）。
// ============================================================================
