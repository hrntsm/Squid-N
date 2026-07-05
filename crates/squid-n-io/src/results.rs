use arrow::array::{BooleanArray, Float64Array, UInt32Array, UInt64Array};
use arrow::datatypes::{DataType, Field, Schema};
use arrow::record_batch::RecordBatch;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use parquet::arrow::arrow_writer::ArrowWriter;
use parquet::arrow::ProjectionMask;
use parquet::file::properties::WriterProperties;
use squid_n_core::ids::{ElemId, NodeId};
use std::fs::File;
use std::sync::Arc;

pub type CaseId = u32;

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ResultKind {
    NodalDisp,
    MemberForce,
    Story,
    Modal,
    TimeHistory,
}

pub struct ResultQuery {
    pub case: CaseId,
    pub kind: ResultKind,
    pub node_filter: Option<Vec<NodeId>>,
    pub member_filter: Option<Vec<ElemId>>,
    pub step_range: Option<(u64, u64)>,
}

pub struct ResultBatch {
    pub batch: RecordBatch,
}

pub trait ResultWriter {
    fn write_rows(&mut self, batch: &RecordBatch);
    fn finish(self: Box<Self>);
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ResultManifest {
    pub entries: Vec<ResultEntry>,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ResultEntry {
    pub case: CaseId,
    pub kind: ResultKind,
    pub rows: u64,
    pub path: String,
}

/// 結果ストア。`Send` を要求するのは、MCP サーバ(P8)が `ServerState` を
/// スレッド間で共有する(`rmcp::ServerHandler: Send + Sync`)ため。
pub trait ResultStore: Send {
    fn writer(&mut self, case: CaseId, kind: ResultKind) -> Box<dyn ResultWriter>;
    fn query(&self, q: &ResultQuery) -> ResultBatch;
    fn manifest(&self) -> &ResultManifest;
}

pub struct ParquetWriter {
    inner: ArrowWriter<File>,
    rows: u64,
}

impl ParquetWriter {
    pub fn create(path: &str, schema: Arc<Schema>) -> parquet::errors::Result<Self> {
        let file = File::create(path)?;
        let props = WriterProperties::builder()
            .set_max_row_group_row_count(Some(64 * 1024))
            .build();
        Ok(Self {
            inner: ArrowWriter::try_new(file, schema, Some(props))?,
            rows: 0,
        })
    }
}

impl ResultWriter for ParquetWriter {
    fn write_rows(&mut self, batch: &RecordBatch) {
        self.rows += batch.num_rows() as u64;
        self.inner.write(batch).expect("parquet write");
    }

    fn finish(self: Box<Self>) {
        self.inner.close().expect("parquet close");
    }
}

pub fn read_partial(path: &str, columns: Vec<usize>) -> parquet::errors::Result<Vec<RecordBatch>> {
    let file = File::open(path)?;
    let builder = ParquetRecordBatchReaderBuilder::try_new(file)?;
    let mask = ProjectionMask::roots(builder.parquet_schema(), columns);
    let reader = builder.with_projection(mask).build()?;
    reader
        .collect::<Result<Vec<_>, _>>()
        .map_err(|e| parquet::errors::ParquetError::General(format!("{e:?}")))
}

pub fn read_all(path: &str) -> parquet::errors::Result<Vec<RecordBatch>> {
    let file = File::open(path)?;
    let builder = ParquetRecordBatchReaderBuilder::try_new(file)?;
    let reader = builder.build()?;
    reader
        .collect::<Result<Vec<_>, _>>()
        .map_err(|e| parquet::errors::ParquetError::General(format!("{e:?}")))
}

pub fn nodal_disp_schema() -> Arc<Schema> {
    Arc::new(Schema::new(vec![
        Field::new("node_id", DataType::UInt32, false),
        Field::new("ux", DataType::Float64, false),
        Field::new("uy", DataType::Float64, false),
        Field::new("uz", DataType::Float64, false),
        Field::new("rx", DataType::Float64, false),
        Field::new("ry", DataType::Float64, false),
        Field::new("rz", DataType::Float64, false),
    ]))
}

pub fn nodal_disp_batch(node_ids: &[u32], disp: &[[f64; 6]]) -> arrow::error::Result<RecordBatch> {
    let n = node_ids.len();
    let id_arr = UInt32Array::from(node_ids.to_vec());
    let mut ux = Vec::with_capacity(n);
    let mut uy = Vec::with_capacity(n);
    let mut uz = Vec::with_capacity(n);
    let mut rx = Vec::with_capacity(n);
    let mut ry = Vec::with_capacity(n);
    let mut rz = Vec::with_capacity(n);
    for d in disp {
        ux.push(d[0]);
        uy.push(d[1]);
        uz.push(d[2]);
        rx.push(d[3]);
        ry.push(d[4]);
        rz.push(d[5]);
    }
    RecordBatch::try_new(
        nodal_disp_schema(),
        vec![
            Arc::new(id_arr),
            Arc::new(Float64Array::from(ux)),
            Arc::new(Float64Array::from(uy)),
            Arc::new(Float64Array::from(uz)),
            Arc::new(Float64Array::from(rx)),
            Arc::new(Float64Array::from(ry)),
            Arc::new(Float64Array::from(rz)),
        ],
    )
}

pub fn member_force_schema() -> Arc<Schema> {
    Arc::new(Schema::new(vec![
        Field::new("elem_id", DataType::UInt32, false),
        Field::new("pos", DataType::Float64, false), // 評価位置 0..1
        Field::new("n", DataType::Float64, false),
        Field::new("qy", DataType::Float64, false),
        Field::new("qz", DataType::Float64, false),
        Field::new("mx", DataType::Float64, false),
        Field::new("my", DataType::Float64, false),
        Field::new("mz", DataType::Float64, false),
    ]))
}

/// 部材内力（評価位置別）を RecordBatch 化する。
/// `rows`: (要素ID, 評価位置 0..1, [N,Qy,Qz,Mx,My,Mz])
pub fn member_force_batch(rows: &[(u32, f64, [f64; 6])]) -> arrow::error::Result<RecordBatch> {
    let n = rows.len();
    let mut elem = Vec::with_capacity(n);
    let mut pos = Vec::with_capacity(n);
    let mut cols: [Vec<f64>; 6] = Default::default();
    for (e, p, f) in rows {
        elem.push(*e);
        pos.push(*p);
        for (c, v) in cols.iter_mut().zip(f.iter()) {
            c.push(*v);
        }
    }
    let [n_, qy, qz, mx, my, mz] = cols;
    RecordBatch::try_new(
        member_force_schema(),
        vec![
            Arc::new(UInt32Array::from(elem)),
            Arc::new(Float64Array::from(pos)),
            Arc::new(Float64Array::from(n_)),
            Arc::new(Float64Array::from(qy)),
            Arc::new(Float64Array::from(qz)),
            Arc::new(Float64Array::from(mx)),
            Arc::new(Float64Array::from(my)),
            Arc::new(Float64Array::from(mz)),
        ],
    )
}

pub fn modal_schema() -> Arc<Schema> {
    Arc::new(Schema::new(vec![
        Field::new("mode", DataType::UInt32, false),
        Field::new("period", DataType::Float64, false),
        Field::new("omega2", DataType::Float64, false),
        Field::new("part_x", DataType::Float64, false),
        Field::new("part_y", DataType::Float64, false),
        Field::new("part_z", DataType::Float64, false),
        Field::new("eff_x", DataType::Float64, false),
        Field::new("eff_y", DataType::Float64, false),
        Field::new("eff_z", DataType::Float64, false),
    ]))
}

pub fn time_history_schema() -> Arc<Schema> {
    Arc::new(Schema::new(vec![
        Field::new("step", DataType::UInt64, false),
        Field::new("time", DataType::Float64, false),
        Field::new("node_id", DataType::UInt32, false),
        Field::new("ux", DataType::Float64, false),
        Field::new("uy", DataType::Float64, false),
        Field::new("uz", DataType::Float64, false),
        Field::new("rx", DataType::Float64, false),
        Field::new("ry", DataType::Float64, false),
        Field::new("rz", DataType::Float64, false),
    ]))
}

/// モーダル結果（固有周期・刺激係数・有効質量）を RecordBatch 化する。
pub fn modal_batch(
    period: &[f64],
    omega2: &[f64],
    participation: &[[f64; 3]],
    effective_mass: &[[f64; 3]],
) -> arrow::error::Result<RecordBatch> {
    let n = period.len();
    let mode: Vec<u32> = (0..n as u32).collect();
    let mut part: [Vec<f64>; 3] = Default::default();
    let mut eff: [Vec<f64>; 3] = Default::default();
    for i in 0..n {
        for d in 0..3 {
            part[d].push(participation.get(i).map(|p| p[d]).unwrap_or(0.0));
            eff[d].push(effective_mass.get(i).map(|e| e[d]).unwrap_or(0.0));
        }
    }
    let [px, py, pz] = part;
    let [ex, ey, ez] = eff;
    RecordBatch::try_new(
        modal_schema(),
        vec![
            Arc::new(UInt32Array::from(mode)),
            Arc::new(Float64Array::from(period.to_vec())),
            Arc::new(Float64Array::from(omega2.to_vec())),
            Arc::new(Float64Array::from(px)),
            Arc::new(Float64Array::from(py)),
            Arc::new(Float64Array::from(pz)),
            Arc::new(Float64Array::from(ex)),
            Arc::new(Float64Array::from(ey)),
            Arc::new(Float64Array::from(ez)),
        ],
    )
}

pub fn time_history_batch(
    step: u64,
    time: f64,
    node_ids: &[u32],
    disp: &[[f64; 6]],
) -> arrow::error::Result<RecordBatch> {
    let n = node_ids.len();
    let step_arr = UInt64Array::from(vec![step; n]);
    let time_arr = Float64Array::from(vec![time; n]);
    let id_arr = UInt32Array::from(node_ids.to_vec());
    let mut ux = Vec::with_capacity(n);
    let mut uy = Vec::with_capacity(n);
    let mut uz = Vec::with_capacity(n);
    let mut rx = Vec::with_capacity(n);
    let mut ry = Vec::with_capacity(n);
    let mut rz = Vec::with_capacity(n);
    for d in disp {
        ux.push(d[0]);
        uy.push(d[1]);
        uz.push(d[2]);
        rx.push(d[3]);
        ry.push(d[4]);
        rz.push(d[5]);
    }
    RecordBatch::try_new(
        time_history_schema(),
        vec![
            Arc::new(step_arr),
            Arc::new(time_arr),
            Arc::new(id_arr),
            Arc::new(Float64Array::from(ux)),
            Arc::new(Float64Array::from(uy)),
            Arc::new(Float64Array::from(uz)),
            Arc::new(Float64Array::from(rx)),
            Arc::new(Float64Array::from(ry)),
            Arc::new(Float64Array::from(rz)),
        ],
    )
}

pub struct TimeHistoryWriter {
    writer: ParquetWriter,
    step: u64,
}

impl TimeHistoryWriter {
    pub fn create(path: &str) -> parquet::errors::Result<Self> {
        let schema = time_history_schema();
        Ok(Self {
            writer: ParquetWriter::create(path, schema)?,
            step: 0,
        })
    }

    pub fn write_step(
        &mut self,
        time: f64,
        node_ids: &[u32],
        disp: &[[f64; 6]],
    ) -> arrow::error::Result<()> {
        let batch = time_history_batch(self.step, time, node_ids, disp)?;
        self.writer.write_rows(&batch);
        self.step += 1;
        Ok(())
    }

    pub fn finish(self) {
        Box::new(self.writer).finish();
    }

    pub fn current_step(&self) -> u64 {
        self.step
    }
}

pub fn read_time_history_range(
    path: &str,
    step_range: Option<(u64, u64)>,
    node_filter: Option<&[u32]>,
) -> parquet::errors::Result<Vec<RecordBatch>> {
    let batches = read_all(path)?;
    if step_range.is_none() && node_filter.is_none() {
        return Ok(batches);
    }

    let node_set: Option<std::collections::HashSet<u32>> =
        node_filter.map(|ids| ids.iter().copied().collect());

    let mut result = Vec::new();
    for batch in batches {
        let step_col = batch
            .column(0)
            .as_any()
            .downcast_ref::<UInt64Array>()
            .expect("step column should be UInt64");
        let node_col = batch
            .column(2)
            .as_any()
            .downcast_ref::<UInt32Array>()
            .expect("node_id column should be UInt32");

        let num_rows = batch.num_rows();
        let mut keep = vec![true; num_rows];

        if let Some((start, end)) = step_range {
            for (i, k) in keep.iter_mut().enumerate().take(num_rows) {
                let s = step_col.value(i);
                if s < start || s > end {
                    *k = false;
                }
            }
        }

        if let Some(ref ids) = node_set {
            for (i, k) in keep.iter_mut().enumerate().take(num_rows) {
                if *k && !ids.contains(&node_col.value(i)) {
                    *k = false;
                }
            }
        }

        let mask = BooleanArray::from(keep);
        let filtered = arrow::compute::filter_record_batch(&batch, &mask)
            .map_err(|e| parquet::errors::ParquetError::General(format!("{e:?}")))?;
        if filtered.num_rows() > 0 {
            result.push(filtered);
        }
    }

    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parquet_roundtrip() {
        let dir = std::env::temp_dir();
        let path = dir.join("p2_test_nodal.parquet");
        let path_str = path.to_str().unwrap();

        {
            let mut writer = ParquetWriter::create(path_str, nodal_disp_schema()).unwrap();
            let batch = nodal_disp_batch(
                &[0, 1, 2],
                &[
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [1.5, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 2.0, 0.0, 0.0, 0.0, 0.0],
                ],
            )
            .unwrap();
            writer.write_rows(&batch);
            Box::new(writer).finish();
        }

        let batches = read_all(path_str).unwrap();
        assert_eq!(batches.len(), 1);
        let batch = &batches[0];
        assert_eq!(batch.num_rows(), 3);
        assert_eq!(batch.num_columns(), 7);
    }

    #[test]
    fn test_modal_roundtrip_partial() {
        let dir = std::env::temp_dir();
        let path = dir.join("p2_test_modal.parquet");
        let path_str = path.to_str().unwrap();
        {
            let mut writer = ParquetWriter::create(path_str, modal_schema()).unwrap();
            let batch = modal_batch(
                &[0.3215, 0.1228],
                &[382.0, 2618.0],
                &[[1.0, 0.0, 0.0], [0.5, 0.0, 0.0]],
                &[[1.894, 0.0, 0.0], [0.106, 0.0, 0.0]],
            )
            .unwrap();
            writer.write_rows(&batch);
            Box::new(writer).finish();
        }
        // 部分読出し: period 列(=1)のみ射影
        let batches = read_partial(path_str, vec![1]).unwrap();
        let b = &batches[0];
        assert_eq!(b.num_columns(), 1);
        assert_eq!(b.num_rows(), 2);
    }

    #[test]
    fn test_member_force_roundtrip() {
        let dir = std::env::temp_dir();
        let path = dir.join("p2_test_member.parquet");
        let path_str = path.to_str().unwrap();
        {
            let mut writer = ParquetWriter::create(path_str, member_force_schema()).unwrap();
            let batch = member_force_batch(&[
                (1, 0.0, [100.0, 5.0, 0.0, 0.0, 0.0, 200.0]),
                (1, 1.0, [100.0, 5.0, 0.0, 0.0, 0.0, -200.0]),
            ])
            .unwrap();
            writer.write_rows(&batch);
            Box::new(writer).finish();
        }
        let batches = read_all(path_str).unwrap();
        assert_eq!(batches[0].num_rows(), 2);
        assert_eq!(batches[0].num_columns(), 8);
    }

    #[test]
    fn test_time_history_schema_fields() {
        let schema = time_history_schema();
        assert_eq!(schema.fields().len(), 9);
        assert_eq!(schema.field(0).name(), "step");
        assert_eq!(schema.field(1).name(), "time");
        assert_eq!(schema.field(2).name(), "node_id");
        assert_eq!(schema.field(3).name(), "ux");
        assert_eq!(schema.field(4).name(), "uy");
        assert_eq!(schema.field(5).name(), "uz");
        assert_eq!(schema.field(6).name(), "rx");
        assert_eq!(schema.field(7).name(), "ry");
        assert_eq!(schema.field(8).name(), "rz");
        assert_eq!(schema.field(0).data_type(), &DataType::UInt64);
        assert_eq!(schema.field(1).data_type(), &DataType::Float64);
        assert_eq!(schema.field(2).data_type(), &DataType::UInt32);
    }

    #[test]
    fn test_time_history_batch_values() {
        let batch = time_history_batch(
            5,
            2.5,
            &[10, 20],
            &[
                [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
                [1.1, 1.2, 1.3, 1.4, 1.5, 1.6],
            ],
        )
        .unwrap();
        assert_eq!(batch.num_rows(), 2);
        assert_eq!(batch.num_columns(), 9);

        let step_col = batch
            .column(0)
            .as_any()
            .downcast_ref::<UInt64Array>()
            .unwrap();
        assert_eq!(step_col.value(0), 5);
        assert_eq!(step_col.value(1), 5);

        let time_col = batch
            .column(1)
            .as_any()
            .downcast_ref::<Float64Array>()
            .unwrap();
        assert!((time_col.value(0) - 2.5).abs() < f64::EPSILON);

        let node_col = batch
            .column(2)
            .as_any()
            .downcast_ref::<UInt32Array>()
            .unwrap();
        assert_eq!(node_col.value(0), 10);
        assert_eq!(node_col.value(1), 20);

        let ux_col = batch
            .column(3)
            .as_any()
            .downcast_ref::<Float64Array>()
            .unwrap();
        assert!((ux_col.value(1) - 1.1).abs() < f64::EPSILON);
    }

    #[test]
    fn test_time_history_write_read_roundtrip() {
        let dir = std::env::temp_dir();
        let path = dir.join("p6_th_roundtrip.parquet");
        let path_str = path.to_str().unwrap();

        {
            let mut writer = TimeHistoryWriter::create(path_str).unwrap();
            let nodes = [1u32, 2, 3];
            for s in 0..3 {
                let t = s as f64 * 0.1;
                let d = [
                    [t + 0.01, t + 0.02, t + 0.03, t + 0.04, t + 0.05, t + 0.06],
                    [t + 0.11, t + 0.12, t + 0.13, t + 0.14, t + 0.15, t + 0.16],
                    [t + 0.21, t + 0.22, t + 0.23, t + 0.24, t + 0.25, t + 0.26],
                ];
                writer.write_step(t, &nodes, &d).unwrap();
            }
            assert_eq!(writer.current_step(), 3);
            writer.finish();
        }

        let batches = read_all(path_str).unwrap();
        let total_rows: usize = batches.iter().map(|b| b.num_rows()).sum();
        assert_eq!(total_rows, 9);
    }

    #[test]
    fn test_time_history_partial_read_step_range() {
        let dir = std::env::temp_dir();
        let path = dir.join("p6_th_step_range.parquet");
        let path_str = path.to_str().unwrap();

        {
            let mut writer = TimeHistoryWriter::create(path_str).unwrap();
            for s in 0..3 {
                writer
                    .write_step(s as f64 * 0.1, &[1, 2], &[[0.1; 6], [0.2; 6]])
                    .unwrap();
            }
            writer.finish();
        }

        let batches = read_time_history_range(path_str, Some((1, 2)), None).unwrap();
        let total_rows: usize = batches.iter().map(|b| b.num_rows()).sum();
        assert_eq!(total_rows, 4);

        for batch in &batches {
            let step_col = batch
                .column(0)
                .as_any()
                .downcast_ref::<UInt64Array>()
                .unwrap();
            for i in 0..batch.num_rows() {
                let s = step_col.value(i);
                assert!((1..=2).contains(&s));
            }
        }
    }

    #[test]
    fn test_time_history_partial_read_node_filter() {
        let dir = std::env::temp_dir();
        let path = dir.join("p6_th_node_filter.parquet");
        let path_str = path.to_str().unwrap();

        {
            let mut writer = TimeHistoryWriter::create(path_str).unwrap();
            writer
                .write_step(0.0, &[1, 2, 3], &[[0.1; 6], [0.2; 6], [0.3; 6]])
                .unwrap();
            writer.finish();
        }

        let batches = read_time_history_range(path_str, None, Some(&[1u32])).unwrap();
        let total_rows: usize = batches.iter().map(|b| b.num_rows()).sum();
        assert_eq!(total_rows, 1);

        let batch = &batches[0];
        let node_col = batch
            .column(2)
            .as_any()
            .downcast_ref::<UInt32Array>()
            .unwrap();
        assert_eq!(node_col.value(0), 1);
    }
}
