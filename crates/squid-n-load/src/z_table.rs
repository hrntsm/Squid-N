//! 地震地域係数 Z の市町村別ローダ機構。
//!
//! 出典: 昭和55年建設省告示第1793号 別表第2（地方区分ごとの Z 値表）。
//! **本モジュールは市町村名 → Z 値のルックアップ機構のみを提供し、
//! 告示別表の実データ（全市町村分）は同梱しない。** 法令データの正確性は
//! 官報・特定行政庁の公表資料と照合した上で利用者が用意する CSV に委ねる方針
//! （データの経年変更・誤記混入リスクをソースコードから切り離すため）。
//!
//! # CSV 形式
//! `市町村名,Z値` の2列。`#` で始まる行および空行はコメント/無視行として
//! スキップする。Z値は [`Z_VALUES`]（1.0/0.9/0.8/0.7）のいずれかでなければ
//! パースエラーとする。
//!
//! ```text
//! # 出典: 昭和55年建設省告示第1793号 別表第2
//! 東京都千代田区,1.0
//! 沖縄県那覇市,0.7
//! ```

/// 告示1793号が規定する Z の取り得る値。
pub const Z_VALUES: [f64; 4] = [1.0, 0.9, 0.8, 0.7];

/// 市町村名 → Z 値の対応表。
#[derive(Debug)]
pub struct ZTable {
    entries: Vec<(String, f64)>,
}

impl ZTable {
    /// CSV テキストから `ZTable` を構築する。`#` 始まりの行と空行は無視する。
    /// Z 値が [`Z_VALUES`] のいずれとも一致しない場合はエラーを返す。
    pub fn from_csv(text: &str) -> Result<Self, String> {
        let mut entries = Vec::new();
        for (lineno0, raw_line) in text.lines().enumerate() {
            let lineno = lineno0 + 1;
            let line = raw_line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let mut parts = line.splitn(2, ',');
            let name = parts
                .next()
                .ok_or_else(|| format!("line {lineno}: missing municipality name"))?
                .trim();
            let z_str = parts
                .next()
                .ok_or_else(|| format!("line {lineno}: missing Z value (expected '市町村名,Z値')"))?
                .trim();
            if name.is_empty() {
                return Err(format!("line {lineno}: empty municipality name"));
            }
            let z: f64 = z_str
                .parse()
                .map_err(|_| format!("line {lineno}: invalid Z value '{z_str}'"))?;
            if !Z_VALUES.iter().any(|v| (v - z).abs() < 1e-9) {
                return Err(format!(
                    "line {lineno}: Z value {z} is not one of the allowed values {Z_VALUES:?} (告示1793号)"
                ));
            }
            entries.push((name.to_string(), z));
        }
        Ok(Self { entries })
    }

    /// 市町村名から Z 値を引く。完全一致のみ（表記ゆれの正規化は行わない）。
    /// 見つからない場合は `None`。
    pub fn lookup(&self, municipality: &str) -> Option<f64> {
        self.entries
            .iter()
            .find(|(name, _)| name == municipality)
            .map(|(_, z)| *z)
    }

    /// 登録されている市町村数。
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// 登録が空かどうか。
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_and_lookup() {
        let csv = "# 出典: 昭和55年建設省告示第1793号 別表第2\n\
                    東京都千代田区,1.0\n\
                    \n\
                    沖縄県那覇市,0.7\n";
        let table = ZTable::from_csv(csv).expect("should parse");
        assert_eq!(table.len(), 2);
        assert_eq!(table.lookup("東京都千代田区"), Some(1.0));
        assert_eq!(table.lookup("沖縄県那覇市"), Some(0.7));
        assert_eq!(table.lookup("存在しない市"), None);
    }

    #[test]
    fn test_comment_and_blank_lines_ignored() {
        let csv = "#comment\n\n  \n#another\n大阪府大阪市,0.9\n";
        let table = ZTable::from_csv(csv).unwrap();
        assert_eq!(table.len(), 1);
        assert_eq!(table.lookup("大阪府大阪市"), Some(0.9));
    }

    #[test]
    fn test_invalid_z_value_rejected() {
        let csv = "変な市,0.85\n";
        let err = ZTable::from_csv(csv).unwrap_err();
        assert!(err.contains("line 1"));
        assert!(err.contains("0.85"));
    }

    #[test]
    fn test_missing_z_value_rejected() {
        let csv = "市町村のみ\n";
        assert!(ZTable::from_csv(csv).is_err());
    }

    #[test]
    fn test_non_numeric_z_rejected() {
        let csv = "市町村,abc\n";
        assert!(ZTable::from_csv(csv).is_err());
    }

    #[test]
    fn test_empty_municipality_name_rejected() {
        let csv = ",1.0\n";
        assert!(ZTable::from_csv(csv).is_err());
    }

    #[test]
    fn test_empty_table() {
        let table = ZTable::from_csv("").unwrap();
        assert!(table.is_empty());
        assert_eq!(table.lookup("anything"), None);
    }

    #[test]
    fn test_all_z_values_accepted() {
        let csv = "a,1.0\nb,0.9\nc,0.8\nd,0.7\n";
        let table = ZTable::from_csv(csv).unwrap();
        assert_eq!(table.len(), 4);
        for (name, z) in [("a", 1.0), ("b", 0.9), ("c", 0.8), ("d", 0.7)] {
            assert_eq!(table.lookup(name), Some(z));
        }
    }
}
