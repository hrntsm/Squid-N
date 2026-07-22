//! 断面形状の型定義。
//!
//! - [`BarSet`] — RC 主筋セット
//! - [`ShearBar`] — せん断補強筋
//! - [`RcRebar`] — RC 配筋情報
//! - [`SectionShape`] — パラメトリック断面形状の列挙
//! - [`bar_set_area`] — 主筋セットの総断面積

/// RC 配筋の主筋セット（方向別）。
///
/// `count`: 本数, `dia`: 径 [mm], `layers`: 段数。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct BarSet {
    pub count: u32,
    pub dia: f64,
    pub layers: u32,
}

/// RC せん断補強筋。
///
/// `dia`: 径 [mm], `pitch`: ピッチ [mm], `legs`: 組数,
/// `grade`: 材質（None は普通強度＝主筋と同種扱い。高強度せん断補強筋は
/// 製品名/規格名で指定する。例: "UB785"（ウルボン785）, "KH785"（スーパーフープ）,
/// "KSS785", "SHD685", "SPR785", "MK785", "SBPD1275" 等）。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ShearBar {
    pub dia: f64,
    pub pitch: f64,
    pub legs: u32,
    #[serde(default)]
    pub grade: Option<String>,
}

/// RC 配筋情報。
///
/// `main_x`: せい方向（X）主筋, `main_y`: 幅方向（Y）主筋,
/// `cover`: かぶり [mm], `shear`: せん断補強筋。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RcRebar {
    pub main_x: BarSet,
    pub main_y: BarSet,
    pub cover: f64,
    pub shear: ShearBar,
}

/// Parametric section shape definition.
///
/// Each variant carries the minimal parameters needed to define the geometry.
/// Call `to_section()` to compute the derived section properties (A, Iy, Iz, J, ...)
/// and produce a `squid_n_core::Section`.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum SectionShape {
    /// Steel H‑shape (H形鋼).
    SteelH {
        height: f64,
        width: f64,
        web_thick: f64,
        flange_thick: f64,
    },
    /// Steel rectangular hollow section / box (角形鋼管).
    SteelBox {
        height: f64,
        width: f64,
        thick: f64,
        /// 角部外半径 r [mm]（せん断有効断面積の算定に用いる）。
        /// 0 は角部を直角とみなす（未入力・溶接組立箱形相当）。
        #[serde(default)]
        corner_r: f64,
    },
    /// Steel L‑angle (山形鋼).
    SteelAngle { leg_a: f64, leg_b: f64, thick: f64 },
    /// Steel C‑channel (溝形鋼).
    SteelChannel {
        height: f64,
        width: f64,
        web_thick: f64,
        flange_thick: f64,
    },
    /// Steel T‑shape (T形鋼).
    SteelTee {
        height: f64,
        width: f64,
        web_thick: f64,
        flange_thick: f64,
    },
    /// Steel round pipe (鋼管).
    SteelPipe { outer_dia: f64, thick: f64 },
    /// Steel flat bar / plate (平鋼・鋼板). 中実矩形。
    ///
    /// `width`: 幅 B [mm]（Z 方向）、`thick`: 板厚 t [mm]（Y 方向＝せい）。
    /// 断面性能は中実矩形として算定する（配筋は無い）。部材のせい/幅の向きは
    /// 局所座標（`ref_vector`）で与える。薄板だが幅厚比検定の対象外（板要素ではない）。
    SteelFlatBar { width: f64, thick: f64 },
    /// Steel solid round bar (中実丸鋼).
    ///
    /// `dia`: 直径 D [mm]。断面性能は中実円として算定する。
    SteelRoundBar { dia: f64 },
    /// Steel welded built-up H with unequal flanges (非対称組立 H 形鋼). `StbSecBuild-H`。
    ///
    /// 上下フランジの幅・厚が異なる溶接組立断面。`height`: せい H（外〜外）、
    /// `upper_width`/`upper_thick`: 上フランジ、`lower_width`/`lower_thick`: 下フランジ、
    /// `web_thick`: ウェブ厚。上下フランジ＋ウェブの矩形分解＋平行軸で断面性能を算定する
    /// （図心は Y 方向に偏心。左右対称）。上下同一寸法なら通常の `SteelH` と等価。
    SteelBuiltH {
        height: f64,
        upper_width: f64,
        upper_thick: f64,
        lower_width: f64,
        lower_thick: f64,
        web_thick: f64,
    },
    /// Steel cold-formed lipped channel (リップ溝形鋼). `StbSecRoll-LipC`。
    ///
    /// `height`: せい H [mm]（Y 方向）、`width`: フランジ幅 B [mm]（Z 方向。ウェブ外面〜
    /// フランジ先端）、`lip`: リップ長 C [mm]（Y 方向）、`thick`: 板厚 t [mm]（全要素一様）。
    /// 薄肉開断面としてウェブ・上下フランジ・上下リップの矩形分解で断面性能を算定する
    /// （図心は Z 方向に偏心。冷間成形材の有効断面・局部座屈は別途検討）。
    SteelLipChannel {
        height: f64,
        width: f64,
        lip: f64,
        thick: f64,
    },
    /// Reinforced concrete rectangle (RC 矩形).
    RcRect { b: f64, d: f64, rebar: RcRebar },
    /// Reinforced concrete circle column (RC 円形).
    RcCircle { d: f64, rebar: RcRebar },
    /// SRC 矩形断面（RC 矩形 + 内蔵 H 形鉄骨、SRC 規準 1987）。
    ///
    /// `steel_grade`: 内蔵鉄骨の鋼種（例 "SN400B"）。コンクリート強度は
    /// `Material.fc`、主筋グレードは `Material.name` を用いる既存慣習を踏襲する。
    ///
    /// 解析用断面性能（`to_section`）は、コンクリート断面にヤング係数比
    /// `N_S_EQ`（=15、暫定既定）による鉄骨の等価換算剛性を加えて算定する。
    /// 断面積は質量算定への影響を避けるためコンクリート全断面 `b·d` とする。
    SrcRect {
        b: f64,
        d: f64,
        rebar: RcRebar,
        steel_height: f64,
        steel_width: f64,
        steel_web_thick: f64,
        steel_flange_thick: f64,
        steel_grade: String,
    },
    /// CFT 角形（角形鋼管 + 充填コンクリート）。
    ///
    /// 解析用断面性能は鋼管部分のみ（充填コンクリートの剛性は暫定的に無視、
    /// 剛性計算編での複合換算は今後の課題）。検定では `Material.fc` の
    /// 充填コンクリート強度を用いる。
    CftBox { height: f64, width: f64, thick: f64 },
    /// CFT 円形（円形鋼管 + 充填コンクリート）。扱いは `CftBox` と同じ。
    CftPipe { outer_dia: f64, thick: f64 },
    /// RC 耐震壁（壁エレメント用）。
    ///
    /// `thickness`: 壁板厚 [mm]、`ps`: 壁板の直交する各方向のせん断補強筋比の
    /// うち小さい方（小数。例 0.0025）。壁の平面寸法は要素の節点座標から得る
    /// ため形状には持たない。`to_section` の断面性能は名目値（壁は暫定的に
    /// 等価梁でモデル化されており、実剛性の評価は要素実装側の課題）。
    RcWall { thickness: f64, ps: f64 },
}

/// 主筋セットの総断面積 [mm²]（本数×πr²。配筋検定・ファイバー生成用）。
pub fn bar_set_area(bs: &BarSet) -> f64 {
    let r = bs.dia / 2.0;
    bs.count as f64 * std::f64::consts::PI * r * r
}
