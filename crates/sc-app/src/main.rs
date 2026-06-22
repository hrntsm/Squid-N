fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions::default();
    eframe::run_native(
        "structcalc",
        options,
        Box::new(|cc| {
            sc_app::app::install_japanese_fonts(&cc.egui_ctx);
            Ok(Box::new(sc_app::app::App::default()))
        }),
    )
}
