fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions::default();
    eframe::run_native(
        "structcalc",
        options,
        Box::new(|_cc| Ok(Box::new(sc_app::app::App::default()))),
    )
}
