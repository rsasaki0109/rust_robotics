//! RustRobotics interactive playground entry point.

mod app;
mod grid_planners;

use app::PlaygroundApp;

fn main() -> eframe::Result<()> {
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1100.0, 780.0]),
        ..Default::default()
    };

    eframe::run_native(
        "RustRobotics Playground",
        native_options,
        Box::new(|ctx| Ok(Box::new(PlaygroundApp::new(ctx)))),
    )
}
