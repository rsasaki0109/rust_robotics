//! RustRobotics interactive playground entry point.

mod admm_formation;
mod app;
mod controller_arena;
mod engagement;
mod grid_planners;
mod localization;
mod parking;
mod pushing;
mod sampling;
mod share;
mod slam;
mod slam_drive;
mod slam_nav;
mod ui_kit;

use app::PlaygroundApp;

#[cfg(not(target_arch = "wasm32"))]
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

#[cfg(target_arch = "wasm32")]
fn main() {
    use wasm_bindgen::JsCast as _;

    console_error_panic_hook::set_once();

    wasm_bindgen_futures::spawn_local(async {
        let document = web_sys::window()
            .expect("no global window")
            .document()
            .expect("no document");
        let canvas = document
            .get_element_by_id("rust_robotics_playground")
            .expect("missing canvas #rust_robotics_playground")
            .dyn_into::<web_sys::HtmlCanvasElement>()
            .expect("canvas element was not HtmlCanvasElement");

        let web_options = eframe::WebOptions::default();
        let started = eframe::WebRunner::new()
            .start(
                canvas,
                web_options,
                Box::new(|ctx| Ok(Box::new(PlaygroundApp::new(ctx)))),
            )
            .await;
        // The page shows a spinner until the app runs (or says why not).
        match started {
            Ok(()) => {
                if let Some(loading) = document.get_element_by_id("loading") {
                    loading.remove();
                }
            }
            Err(error) => {
                if let Some(text) = document.get_element_by_id("loading-text") {
                    text.set_text_content(Some(
                        "The playground could not start (WebGL unavailable?).",
                    ));
                }
                panic!("failed to start RustRobotics playground: {error:?}");
            }
        }
    });
}
