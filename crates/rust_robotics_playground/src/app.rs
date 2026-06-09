//! Top-level egui application shell.

use crate::grid_planners::GridPlannerDemo;

pub struct PlaygroundApp {
    grid_demo: GridPlannerDemo,
}

impl PlaygroundApp {
    pub fn new(_ctx: &eframe::CreationContext<'_>) -> Self {
        Self {
            grid_demo: GridPlannerDemo::default(),
        }
    }
}

impl eframe::App for PlaygroundApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::TopBottomPanel::top("header").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.heading("RustRobotics Playground");
                ui.separator();
                ui.label("Grid planners — click obstacles, drag start/goal, compare algorithms");
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            self.grid_demo.ui(ui);
        });
    }
}
