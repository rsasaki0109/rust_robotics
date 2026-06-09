//! Top-level egui application shell.

use crate::grid_planners::GridPlannerDemo;
use crate::localization::LocalizationDemo;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PlaygroundTab {
    GridPlanners,
    Localization,
}

pub struct PlaygroundApp {
    tab: PlaygroundTab,
    grid_demo: GridPlannerDemo,
    localization_demo: LocalizationDemo,
}

impl PlaygroundApp {
    pub fn new(_ctx: &eframe::CreationContext<'_>) -> Self {
        Self {
            tab: PlaygroundTab::GridPlanners,
            grid_demo: GridPlannerDemo::default(),
            localization_demo: LocalizationDemo::default(),
        }
    }

    fn tab_label(tab: PlaygroundTab) -> &'static str {
        match tab {
            PlaygroundTab::GridPlanners => "Grid Planners",
            PlaygroundTab::Localization => "Localization",
        }
    }

    fn tab_hint(tab: PlaygroundTab) -> &'static str {
        match tab {
            PlaygroundTab::GridPlanners => {
                "Click obstacles, drag start/goal, compare A* / Dijkstra / JPS / Theta*"
            }
            PlaygroundTab::Localization => {
                "Arrow keys drive the robot; compare Particle Filter vs EKF under sensor noise"
            }
        }
    }
}

impl eframe::App for PlaygroundApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::TopBottomPanel::top("header").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.heading("RustRobotics Playground");
                ui.separator();
                for tab in [PlaygroundTab::GridPlanners, PlaygroundTab::Localization] {
                    if ui
                        .selectable_label(self.tab == tab, Self::tab_label(tab))
                        .clicked()
                    {
                        self.tab = tab;
                    }
                }
            });
            ui.label(Self::tab_hint(self.tab));
        });

        egui::CentralPanel::default().show(ctx, |ui| match self.tab {
            PlaygroundTab::GridPlanners => self.grid_demo.ui(ui),
            PlaygroundTab::Localization => self.localization_demo.ui(ctx, ui),
        });

        if self.tab == PlaygroundTab::Localization {
            ctx.request_repaint();
        }
    }
}
