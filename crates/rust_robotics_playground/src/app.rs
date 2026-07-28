//! Top-level egui application shell.

use crate::admm_formation::AdmmFormationDemo;
use crate::controller_arena::ControllerArenaDemo;
use crate::grid_planners::GridPlannerDemo;
use crate::localization::LocalizationDemo;
use crate::slam::SlamDemo;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PlaygroundTab {
    GridPlanners,
    Localization,
    Slam,
    AdmmFormation,
    ControllerArena,
}

pub struct PlaygroundApp {
    tab: PlaygroundTab,
    grid_demo: GridPlannerDemo,
    localization_demo: LocalizationDemo,
    slam_demo: SlamDemo,
    admm_demo: AdmmFormationDemo,
    controller_arena_demo: ControllerArenaDemo,
    share_status: Option<&'static str>,
}

impl PlaygroundApp {
    pub fn new(_ctx: &eframe::CreationContext<'_>) -> Self {
        let query = crate::share::current_query();
        let tab = crate::share::value(&query, "tab")
            .and_then(PlaygroundTab::from_slug)
            .unwrap_or(PlaygroundTab::GridPlanners);
        let mut grid_demo = GridPlannerDemo::default();
        grid_demo.apply_share_query(&query);
        let mut controller_arena_demo = ControllerArenaDemo::default();
        controller_arena_demo.apply_share_query(&query);
        Self {
            tab,
            grid_demo,
            localization_demo: LocalizationDemo::default(),
            slam_demo: SlamDemo::default(),
            admm_demo: AdmmFormationDemo::default(),
            controller_arena_demo,
            share_status: None,
        }
    }

    fn tab_label(tab: PlaygroundTab) -> &'static str {
        match tab {
            PlaygroundTab::GridPlanners => "Grid Planners",
            PlaygroundTab::Localization => "Localization",
            PlaygroundTab::Slam => "SLAM",
            PlaygroundTab::AdmmFormation => "ADMM Formation",
            PlaygroundTab::ControllerArena => "Controller Arena",
        }
    }

    fn share_query(&self) -> String {
        match self.tab {
            PlaygroundTab::GridPlanners => self.grid_demo.share_query(),
            PlaygroundTab::ControllerArena => self.controller_arena_demo.share_query(),
            tab => format!("tab={}", tab.slug()),
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
            PlaygroundTab::Slam => {
                "Scrub the timeline to replay EKF-SLAM, FastSLAM, or ICP scan matching on a canned loop"
            }
            PlaygroundTab::AdmmFormation => {
                "Receding-horizon ADMM formation: four agents track a noisy moving goal past an L-corner"
            }
            PlaygroundTab::ControllerArena => {
                "Replay Pure Pursuit / Stanley / LQR Steer under identical paths and dynamics"
            }
        }
    }
}

impl PlaygroundTab {
    fn slug(self) -> &'static str {
        match self {
            Self::GridPlanners => "grid",
            Self::Localization => "localization",
            Self::Slam => "slam",
            Self::AdmmFormation => "admm",
            Self::ControllerArena => "arena",
        }
    }

    fn from_slug(value: &str) -> Option<Self> {
        match value {
            "grid" => Some(Self::GridPlanners),
            "localization" => Some(Self::Localization),
            "slam" => Some(Self::Slam),
            "admm" => Some(Self::AdmmFormation),
            "arena" => Some(Self::ControllerArena),
            _ => None,
        }
    }
}

impl eframe::App for PlaygroundApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::TopBottomPanel::top("header").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.heading("RustRobotics Playground");
                ui.separator();
                for tab in [
                    PlaygroundTab::GridPlanners,
                    PlaygroundTab::Localization,
                    PlaygroundTab::Slam,
                    PlaygroundTab::AdmmFormation,
                    PlaygroundTab::ControllerArena,
                ] {
                    if ui
                        .selectable_label(self.tab == tab, Self::tab_label(tab))
                        .clicked()
                    {
                        self.tab = tab;
                        self.share_status = None;
                    }
                }
                ui.separator();
                if ui.button("Copy share link").clicked() {
                    let url = crate::share::share_url(&self.share_query());
                    ctx.copy_text(url);
                    self.share_status = Some("Copied!");
                }
                if let Some(status) = self.share_status {
                    ui.label(status);
                }
            });
            ui.label(Self::tab_hint(self.tab));
        });

        egui::CentralPanel::default().show(ctx, |ui| match self.tab {
            PlaygroundTab::GridPlanners => self.grid_demo.ui(ui),
            PlaygroundTab::Localization => self.localization_demo.ui(ctx, ui),
            PlaygroundTab::Slam => self.slam_demo.ui(ctx, ui),
            PlaygroundTab::AdmmFormation => self.admm_demo.ui(ctx, ui),
            PlaygroundTab::ControllerArena => self.controller_arena_demo.ui(ctx, ui),
        });

        if matches!(
            self.tab,
            PlaygroundTab::Localization
                | PlaygroundTab::Slam
                | PlaygroundTab::AdmmFormation
                | PlaygroundTab::ControllerArena
        ) {
            ctx.request_repaint();
        }
    }
}
