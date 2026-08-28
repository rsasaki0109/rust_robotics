//! Top-level egui application shell.

use crate::admm_formation::AdmmFormationDemo;
use crate::controller_arena::ControllerArenaDemo;
use crate::engagement::Experiment;
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
    onboarding_step: Option<u8>,
    recent_experiments: Vec<Experiment>,
    resume_query: Option<String>,
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
        let mut localization_demo = LocalizationDemo::default();
        localization_demo.apply_share_query(&query);
        let mut slam_demo = SlamDemo::default();
        slam_demo.apply_share_query(&query);
        let mut admm_demo = AdmmFormationDemo::default();
        admm_demo.apply_share_query(&query);
        crate::engagement::track("playground_loaded");
        if !query.is_empty() {
            crate::engagement::track("shared_experiment_opened");
        }
        Self {
            tab,
            grid_demo,
            localization_demo,
            slam_demo,
            admm_demo,
            controller_arena_demo,
            share_status: None,
            onboarding_step: (!crate::engagement::onboarding_complete() && query.is_empty())
                .then_some(0),
            recent_experiments: crate::engagement::recent_experiments(),
            resume_query: query
                .is_empty()
                .then(crate::engagement::last_query)
                .flatten(),
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
            PlaygroundTab::Localization => self.localization_demo.share_query(),
            PlaygroundTab::Slam => self.slam_demo.share_query(),
            PlaygroundTab::AdmmFormation => self.admm_demo.share_query(),
            PlaygroundTab::ControllerArena => self.controller_arena_demo.share_query(),
        }
    }

    fn current_label(&self) -> String {
        format!("{} experiment", Self::tab_label(self.tab))
    }

    fn apply_query(&mut self, query: &str) {
        if let Some(tab) = crate::share::value(query, "tab").and_then(PlaygroundTab::from_slug) {
            self.tab = tab;
        }
        self.grid_demo.apply_share_query(query);
        self.localization_demo.apply_share_query(query);
        self.slam_demo.apply_share_query(query);
        self.admm_demo.apply_share_query(query);
        self.controller_arena_demo.apply_share_query(query);
        self.share_status = None;
    }

    fn save_current_experiment(&mut self) {
        let query = self.share_query();
        self.recent_experiments = crate::engagement::save_experiment(&self.current_label(), &query);
        self.resume_query = Some(query);
    }

    fn onboarding_ui(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        let Some(step) = self.onboarding_step else {
            return;
        };
        egui::Frame::group(ui.style())
            .fill(egui::Color32::from_rgb(27, 39, 54))
            .show(ui, |ui| {
                ui.horizontal_wrapped(|ui| match step {
                    0 => {
                        ui.strong("30-second mission");
                        ui.label("Compare four planners on the same map, then save a reproducible result.");
                        if ui.button("Start mission").clicked() {
                            self.tab = PlaygroundTab::GridPlanners;
                            self.onboarding_step = Some(1);
                            crate::engagement::track("preset_started");
                        }
                        if ui.small_button("Skip").clicked() {
                            self.onboarding_step = None;
                            crate::engagement::mark_onboarding_complete();
                            crate::engagement::track("onboarding_skipped");
                        }
                    }
                    1 => {
                        ui.strong("Step 1 of 2");
                        ui.label("Run A*, Dijkstra, JPS, and Theta* on the current obstacle map.");
                        if ui.button("Compare all planners").clicked() {
                            self.grid_demo.run_guided_comparison();
                            self.onboarding_step = Some(2);
                            crate::engagement::track("experiment_completed");
                        }
                    }
                    _ => {
                        ui.strong("Result ready");
                        ui.label("Save the exact map, endpoints, and selected planner for your next visit.");
                        if ui.button("Save result and finish").clicked() {
                            let url = crate::share::share_url(&self.share_query());
                            ctx.copy_text(url);
                            self.save_current_experiment();
                            self.share_status = Some("Saved and copied!");
                            self.onboarding_step = None;
                            crate::engagement::mark_onboarding_complete();
                            crate::engagement::track("onboarding_completed");
                            crate::engagement::track("share_link_copied");
                        }
                    }
                });
            });
        ui.add_space(6.0);
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
                        if self.tab != tab {
                            self.save_current_experiment();
                            crate::engagement::track("tab_changed");
                        }
                        self.tab = tab;
                        self.share_status = None;
                    }
                }
                ui.separator();
                if ui.button("Copy share link").clicked() {
                    let url = crate::share::share_url(&self.share_query());
                    ctx.copy_text(url);
                    self.save_current_experiment();
                    self.share_status = Some("Copied!");
                    crate::engagement::track("share_link_copied");
                }
                let recent = self.recent_experiments.clone();
                ui.menu_button("Recent experiments", |ui| {
                    if recent.is_empty() {
                        ui.label("No saved experiments yet");
                    }
                    for experiment in recent {
                        if ui.button(&experiment.label).clicked() {
                            self.apply_query(&experiment.query);
                            crate::engagement::track("returning_experiment_resumed");
                            ui.close_menu();
                        }
                    }
                });
                if let Some(status) = self.share_status {
                    ui.label(status);
                }
            });
            ui.label(Self::tab_hint(self.tab));
            if let Some(query) = self.resume_query.clone() {
                ui.horizontal(|ui| {
                    ui.label("Continue where you left off?");
                    if ui.small_button("Resume last experiment").clicked() {
                        self.apply_query(&query);
                        self.resume_query = None;
                        crate::engagement::track("returning_experiment_resumed");
                    }
                    if ui.small_button("Dismiss").clicked() {
                        self.resume_query = None;
                    }
                });
            }
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            self.onboarding_ui(ctx, ui);
            match self.tab {
                PlaygroundTab::GridPlanners => self.grid_demo.ui(ui),
                PlaygroundTab::Localization => self.localization_demo.ui(ctx, ui),
                PlaygroundTab::Slam => self.slam_demo.ui(ctx, ui),
                PlaygroundTab::AdmmFormation => self.admm_demo.ui(ctx, ui),
                PlaygroundTab::ControllerArena => self.controller_arena_demo.ui(ctx, ui),
            }
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
