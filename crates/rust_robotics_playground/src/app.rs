//! Top-level egui application shell.

use crate::admm_formation::AdmmFormationDemo;
use crate::controller_arena::ControllerArenaDemo;
use crate::engagement::Experiment;
use crate::grid_planners::GridPlannerDemo;
use crate::localization::LocalizationDemo;
use crate::pushing::PushingDemo;
use crate::sampling::SamplingDemo;
use crate::slam::SlamDemo;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PlaygroundTab {
    GridPlanners,
    Sampling,
    Localization,
    Slam,
    AdmmFormation,
    ControllerArena,
    Pushing,
}

pub struct PlaygroundApp {
    tab: PlaygroundTab,
    grid_demo: GridPlannerDemo,
    sampling_demo: SamplingDemo,
    localization_demo: LocalizationDemo,
    slam_demo: SlamDemo,
    admm_demo: AdmmFormationDemo,
    controller_arena_demo: ControllerArenaDemo,
    pushing_demo: PushingDemo,
    share_status: Option<&'static str>,
    onboarding_step: Option<u8>,
    recent_experiments: Vec<Experiment>,
    resume_query: Option<String>,
}

impl PlaygroundApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        crate::ui_kit::apply_style(&cc.egui_ctx);
        let query = crate::share::current_query();
        let tab = crate::share::value(&query, "tab")
            .and_then(PlaygroundTab::from_slug)
            .unwrap_or(PlaygroundTab::GridPlanners);
        let mut grid_demo = GridPlannerDemo::default();
        grid_demo.apply_share_query(&query);
        let mut sampling_demo = SamplingDemo::default();
        sampling_demo.apply_share_query(&query);
        let mut controller_arena_demo = ControllerArenaDemo::default();
        controller_arena_demo.apply_share_query(&query);
        let mut localization_demo = LocalizationDemo::default();
        localization_demo.apply_share_query(&query);
        let mut slam_demo = SlamDemo::default();
        slam_demo.apply_share_query(&query);
        let mut admm_demo = AdmmFormationDemo::default();
        admm_demo.apply_share_query(&query);
        let mut pushing_demo = PushingDemo::default();
        if query.contains("tab=pushing") {
            pushing_demo.apply_share_query(&query);
        }
        crate::engagement::track("playground_loaded");
        if !query.is_empty() {
            crate::engagement::track("shared_experiment_opened");
        }
        Self {
            tab,
            grid_demo,
            sampling_demo,
            localization_demo,
            slam_demo,
            admm_demo,
            controller_arena_demo,
            pushing_demo,
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
            PlaygroundTab::Sampling => "Sampling Planners",
            PlaygroundTab::Localization => "Localization",
            PlaygroundTab::Slam => "SLAM",
            PlaygroundTab::AdmmFormation => "ADMM Formation",
            PlaygroundTab::ControllerArena => "Controller Arena",
            PlaygroundTab::Pushing => "Pushing",
        }
    }

    fn share_query(&self) -> String {
        match self.tab {
            PlaygroundTab::GridPlanners => self.grid_demo.share_query(),
            PlaygroundTab::Sampling => self.sampling_demo.share_query(),
            PlaygroundTab::Localization => self.localization_demo.share_query(),
            PlaygroundTab::Slam => self.slam_demo.share_query(),
            PlaygroundTab::AdmmFormation => self.admm_demo.share_query(),
            PlaygroundTab::ControllerArena => self.controller_arena_demo.share_query(),
            PlaygroundTab::Pushing => self.pushing_demo.share_query(),
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
        self.sampling_demo.apply_share_query(query);
        self.localization_demo.apply_share_query(query);
        self.slam_demo.apply_share_query(query);
        self.admm_demo.apply_share_query(query);
        self.controller_arena_demo.apply_share_query(query);
        if self.tab == PlaygroundTab::Pushing {
            self.pushing_demo.apply_share_query(query);
        }
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
        if self.tab != PlaygroundTab::GridPlanners {
            return;
        }
        egui::Frame::new()
            .fill(crate::ui_kit::ACCENT.gamma_multiply(0.12))
            .stroke(egui::Stroke::new(
                1.0_f32,
                crate::ui_kit::ACCENT.gamma_multiply(0.5),
            ))
            .corner_radius(8.0)
            .inner_margin(10.0)
            .show(ui, |ui| {
                ui.set_width(ui.available_width());
                match step {
                    0 => {
                        ui.strong("30-second mission");
                        ui.label("Race four planners on one map, then save the result.");
                        ui.horizontal(|ui| {
                            if ui.button("Start").clicked() {
                                self.onboarding_step = Some(1);
                                crate::engagement::track("preset_started");
                            }
                            if ui.small_button("Skip").clicked() {
                                self.onboarding_step = None;
                                crate::engagement::mark_onboarding_complete();
                                crate::engagement::track("onboarding_skipped");
                            }
                        });
                    }
                    1 => {
                        ui.strong("Step 1 of 2");
                        ui.label("Run A*, Dijkstra, JPS, and Theta* on the current map.");
                        if ui.button("Compare all planners").clicked() {
                            self.grid_demo.run_guided_comparison();
                            self.onboarding_step = Some(2);
                            crate::engagement::track("experiment_completed");
                        }
                    }
                    _ => {
                        ui.strong("Step 2 of 2");
                        ui.label("Save the map, endpoints, and planner as a link.");
                        if ui.button("Save and copy link").clicked() {
                            let url = crate::share::share_url(&self.share_query());
                            ctx.copy_text(url);
                            self.save_current_experiment();
                            self.share_status = Some("Link copied");
                            self.onboarding_step = None;
                            crate::engagement::mark_onboarding_complete();
                            crate::engagement::track("onboarding_completed");
                            crate::engagement::track("share_link_copied");
                        }
                    }
                }
            });
        ui.add_space(4.0);
    }

    fn resume_ui(&mut self, ui: &mut egui::Ui) {
        let Some(query) = self.resume_query.clone() else {
            return;
        };
        ui.horizontal_wrapped(|ui| {
            if ui.button("↺ Resume last experiment").clicked() {
                self.apply_query(&query);
                self.resume_query = None;
                crate::engagement::track("returning_experiment_resumed");
            }
            if ui.small_button("✕").on_hover_text("Dismiss").clicked() {
                self.resume_query = None;
            }
        });
    }

    fn tab_hint(tab: PlaygroundTab) -> &'static str {
        match tab {
            PlaygroundTab::GridPlanners => "Draw walls, drag start and goal, race four planners.",
            PlaygroundTab::Sampling => {
                "Watch RRT, RRT*, Informed RRT*, and PRM explore around obstacles."
            }
            PlaygroundTab::Localization => {
                "Particle filter vs EKF: steer the robot and watch the estimate."
            }
            PlaygroundTab::Slam => {
                "Drive a robot with live LiDAR SLAM, or replay classic SLAM algorithms."
            }
            PlaygroundTab::AdmmFormation => {
                "Four agents agree on a formation via ADMM while tracking a noisy goal."
            }
            PlaygroundTab::ControllerArena => {
                "Pure Pursuit, Stanley, and LQR on the same course and vehicle."
            }
            PlaygroundTab::Pushing => "Push a box to a goal pose with face-switching MPPI.",
        }
    }
}

impl PlaygroundTab {
    fn from_slug(value: &str) -> Option<Self> {
        match value {
            "grid" => Some(Self::GridPlanners),
            "sampling" => Some(Self::Sampling),
            "localization" => Some(Self::Localization),
            "slam" => Some(Self::Slam),
            "admm" => Some(Self::AdmmFormation),
            "arena" => Some(Self::ControllerArena),
            "pushing" => Some(Self::Pushing),
            _ => None,
        }
    }
}

const TABS: [PlaygroundTab; 7] = [
    PlaygroundTab::GridPlanners,
    PlaygroundTab::Sampling,
    PlaygroundTab::Localization,
    PlaygroundTab::Slam,
    PlaygroundTab::AdmmFormation,
    PlaygroundTab::ControllerArena,
    PlaygroundTab::Pushing,
];

/// Below this width \[points\] the controls move below the scene (phones).
const NARROW_WIDTH: f32 = 700.0;
/// Below this width \[points\] the tab row no longer fits the header and
/// becomes a drop-down.
const TAB_ROW_WIDTH: f32 = 980.0;

impl PlaygroundApp {
    fn controls_ui(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        match self.tab {
            PlaygroundTab::GridPlanners => self.grid_demo.controls(ctx, ui),
            PlaygroundTab::Sampling => self.sampling_demo.controls(ctx, ui),
            PlaygroundTab::Localization => self.localization_demo.controls(ctx, ui),
            PlaygroundTab::Slam => self.slam_demo.controls(ctx, ui),
            PlaygroundTab::AdmmFormation => self.admm_demo.controls(ctx, ui),
            PlaygroundTab::ControllerArena => self.controller_arena_demo.controls(ctx, ui),
            PlaygroundTab::Pushing => self.pushing_demo.controls(ctx, ui),
        }
    }

    fn scene_ui(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        match self.tab {
            PlaygroundTab::GridPlanners => self.grid_demo.scene(ctx, ui),
            PlaygroundTab::Sampling => self.sampling_demo.scene(ctx, ui),
            PlaygroundTab::Localization => self.localization_demo.scene(ctx, ui),
            PlaygroundTab::Slam => self.slam_demo.scene(ctx, ui),
            PlaygroundTab::AdmmFormation => self.admm_demo.scene(ctx, ui),
            PlaygroundTab::ControllerArena => self.controller_arena_demo.scene(ctx, ui),
            PlaygroundTab::Pushing => self.pushing_demo.scene(ctx, ui),
        }
    }

    /// Title, one-line description, and the panel's extras above the controls.
    fn panel_intro(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        ui.add_space(4.0);
        ui.heading(Self::tab_label(self.tab));
        crate::ui_kit::hint(ui, Self::tab_hint(self.tab));
        ui.add_space(2.0);
        self.resume_ui(ui);
        self.onboarding_ui(ctx, ui);
    }

    fn select_tab(&mut self, tab: PlaygroundTab) {
        if self.tab != tab {
            self.save_current_experiment();
            crate::engagement::track("tab_changed");
        }
        self.tab = tab;
        self.share_status = None;
    }

    fn header_ui(&mut self, ctx: &egui::Context, ui: &mut egui::Ui, narrow: bool) {
        let tab_menu = ctx.screen_rect().width() < TAB_ROW_WIDTH;
        ui.horizontal(|ui| {
            ui.hyperlink_to(
                egui::RichText::new("RustRobotics")
                    .strong()
                    .color(egui::Color32::WHITE),
                "../",
            )
            .on_hover_text("Back to the project page");
            ui.add_space(6.0);
            if tab_menu {
                let mut selected = self.tab;
                egui::ComboBox::from_id_salt("tab_select")
                    .selected_text(Self::tab_label(self.tab))
                    .show_ui(ui, |ui| {
                        for tab in TABS {
                            ui.selectable_value(&mut selected, tab, Self::tab_label(tab));
                        }
                    });
                if selected != self.tab {
                    self.select_tab(selected);
                }
            } else {
                for tab in TABS {
                    if ui
                        .selectable_label(self.tab == tab, Self::tab_label(tab))
                        .clicked()
                    {
                        self.select_tab(tab);
                    }
                }
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let recent = self.recent_experiments.clone();
                if !recent.is_empty() {
                    ui.menu_button("⏱", |ui| {
                        ui.label(egui::RichText::new("Recent experiments").small().weak());
                        for experiment in recent {
                            if ui.button(&experiment.label).clicked() {
                                self.apply_query(&experiment.query);
                                crate::engagement::track("returning_experiment_resumed");
                                ui.close_menu();
                            }
                        }
                    })
                    .response
                    .on_hover_text("Recent experiments");
                }
                let share = if narrow { "🔗" } else { "🔗 Share" };
                if ui
                    .button(share)
                    .on_hover_text("Copy a link to this exact experiment")
                    .clicked()
                {
                    let url = crate::share::share_url(&self.share_query());
                    ctx.copy_text(url);
                    self.save_current_experiment();
                    self.share_status = Some("Link copied");
                    crate::engagement::track("share_link_copied");
                }
                if let Some(status) = self.share_status {
                    ui.label(
                        egui::RichText::new(status)
                            .small()
                            .color(crate::ui_kit::ACCENT),
                    );
                }
            });
        });
    }
}

impl eframe::App for PlaygroundApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let narrow = ctx.screen_rect().width() < NARROW_WIDTH;
        let bar = ctx.style().visuals.extreme_bg_color;
        egui::TopBottomPanel::top("header")
            .frame(
                egui::Frame::new()
                    .fill(bar)
                    .inner_margin(egui::Margin::symmetric(12, 8)),
            )
            .show(ctx, |ui| self.header_ui(ctx, ui, narrow));

        if narrow {
            // Phones: the scene first, the controls below it (scroll down).
            egui::CentralPanel::default().show(ctx, |ui| {
                egui::ScrollArea::vertical()
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        self.scene_ui(ctx, ui);
                        ui.separator();
                        self.panel_intro(ctx, ui);
                        self.controls_ui(ctx, ui);
                        ui.add_space(24.0);
                    });
            });
        } else {
            egui::SidePanel::left("controls")
                .resizable(true)
                .default_width(320.0)
                .width_range(260.0..=440.0)
                .frame(
                    egui::Frame::new()
                        .fill(ctx.style().visuals.panel_fill)
                        .inner_margin(egui::Margin::symmetric(14, 10)),
                )
                .show(ctx, |ui| {
                    egui::ScrollArea::vertical()
                        .auto_shrink([false, false])
                        .show(ui, |ui| {
                            self.panel_intro(ctx, ui);
                            ui.separator();
                            self.controls_ui(ctx, ui);
                            ui.add_space(12.0);
                        });
                });
            egui::CentralPanel::default()
                .frame(
                    egui::Frame::new()
                        .fill(ctx.style().visuals.extreme_bg_color)
                        .inner_margin(16),
                )
                .show(ctx, |ui| self.scene_ui(ctx, ui));
        }

        if self.tab == PlaygroundTab::Localization {
            ctx.request_repaint();
        }
    }
}
