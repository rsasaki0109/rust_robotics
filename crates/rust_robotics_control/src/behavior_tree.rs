//! Behavior Tree implementation for mission planning.
//!
//! This module provides a compact behavior tree runtime suitable for
//! robotics decision making with a shared blackboard.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BehaviorStatus {
    Success,
    Failure,
    Running,
}

#[derive(Debug, Clone, Copy)]
pub struct Blackboard {
    pub battery_level: f64,
    pub obstacle_detected: bool,
    pub path_ready: bool,
    pub goal_visible: bool,
}

impl Default for Blackboard {
    fn default() -> Self {
        Self {
            battery_level: 100.0,
            obstacle_detected: false,
            path_ready: true,
            goal_visible: true,
        }
    }
}

pub trait BehaviorNode {
    fn tick(&mut self, blackboard: &Blackboard) -> BehaviorStatus;
    fn reset(&mut self) {}
    fn name(&self) -> &str;
}

pub struct BehaviorTree {
    root: Box<dyn BehaviorNode>,
}

impl BehaviorTree {
    pub fn new(root: Box<dyn BehaviorNode>) -> Self {
        Self { root }
    }

    pub fn tick(&mut self, blackboard: &Blackboard) -> BehaviorStatus {
        self.root.tick(blackboard)
    }

    pub fn reset(&mut self) {
        self.root.reset();
    }

    pub fn root_name(&self) -> &str {
        self.root.name()
    }
}

pub struct ConditionNode {
    name: String,
    evaluator: Box<dyn Fn(&Blackboard) -> bool>,
}

impl ConditionNode {
    pub fn new(name: impl Into<String>, evaluator: impl Fn(&Blackboard) -> bool + 'static) -> Self {
        Self {
            name: name.into(),
            evaluator: Box::new(evaluator),
        }
    }
}

impl BehaviorNode for ConditionNode {
    fn tick(&mut self, blackboard: &Blackboard) -> BehaviorStatus {
        if (self.evaluator)(blackboard) {
            BehaviorStatus::Success
        } else {
            BehaviorStatus::Failure
        }
    }

    fn name(&self) -> &str {
        &self.name
    }
}

pub struct ActionNode {
    name: String,
    action: Box<dyn FnMut(&Blackboard) -> BehaviorStatus>,
}

impl ActionNode {
    pub fn new(
        name: impl Into<String>,
        action: impl FnMut(&Blackboard) -> BehaviorStatus + 'static,
    ) -> Self {
        Self {
            name: name.into(),
            action: Box::new(action),
        }
    }
}

impl BehaviorNode for ActionNode {
    fn tick(&mut self, blackboard: &Blackboard) -> BehaviorStatus {
        (self.action)(blackboard)
    }

    fn name(&self) -> &str {
        &self.name
    }
}

pub struct SequenceNode {
    name: String,
    children: Vec<Box<dyn BehaviorNode>>,
    current_index: usize,
}

impl SequenceNode {
    pub fn new(name: impl Into<String>, children: Vec<Box<dyn BehaviorNode>>) -> Self {
        Self {
            name: name.into(),
            children,
            current_index: 0,
        }
    }
}

impl BehaviorNode for SequenceNode {
    fn tick(&mut self, blackboard: &Blackboard) -> BehaviorStatus {
        while self.current_index < self.children.len() {
            match self.children[self.current_index].tick(blackboard) {
                BehaviorStatus::Success => {
                    self.children[self.current_index].reset();
                    self.current_index += 1;
                }
                BehaviorStatus::Failure => {
                    self.reset();
                    return BehaviorStatus::Failure;
                }
                BehaviorStatus::Running => return BehaviorStatus::Running,
            }
        }

        self.reset();
        BehaviorStatus::Success
    }

    fn reset(&mut self) {
        self.current_index = 0;
        for child in &mut self.children {
            child.reset();
        }
    }

    fn name(&self) -> &str {
        &self.name
    }
}

pub struct SelectorNode {
    name: String,
    children: Vec<Box<dyn BehaviorNode>>,
    current_index: usize,
}

impl SelectorNode {
    pub fn new(name: impl Into<String>, children: Vec<Box<dyn BehaviorNode>>) -> Self {
        Self {
            name: name.into(),
            children,
            current_index: 0,
        }
    }
}

impl BehaviorNode for SelectorNode {
    fn tick(&mut self, blackboard: &Blackboard) -> BehaviorStatus {
        while self.current_index < self.children.len() {
            match self.children[self.current_index].tick(blackboard) {
                BehaviorStatus::Success => {
                    self.reset();
                    return BehaviorStatus::Success;
                }
                BehaviorStatus::Failure => {
                    self.children[self.current_index].reset();
                    self.current_index += 1;
                }
                BehaviorStatus::Running => return BehaviorStatus::Running,
            }
        }

        self.reset();
        BehaviorStatus::Failure
    }

    fn reset(&mut self) {
        self.current_index = 0;
        for child in &mut self.children {
            child.reset();
        }
    }

    fn name(&self) -> &str {
        &self.name
    }
}

pub fn condition(
    name: impl Into<String>,
    evaluator: impl Fn(&Blackboard) -> bool + 'static,
) -> Box<dyn BehaviorNode> {
    Box::new(ConditionNode::new(name, evaluator))
}

pub fn action(
    name: impl Into<String>,
    executor: impl FnMut(&Blackboard) -> BehaviorStatus + 'static,
) -> Box<dyn BehaviorNode> {
    Box::new(ActionNode::new(name, executor))
}

pub fn sequence(
    name: impl Into<String>,
    children: Vec<Box<dyn BehaviorNode>>,
) -> Box<dyn BehaviorNode> {
    Box::new(SequenceNode::new(name, children))
}

pub fn selector(
    name: impl Into<String>,
    children: Vec<Box<dyn BehaviorNode>>,
) -> Box<dyn BehaviorNode> {
    Box::new(SelectorNode::new(name, children))
}

pub fn create_demo_behavior_tree() -> BehaviorTree {
    BehaviorTree::new(selector(
        "mission_root",
        vec![
            sequence(
                "navigate",
                vec![
                    condition("battery_ok", |bb| bb.battery_level > 20.0),
                    condition("path_ready", |bb| bb.path_ready),
                    condition("goal_visible", |bb| bb.goal_visible),
                    action("move_to_goal", |_| BehaviorStatus::Success),
                ],
            ),
            sequence(
                "avoid",
                vec![
                    condition("obstacle_detected", |bb| bb.obstacle_detected),
                    action("hover_and_replan", |_| BehaviorStatus::Running),
                ],
            ),
            action("hold_position", |_| BehaviorStatus::Running),
        ],
    ))
}

pub fn demo_behavior_tree() {
    let scenarios = [
        (
            "Nominal navigation",
            Blackboard {
                battery_level: 90.0,
                obstacle_detected: false,
                path_ready: true,
                goal_visible: true,
            },
        ),
        (
            "Obstacle avoidance",
            Blackboard {
                battery_level: 90.0,
                obstacle_detected: true,
                path_ready: false,
                goal_visible: false,
            },
        ),
        (
            "Low battery hold",
            Blackboard {
                battery_level: 5.0,
                obstacle_detected: false,
                path_ready: false,
                goal_visible: false,
            },
        ),
    ];

    let mut tree = create_demo_behavior_tree();
    println!("Behavior tree root: {}", tree.root_name());

    for (label, blackboard) in scenarios {
        tree.reset();
        let status = tree.tick(&blackboard);
        println!(
            "{} => {:?} (battery={:.1}, obstacle={}, path_ready={}, goal_visible={})",
            label,
            status,
            blackboard.battery_level,
            blackboard.obstacle_detected,
            blackboard.path_ready,
            blackboard.goal_visible
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sequence_succeeds_when_all_children_succeed() {
        let mut tree = BehaviorTree::new(sequence(
            "sequence",
            vec![
                condition("battery_ok", |bb| bb.battery_level > 20.0),
                action("move", |_| BehaviorStatus::Success),
            ],
        ));

        let status = tree.tick(&Blackboard::default());

        assert_eq!(status, BehaviorStatus::Success);
    }

    #[test]
    fn test_selector_uses_fallback_branch() {
        let mut tree = BehaviorTree::new(selector(
            "selector",
            vec![
                condition("goal_visible", |bb| bb.goal_visible),
                action("fallback", |_| BehaviorStatus::Success),
            ],
        ));

        let status = tree.tick(&Blackboard {
            goal_visible: false,
            ..Default::default()
        });

        assert_eq!(status, BehaviorStatus::Success);
    }

    #[test]
    fn test_running_action_resumes_until_success() {
        let mut tree = BehaviorTree::new(sequence(
            "warmup",
            vec![
                action("spin_up", {
                    let mut count = 0;
                    move |_| {
                        count += 1;
                        if count < 2 {
                            BehaviorStatus::Running
                        } else {
                            BehaviorStatus::Success
                        }
                    }
                }),
                action("execute", |_| BehaviorStatus::Success),
            ],
        ));

        assert_eq!(tree.tick(&Blackboard::default()), BehaviorStatus::Running);
        assert_eq!(tree.tick(&Blackboard::default()), BehaviorStatus::Success);
    }

    #[test]
    fn test_demo_tree_holds_position_on_low_battery() {
        let mut tree = create_demo_behavior_tree();
        let status = tree.tick(&Blackboard {
            battery_level: 5.0,
            obstacle_detected: false,
            path_ready: false,
            goal_visible: false,
        });

        assert_eq!(status, BehaviorStatus::Running);
    }
}
