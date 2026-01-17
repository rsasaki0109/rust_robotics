/*!
 * State Machine implementation for Mission Planning
 * 
 * This module implements a finite state machine for robot behavior management.
 * It provides a flexible framework for defining states, transitions, events,
 * guards, and actions in robotics applications.
 * 
 * Ported from PythonRobotics
 * Original author: Wang Zheng (@Aglargil)
 */

use std::collections::HashMap;
use std::fmt;

/// Type alias for callback functions
pub type CallbackFn = Box<dyn Fn() -> ()>;
pub type GuardFn = Box<dyn Fn() -> bool>;
pub type ActionFn = Box<dyn Fn() -> ()>;

/// Represents a state in the state machine
#[derive(Clone, PartialEq)]
pub struct State {
    pub name: String,
    on_enter: Option<String>, // Store function name as string for demo purposes
    on_exit: Option<String>,
}

impl State {
    /// Create a new state
    pub fn new(name: &str) -> Self {
        State {
            name: name.to_string(),
            on_enter: None,
            on_exit: None,
        }
    }

    /// Create a new state with callbacks
    pub fn with_callbacks(name: &str, on_enter: Option<&str>, on_exit: Option<&str>) -> Self {
        State {
            name: name.to_string(),
            on_enter: on_enter.map(|s| s.to_string()),
            on_exit: on_exit.map(|s| s.to_string()),
        }
    }

    /// Enter the state
    pub fn enter(&self) {
        println!("entering <{}>", self.name);
        if let Some(ref callback) = self.on_enter {
            println!("  executing on_enter: {}", callback);
        }
    }

    /// Exit the state
    pub fn exit(&self) {
        println!("exiting <{}>", self.name);
        if let Some(ref callback) = self.on_exit {
            println!("  executing on_exit: {}", callback);
        }
    }
}

impl fmt::Display for State {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.name)
    }
}

/// Represents a transition in the state machine
#[derive(Clone)]
pub struct Transition {
    pub src_state: String,
    pub event: String,
    pub dst_state: String,
    pub guard: Option<String>,
    pub action: Option<String>,
}

impl Transition {
    pub fn new(src_state: &str, event: &str, dst_state: &str) -> Self {
        Transition {
            src_state: src_state.to_string(),
            event: event.to_string(),
            dst_state: dst_state.to_string(),
            guard: None,
            action: None,
        }
    }

    pub fn with_guard(mut self, guard: &str) -> Self {
        self.guard = Some(guard.to_string());
        self
    }

    pub fn with_action(mut self, action: &str) -> Self {
        self.action = Some(action.to_string());
        self
    }
}

/// Main State Machine implementation
pub struct StateMachine {
    name: String,
    states: HashMap<String, State>,
    events: HashMap<String, String>,
    transitions: HashMap<(String, String), Transition>,
    current_state: Option<State>,
    transition_history: Vec<(String, String, String)>, // (from, event, to)
}

impl StateMachine {
    /// Create a new state machine
    pub fn new(name: &str) -> Self {
        StateMachine {
            name: name.to_string(),
            states: HashMap::new(),
            events: HashMap::new(),
            transitions: HashMap::new(),
            current_state: None,
            transition_history: Vec::new(),
        }
    }

    /// Register a state in the state machine
    pub fn register_state(&mut self, state: State) {
        self.states.insert(state.name.clone(), state);
    }

    /// Register an event in the state machine
    pub fn register_event(&mut self, event: &str) {
        self.events.insert(event.to_string(), event.to_string());
    }

    /// Add a transition to the state machine
    pub fn add_transition(&mut self, transition: Transition) {
        // Register states and events if they don't exist
        if !self.states.contains_key(&transition.src_state) {
            self.register_state(State::new(&transition.src_state));
        }
        if !self.states.contains_key(&transition.dst_state) {
            self.register_state(State::new(&transition.dst_state));
        }
        self.register_event(&transition.event);

        let key = (transition.src_state.clone(), transition.event.clone());
        self.transitions.insert(key, transition);
    }

    /// Set the initial state
    pub fn set_initial_state(&mut self, state_name: &str) {
        if let Some(state) = self.states.get(state_name) {
            self.current_state = Some(state.clone());
            println!("|{}| initial state set to <{}>", self.name, state_name);
            state.enter();
        } else {
            panic!("State '{}' not found", state_name);
        }
    }

    /// Get the current state
    pub fn get_current_state(&self) -> Option<&State> {
        self.current_state.as_ref()
    }

    /// Process an event
    pub fn process(&mut self, event: &str) -> Result<(), String> {
        if let Some(ref current_state) = self.current_state {
            let key = (current_state.name.clone(), event.to_string());
            
            if let Some(transition) = self.transitions.get(&key).cloned() {
                self.execute_transition(&transition, event)
            } else {
                Err(format!(
                    "|{}| invalid transition: <{}> : [{}]",
                    self.name, current_state.name, event
                ))
            }
        } else {
            Err("State machine is not initialized".to_string())
        }
    }

    /// Execute a transition
    fn execute_transition(&mut self, transition: &Transition, event: &str) -> Result<(), String> {
        let current_state = self.current_state.as_ref().unwrap();
        
        // Check guard condition
        if let Some(ref guard) = transition.guard {
            println!("  checking guard: {}", guard);
            // For demo purposes, we'll simulate guard evaluation
            if !self.evaluate_guard(guard) {
                println!(
                    "|{}| skipping transition from <{}> to <{}> because guard [{}] failed",
                    self.name, current_state.name, transition.dst_state, guard
                );
                return Ok(());
            }
        }

        // Execute action
        if let Some(ref action) = transition.action {
            println!("  executing action: {}", action);
            self.execute_action(action);
        }

        // Perform state transition
        if current_state.name != transition.dst_state {
            println!(
                "|{}| transitioning from <{}> to <{}> on event [{}]",
                self.name, current_state.name, transition.dst_state, event
            );
            
            // Record transition history
            self.transition_history.push((
                current_state.name.clone(),
                event.to_string(),
                transition.dst_state.clone(),
            ));

            // Exit current state
            current_state.exit();

            // Enter new state
            if let Some(new_state) = self.states.get(&transition.dst_state) {
                self.current_state = Some(new_state.clone());
                new_state.enter();
            } else {
                return Err(format!("Destination state '{}' not found", transition.dst_state));
            }
        } else {
            println!(
                "|{}| self-transition on <{}> with event [{}]",
                self.name, current_state.name, event
            );
        }

        Ok(())
    }

    /// Evaluate a guard condition (simplified for demo)
    fn evaluate_guard(&self, guard: &str) -> bool {
        match guard {
            "can_start" => true,
            "has_battery" => true,
            "obstacle_detected" => false,
            "goal_reached" => false,
            "emergency" => false,
            _ => true, // Default to true for unknown guards
        }
    }

    /// Execute an action (simplified for demo)
    fn execute_action(&self, action: &str) {
        match action {
            "start_motors" => println!("    -> Starting motors"),
            "stop_motors" => println!("    -> Stopping motors"),
            "play_sound" => println!("    -> Playing notification sound"),
            "save_position" => println!("    -> Saving current position"),
            "send_alert" => println!("    -> Sending emergency alert"),
            _ => println!("    -> Executing action: {}", action),
        }
    }

    /// Get transition history
    pub fn get_transition_history(&self) -> &Vec<(String, String, String)> {
        &self.transition_history
    }

    /// Generate a simple text representation of the state machine
    pub fn generate_diagram(&self) -> String {
        let mut diagram = Vec::new();
        diagram.push(format!("State Machine: {}", self.name));
        diagram.push("".to_string());
        
        if let Some(ref current) = self.current_state {
            diagram.push(format!("Current State: {}", current.name));
            diagram.push("".to_string());
        }

        diagram.push("States:".to_string());
        for state in self.states.values() {
            let marker = if Some(state) == self.current_state.as_ref() { " [CURRENT]" } else { "" };
            diagram.push(format!("  - {}{}", state.name, marker));
        }
        diagram.push("".to_string());

        diagram.push("Transitions:".to_string());
        for transition in self.transitions.values() {
            let mut trans_str = format!("  {} --[{}]--> {}", 
                transition.src_state, transition.event, transition.dst_state);
            
            if let Some(ref guard) = transition.guard {
                trans_str.push_str(&format!(" [guard: {}]", guard));
            }
            if let Some(ref action) = transition.action {
                trans_str.push_str(&format!(" / {}", action));
            }
            diagram.push(trans_str);
        }

        diagram.join("\n")
    }

    /// Create a visualization of the state machine as SVG
    pub fn visualize(&self, filename: &str) {
        use std::io::Write;

        // Get sorted state names for consistent layout
        let mut state_names: Vec<_> = self.states.keys().cloned().collect();
        state_names.sort();
        let n_states = state_names.len();

        if n_states == 0 {
            return;
        }

        // SVG dimensions
        let width = 640.0;
        let height = 640.0;
        let cx = width / 2.0;
        let cy = height / 2.0;
        let radius = 200.0; // Circle radius for state layout
        let node_radius = 40.0;

        // Position states in a circle
        let mut state_positions: HashMap<String, (f64, f64)> = HashMap::new();
        for (i, state_name) in state_names.iter().enumerate() {
            let angle = std::f64::consts::PI / 2.0 - 2.0 * std::f64::consts::PI * i as f64 / n_states as f64;
            let x = cx + radius * angle.cos();
            let y = cy - radius * angle.sin(); // SVG y is inverted
            state_positions.insert(state_name.clone(), (x, y));
        }

        let mut svg = String::new();

        // SVG header
        svg.push_str(&format!(
            r##"<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{}" height="{}" viewBox="0 0 {} {}">
  <defs>
    <marker id="arrowhead" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
      <polygon points="0 0, 10 3.5, 0 7" fill="#666"/>
    </marker>
  </defs>
  <rect width="100%" height="100%" fill="white"/>
  <text x="{}" y="30" text-anchor="middle" font-family="Arial" font-size="18" font-weight="bold">Robot State Machine</text>
"##,
            width, height, width, height, cx
        ));

        // Draw transitions as arrows
        for transition in self.transitions.values() {
            if let (Some(&(x1, y1)), Some(&(x2, y2))) = (
                state_positions.get(&transition.src_state),
                state_positions.get(&transition.dst_state)
            ) {
                if transition.src_state != transition.dst_state {
                    let dx = x2 - x1;
                    let dy = y2 - y1;
                    let dist = (dx * dx + dy * dy).sqrt();

                    // Offset from node centers
                    let start_x = x1 + (node_radius + 5.0) * dx / dist;
                    let start_y = y1 + (node_radius + 5.0) * dy / dist;
                    let end_x = x2 - (node_radius + 10.0) * dx / dist;
                    let end_y = y2 - (node_radius + 10.0) * dy / dist;

                    // Draw arrow line
                    svg.push_str(&format!(
                        r##"  <line x1="{:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}" stroke="#666" stroke-width="2" marker-end="url(#arrowhead)"/>
"##,
                        start_x, start_y, end_x, end_y
                    ));

                    // Event label at midpoint, offset perpendicular to line
                    let mid_x = (start_x + end_x) / 2.0;
                    let mid_y = (start_y + end_y) / 2.0;
                    let perp_x = -dy / dist * 15.0;
                    let perp_y = dx / dist * 15.0;

                    svg.push_str(&format!(
                        r##"  <text x="{:.1}" y="{:.1}" text-anchor="middle" font-family="Arial" font-size="11" fill="#0066cc">{}</text>
"##,
                        mid_x + perp_x, mid_y + perp_y, transition.event
                    ));
                }
            }
        }

        // Draw state nodes
        for state_name in &state_names {
            if let Some(&(x, y)) = state_positions.get(state_name) {
                // Circle
                svg.push_str(&format!(
                    r##"  <circle cx="{:.1}" cy="{:.1}" r="{}" fill="#4682b4" stroke="#2c5574" stroke-width="2"/>
"##,
                    x, y, node_radius
                ));

                // State name label
                svg.push_str(&format!(
                    "  <text x=\"{:.1}\" y=\"{:.1}\" text-anchor=\"middle\" dominant-baseline=\"middle\" font-family=\"Arial\" font-size=\"12\" fill=\"white\" font-weight=\"bold\">{}</text>\n",
                    x, y, state_name
                ));
            }
        }

        // Legend
        svg.push_str(&format!(
            r##"  <text x="{}" y="{}" text-anchor="middle" font-family="Arial" font-size="10" fill="#666">States: circles | Transitions: arrows with event labels</text>
"##,
            cx, height - 15.0
        ));

        svg.push_str("</svg>\n");

        // Determine output filename (convert .png to .svg if needed)
        let output_filename = if filename.ends_with(".png") {
            filename.replace(".png", ".svg")
        } else {
            filename.to_string()
        };

        // Write to file
        if let Ok(mut file) = std::fs::File::create(&output_filename) {
            let _ = file.write_all(svg.as_bytes());
            println!("State machine diagram saved to {}", output_filename);
        }
    }
}

/// Demo robot behavior model
pub struct RobotBehavior {
    pub battery_level: f64,
    pub has_obstacle: bool,
    pub goal_reached: bool,
    pub emergency_stop: bool,
}

impl RobotBehavior {
    pub fn new() -> Self {
        RobotBehavior {
            battery_level: 100.0,
            has_obstacle: false,
            goal_reached: false,
            emergency_stop: false,
        }
    }

    pub fn can_start(&self) -> bool {
        self.battery_level > 20.0 && !self.emergency_stop
    }

    pub fn has_battery(&self) -> bool {
        self.battery_level > 10.0
    }

    pub fn obstacle_detected(&self) -> bool {
        self.has_obstacle
    }
}

/// Create a demo robot state machine
pub fn create_robot_state_machine() -> StateMachine {
    let mut machine = StateMachine::new("RobotController");

    // Define states
    machine.register_state(State::with_callbacks("idle", Some("on_enter_idle"), Some("on_exit_idle")));
    machine.register_state(State::with_callbacks("moving", Some("on_enter_moving"), Some("on_exit_moving")));
    machine.register_state(State::with_callbacks("avoiding", Some("on_enter_avoiding"), None));
    machine.register_state(State::with_callbacks("charging", Some("on_enter_charging"), None));
    machine.register_state(State::with_callbacks("emergency", Some("on_enter_emergency"), None));

    // Define transitions
    machine.add_transition(
        Transition::new("idle", "start", "moving")
            .with_guard("can_start")
            .with_action("start_motors")
    );

    machine.add_transition(
        Transition::new("moving", "obstacle", "avoiding")
            .with_action("stop_motors")
    );

    machine.add_transition(
        Transition::new("avoiding", "clear", "moving")
            .with_action("start_motors")
    );

    machine.add_transition(
        Transition::new("moving", "low_battery", "charging")
            .with_action("save_position")
    );

    machine.add_transition(
        Transition::new("charging", "charged", "idle")
            .with_action("play_sound")
    );

    machine.add_transition(
        Transition::new("moving", "stop", "idle")
            .with_action("stop_motors")
    );

    machine.add_transition(
        Transition::new("idle", "emergency", "emergency")
            .with_action("send_alert")
    );

    machine.add_transition(
        Transition::new("moving", "emergency", "emergency")
            .with_action("send_alert")
    );

    machine.add_transition(
        Transition::new("emergency", "reset", "idle")
    );

    machine
}

/// Run a demo of the state machine
pub fn demo_state_machine() {
    println!("=== Robot State Machine Demo ===\n");

    // Create output directory
    std::fs::create_dir_all("img/mission_planning").unwrap_or_default();

    let mut machine = create_robot_state_machine();
    
    // Set initial state
    machine.set_initial_state("idle");
    
    println!("\n{}\n", machine.generate_diagram());

    // Simulate a sequence of events
    let events = vec![
        "start",
        "obstacle", 
        "clear",
        "low_battery",
        "charged",
        "start",
        "emergency",
        "reset",
    ];

    println!("=== Processing Events ===\n");
    
    for event in events {
        println!("Processing event: [{}]", event);
        match machine.process(event) {
            Ok(()) => println!("  -> Success\n"),
            Err(e) => println!("  -> Error: {}\n", e),
        }
    }

    // Show final state and history
    if let Some(current) = machine.get_current_state() {
        println!("Final state: {}", current.name);
    }

    println!("\n=== Transition History ===");
    for (i, (from, event, to)) in machine.get_transition_history().iter().enumerate() {
        println!("{}. {} --[{}]--> {}", i + 1, from, event, to);
    }

    // Generate visualization
    machine.visualize("img/mission_planning/state_machine_diagram.png");
    println!("\nState machine diagram saved to img/mission_planning/state_machine_diagram.png");

    // Generate final diagram
    println!("\n=== Final State Machine ===");
    println!("{}", machine.generate_diagram());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_state_creation() {
        let state = State::new("test_state");
        assert_eq!(state.name, "test_state");
    }

    #[test]
    fn test_state_machine_creation() {
        let machine = StateMachine::new("test_machine");
        assert_eq!(machine.name, "test_machine");
        assert!(machine.current_state.is_none());
    }

    #[test]
    fn test_state_registration() {
        let mut machine = StateMachine::new("test");
        let state = State::new("idle");
        machine.register_state(state);
        assert!(machine.states.contains_key("idle"));
    }

    #[test]
    fn test_transition_creation() {
        let transition = Transition::new("idle", "start", "running")
            .with_guard("can_start")
            .with_action("start_motors");
        
        assert_eq!(transition.src_state, "idle");
        assert_eq!(transition.event, "start");
        assert_eq!(transition.dst_state, "running");
        assert_eq!(transition.guard, Some("can_start".to_string()));
        assert_eq!(transition.action, Some("start_motors".to_string()));
    }

    #[test]
    fn test_simple_transition() {
        let mut machine = StateMachine::new("test");
        
        machine.add_transition(Transition::new("idle", "start", "running"));
        machine.set_initial_state("idle");
        
        assert!(machine.process("start").is_ok());
        assert_eq!(machine.get_current_state().unwrap().name, "running");
    }
}

fn main() {
    demo_state_machine();
}
