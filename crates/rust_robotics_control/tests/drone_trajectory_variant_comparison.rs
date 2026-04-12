use rust_robotics_control::experiments::drone_trajectory_quality::{
    coupled_continuity_drone_trajectory_cases, default_drone_controller_variants,
    default_drone_trajectory_cases, default_drone_trajectory_variants,
    evaluate_drone_controller_variants, evaluate_drone_trajectory_variants,
    evaluate_lateral_gain_case, pass_through_accel_drone_trajectory_cases,
    pass_through_accel_jerk_drone_trajectory_cases, pass_through_drone_trajectory_cases,
    DroneControllerVariant, DroneTrajectoryVariant, LateralGainSet,
};

fn find_summary(
    summaries: &[rust_robotics_control::experiments::drone_trajectory_quality::DroneTrajectorySummary],
    variant: DroneTrajectoryVariant,
) -> &rust_robotics_control::experiments::drone_trajectory_quality::DroneTrajectorySummary {
    summaries
        .iter()
        .find(|summary| summary.variant == variant)
        .expect("variant summary should exist")
}

fn find_controller_summary(
    summaries: &[rust_robotics_control::experiments::drone_trajectory_quality::DroneControllerSummary],
    variant: DroneControllerVariant,
) -> &rust_robotics_control::experiments::drone_trajectory_quality::DroneControllerSummary {
    summaries
        .iter()
        .find(|summary| summary.controller_variant == variant)
        .expect("controller summary should exist")
}

fn find_controller_metric<'a>(
    metrics: &'a [rust_robotics_control::experiments::drone_trajectory_quality::DroneControllerMetrics],
    family_name: &str,
    trajectory_variant: DroneTrajectoryVariant,
    controller_variant: DroneControllerVariant,
) -> &'a rust_robotics_control::experiments::drone_trajectory_quality::DroneControllerMetrics {
    metrics
        .iter()
        .find(|metric| {
            metric.family_name == family_name
                && metric.trajectory_variant == trajectory_variant
                && metric.controller_variant == controller_variant
        })
        .expect("controller metric should exist")
}

#[derive(Debug, Clone)]
struct ControllerTrajectorySummary {
    trajectory_variant: DroneTrajectoryVariant,
    controller_variant: DroneControllerVariant,
    mean_generation_us: f64,
    mean_tracking_rmse_m: f64,
    mean_max_position_error_m: f64,
    mean_thrust_n: f64,
    mean_max_thrust_n: f64,
    mean_torque_norm_nm: f64,
}

fn summarize_controller_trajectory(
    metrics: &[rust_robotics_control::experiments::drone_trajectory_quality::DroneControllerMetrics],
    trajectory_variant: DroneTrajectoryVariant,
    controller_variant: DroneControllerVariant,
) -> ControllerTrajectorySummary {
    let rows: Vec<_> = metrics
        .iter()
        .filter(|row| {
            row.trajectory_variant == trajectory_variant
                && row.controller_variant == controller_variant
        })
        .collect();
    let denom = rows.len() as f64;

    ControllerTrajectorySummary {
        trajectory_variant,
        controller_variant,
        mean_generation_us: rows.iter().map(|row| row.generation_us).sum::<f64>() / denom,
        mean_tracking_rmse_m: rows.iter().map(|row| row.tracking_rmse_m).sum::<f64>() / denom,
        mean_max_position_error_m: rows.iter().map(|row| row.max_position_error_m).sum::<f64>()
            / denom,
        mean_thrust_n: rows.iter().map(|row| row.mean_thrust_n).sum::<f64>() / denom,
        mean_max_thrust_n: rows.iter().map(|row| row.max_thrust_n).sum::<f64>() / denom,
        mean_torque_norm_nm: rows.iter().map(|row| row.mean_torque_norm_nm).sum::<f64>() / denom,
    }
}

#[test]
fn drone_trajectory_variants_report_quality_tradeoffs() {
    let cases = default_drone_trajectory_cases();
    let variants = default_drone_trajectory_variants();
    let (metrics, summaries) = evaluate_drone_trajectory_variants(&cases, &variants);

    println!(
        "variant                     style                         mean us   mean rmse  mean jerk  mean snap  mean vmax  mean amax  total s"
    );
    println!(
        "-------------------------------------------------------------------------------------------------------------------------------"
    );
    for summary in &summaries {
        println!(
            "{:<27} {:<29} {:>8.2} {:>10.4} {:>10.4} {:>10.4} {:>10.4} {:>10.4} {:>8.2}",
            summary.variant.id(),
            summary.variant.design_style(),
            summary.mean_generation_us,
            summary.mean_tracking_rmse_m,
            summary.mean_jerk_rms,
            summary.mean_snap_rms,
            summary.mean_max_speed_mps,
            summary.mean_max_acceleration_mps2,
            summary.mean_total_duration_s,
        );
    }
    println!();
    println!(
        "case               variant                     rmse m    jerk rms  snap rms   max err   vmax   amax"
    );
    println!(
        "---------------------------------------------------------------------------------------------------"
    );
    for row in &metrics {
        println!(
            "{:<18} {:<27} {:>8.4} {:>10.4} {:>10.4} {:>9.4} {:>6.3} {:>6.3}",
            row.family_name,
            row.variant.id(),
            row.tracking_rmse_m,
            row.jerk_rms,
            row.snap_rms,
            row.max_position_error_m,
            row.max_speed_mps,
            row.max_acceleration_mps2,
        );
    }

    let quintic_uniform = find_summary(&summaries, DroneTrajectoryVariant::QuinticUniform);
    let quintic_distance = find_summary(&summaries, DroneTrajectoryVariant::QuinticDistanceScaled);
    let minimum_snap = find_summary(
        &summaries,
        DroneTrajectoryVariant::MinimumSnapDistanceScaled,
    );

    assert!(
        quintic_distance.mean_max_speed_mps < quintic_uniform.mean_max_speed_mps,
        "distance-scaled quintic should reduce peak speed against the uniform baseline"
    );
    assert!(
        quintic_distance.mean_max_acceleration_mps2 < quintic_uniform.mean_max_acceleration_mps2,
        "distance-scaled quintic should reduce peak acceleration against the uniform baseline"
    );
    assert!(
        minimum_snap.mean_generation_us > quintic_distance.mean_generation_us,
        "minimum-snap should remain the more expensive experimental generator on this preset"
    );
}

#[test]
fn drone_trajectory_variants_report_pass_through_tradeoffs() {
    let cases = pass_through_drone_trajectory_cases();
    let variants = default_drone_trajectory_variants();
    let (metrics, summaries) = evaluate_drone_trajectory_variants(&cases, &variants);

    println!(
        "pass-through variant            style                         mean us   mean rmse  mean jerk  mean snap  mean vmax  mean amax  total s"
    );
    println!(
        "------------------------------------------------------------------------------------------------------------------------------------"
    );
    for summary in &summaries {
        println!(
            "{:<31} {:<29} {:>8.2} {:>10.4} {:>10.4} {:>10.4} {:>10.4} {:>10.4} {:>8.2}",
            summary.variant.id(),
            summary.variant.design_style(),
            summary.mean_generation_us,
            summary.mean_tracking_rmse_m,
            summary.mean_jerk_rms,
            summary.mean_snap_rms,
            summary.mean_max_speed_mps,
            summary.mean_max_acceleration_mps2,
            summary.mean_total_duration_s,
        );
    }
    println!();
    println!(
        "pass-through case  variant                     rmse m    jerk rms  snap rms   max err   vmax   amax"
    );
    println!(
        "------------------------------------------------------------------------------------------------------"
    );
    for row in &metrics {
        println!(
            "{:<18} {:<27} {:>8.4} {:>10.4} {:>10.4} {:>9.4} {:>6.3} {:>6.3}",
            row.family_name,
            row.variant.id(),
            row.tracking_rmse_m,
            row.jerk_rms,
            row.snap_rms,
            row.max_position_error_m,
            row.max_speed_mps,
            row.max_acceleration_mps2,
        );
    }

    let quintic_uniform = find_summary(&summaries, DroneTrajectoryVariant::QuinticUniform);
    let quintic_distance = find_summary(&summaries, DroneTrajectoryVariant::QuinticDistanceScaled);
    let minimum_snap = find_summary(
        &summaries,
        DroneTrajectoryVariant::MinimumSnapDistanceScaled,
    );

    assert!(
        quintic_distance.mean_generation_us <= minimum_snap.mean_generation_us,
        "distance-scaled quintic should remain cheaper than minimum-snap on pass-through cases"
    );
    assert!(
        quintic_distance.mean_tracking_rmse_m < minimum_snap.mean_tracking_rmse_m,
        "pass-through preset still favors distance-scaled quintic over minimum-snap on tracking RMSE"
    );
    assert!(
        quintic_distance.mean_snap_rms < minimum_snap.mean_snap_rms,
        "pass-through preset still favors distance-scaled quintic over minimum-snap on snap RMS"
    );
    assert!(
        quintic_uniform.mean_total_duration_s > 0.0
            && quintic_distance.mean_total_duration_s > 0.0
            && minimum_snap.mean_total_duration_s > 0.0,
        "pass-through runs should produce valid durations"
    );
}

#[test]
fn drone_trajectory_variants_report_pass_through_accel_tradeoffs() {
    let cases = pass_through_accel_drone_trajectory_cases();
    let variants = default_drone_trajectory_variants();
    let (metrics, summaries) = evaluate_drone_trajectory_variants(&cases, &variants);

    println!(
        "pass-through+accel variant      style                         mean us   mean rmse  mean jerk  mean snap  mean vmax  mean amax  total s"
    );
    println!(
        "------------------------------------------------------------------------------------------------------------------------------------"
    );
    for summary in &summaries {
        println!(
            "{:<31} {:<29} {:>8.2} {:>10.4} {:>10.4} {:>10.4} {:>10.4} {:>10.4} {:>8.2}",
            summary.variant.id(),
            summary.variant.design_style(),
            summary.mean_generation_us,
            summary.mean_tracking_rmse_m,
            summary.mean_jerk_rms,
            summary.mean_snap_rms,
            summary.mean_max_speed_mps,
            summary.mean_max_acceleration_mps2,
            summary.mean_total_duration_s,
        );
    }
    println!();
    println!(
        "pass-through+accel case variant                     rmse m    jerk rms  snap rms   max err   vmax   amax"
    );
    println!(
        "-------------------------------------------------------------------------------------------------------------"
    );
    for row in &metrics {
        println!(
            "{:<18} {:<27} {:>8.4} {:>10.4} {:>10.4} {:>9.4} {:>6.3} {:>6.3}",
            row.family_name,
            row.variant.id(),
            row.tracking_rmse_m,
            row.jerk_rms,
            row.snap_rms,
            row.max_position_error_m,
            row.max_speed_mps,
            row.max_acceleration_mps2,
        );
    }

    let quintic_uniform = find_summary(&summaries, DroneTrajectoryVariant::QuinticUniform);
    let quintic_distance = find_summary(&summaries, DroneTrajectoryVariant::QuinticDistanceScaled);
    let minimum_snap = find_summary(
        &summaries,
        DroneTrajectoryVariant::MinimumSnapDistanceScaled,
    );

    assert!(
        minimum_snap.mean_generation_us > quintic_distance.mean_generation_us,
        "minimum-snap should remain more expensive than distance-scaled quintic on accel pass-through cases"
    );
    assert!(
        quintic_distance.mean_tracking_rmse_m < minimum_snap.mean_tracking_rmse_m,
        "accel pass-through preset still favors distance-scaled quintic over minimum-snap on tracking RMSE"
    );
    assert!(
        quintic_distance.mean_snap_rms < minimum_snap.mean_snap_rms,
        "accel pass-through preset still favors distance-scaled quintic over minimum-snap on snap RMS"
    );
    assert!(
        quintic_uniform.mean_total_duration_s > 0.0
            && quintic_distance.mean_total_duration_s > 0.0
            && minimum_snap.mean_total_duration_s > 0.0,
        "pass-through+accel runs should produce valid durations"
    );
}

#[test]
fn drone_trajectory_variants_report_pass_through_accel_jerk_tradeoffs() {
    let cases = pass_through_accel_jerk_drone_trajectory_cases();
    let variants = default_drone_trajectory_variants();
    let (metrics, summaries) = evaluate_drone_trajectory_variants(&cases, &variants);

    println!(
        "pass-through+accel+jerk variant style                         mean us   mean rmse  mean jerk  mean snap  mean vmax  mean amax  total s"
    );
    println!(
        "-------------------------------------------------------------------------------------------------------------------------------------"
    );
    for summary in &summaries {
        println!(
            "{:<31} {:<29} {:>8.2} {:>10.4} {:>10.4} {:>10.4} {:>10.4} {:>10.4} {:>8.2}",
            summary.variant.id(),
            summary.variant.design_style(),
            summary.mean_generation_us,
            summary.mean_tracking_rmse_m,
            summary.mean_jerk_rms,
            summary.mean_snap_rms,
            summary.mean_max_speed_mps,
            summary.mean_max_acceleration_mps2,
            summary.mean_total_duration_s,
        );
    }
    println!();
    println!(
        "pass-through+accel+jerk case   variant                     rmse m    jerk rms  snap rms   max err   vmax   amax"
    );
    println!(
        "----------------------------------------------------------------------------------------------------------------"
    );
    for row in &metrics {
        println!(
            "{:<26} {:<27} {:>8.4} {:>10.4} {:>10.4} {:>9.4} {:>6.3} {:>6.3}",
            row.family_name,
            row.variant.id(),
            row.tracking_rmse_m,
            row.jerk_rms,
            row.snap_rms,
            row.max_position_error_m,
            row.max_speed_mps,
            row.max_acceleration_mps2,
        );
    }

    let quintic_uniform = find_summary(&summaries, DroneTrajectoryVariant::QuinticUniform);
    let quintic_distance = find_summary(&summaries, DroneTrajectoryVariant::QuinticDistanceScaled);
    let minimum_snap = find_summary(
        &summaries,
        DroneTrajectoryVariant::MinimumSnapDistanceScaled,
    );

    assert!(
        minimum_snap.mean_generation_us > quintic_distance.mean_generation_us,
        "minimum-snap should remain more expensive than distance-scaled quintic on accel+jerk pass-through cases"
    );
    assert!(
        quintic_distance.mean_tracking_rmse_m < minimum_snap.mean_tracking_rmse_m,
        "accel+jerk pass-through preset should beat minimum-snap on tracking RMSE unless the ranking flips"
    );
    assert!(
        quintic_distance.mean_snap_rms < minimum_snap.mean_snap_rms,
        "accel+jerk pass-through preset should beat minimum-snap on snap RMS unless the ranking flips"
    );
    assert!(
        quintic_uniform.mean_total_duration_s > 0.0
            && quintic_distance.mean_total_duration_s > 0.0
            && minimum_snap.mean_total_duration_s > 0.0,
        "pass-through+accel+jerk runs should produce valid durations"
    );
}

#[test]
fn drone_trajectory_variants_report_coupled_continuity_tradeoffs() {
    let local_cases = pass_through_accel_jerk_drone_trajectory_cases();
    let coupled_cases = coupled_continuity_drone_trajectory_cases();
    let variants = default_drone_trajectory_variants();
    let (_local_metrics, local_summaries) =
        evaluate_drone_trajectory_variants(&local_cases, &variants);
    let (metrics, summaries) = evaluate_drone_trajectory_variants(&coupled_cases, &variants);

    println!(
        "coupled-continuity variant     style                         mean us   mean rmse  mean jerk  mean snap  mean vmax  mean amax  total s"
    );
    println!(
        "------------------------------------------------------------------------------------------------------------------------------------"
    );
    for summary in &summaries {
        println!(
            "{:<31} {:<29} {:>8.2} {:>10.4} {:>10.4} {:>10.4} {:>10.4} {:>10.4} {:>8.2}",
            summary.variant.id(),
            summary.variant.design_style(),
            summary.mean_generation_us,
            summary.mean_tracking_rmse_m,
            summary.mean_jerk_rms,
            summary.mean_snap_rms,
            summary.mean_max_speed_mps,
            summary.mean_max_acceleration_mps2,
            summary.mean_total_duration_s,
        );
    }
    println!();
    println!(
        "coupled case        variant                     rmse m    jerk rms  snap rms   max err   vmax   amax"
    );
    println!(
        "------------------------------------------------------------------------------------------------------"
    );
    for row in &metrics {
        println!(
            "{:<18} {:<27} {:>8.4} {:>10.4} {:>10.4} {:>9.4} {:>6.3} {:>6.3}",
            row.family_name,
            row.variant.id(),
            row.tracking_rmse_m,
            row.jerk_rms,
            row.snap_rms,
            row.max_position_error_m,
            row.max_speed_mps,
            row.max_acceleration_mps2,
        );
    }

    let local_quintic_distance = find_summary(
        &local_summaries,
        DroneTrajectoryVariant::QuinticDistanceScaled,
    );
    let local_minimum_snap = find_summary(
        &local_summaries,
        DroneTrajectoryVariant::MinimumSnapDistanceScaled,
    );
    let quintic_uniform = find_summary(&summaries, DroneTrajectoryVariant::QuinticUniform);
    let quintic_distance = find_summary(&summaries, DroneTrajectoryVariant::QuinticDistanceScaled);
    let minimum_snap = find_summary(
        &summaries,
        DroneTrajectoryVariant::MinimumSnapDistanceScaled,
    );

    assert!(
        minimum_snap.mean_generation_us > quintic_distance.mean_generation_us,
        "minimum-snap should remain more expensive than distance-scaled quintic on coupled continuity cases"
    );
    assert!(
        quintic_distance.mean_jerk_rms < local_quintic_distance.mean_jerk_rms,
        "coupled continuity should reduce distance-scaled quintic jerk against the local accel+jerk preset"
    );
    assert!(
        quintic_distance.mean_snap_rms < local_quintic_distance.mean_snap_rms,
        "coupled continuity should reduce distance-scaled quintic snap against the local accel+jerk preset"
    );
    assert!(
        minimum_snap.mean_jerk_rms < local_minimum_snap.mean_jerk_rms,
        "coupled continuity should reduce minimum-snap jerk against the local accel+jerk preset"
    );
    assert!(
        minimum_snap.mean_snap_rms < local_minimum_snap.mean_snap_rms,
        "coupled continuity should reduce minimum-snap snap against the local accel+jerk preset"
    );
    assert!(
        quintic_distance.mean_tracking_rmse_m >= local_quintic_distance.mean_tracking_rmse_m,
        "coupled continuity currently trades tracking RMSE for smoother distance-scaled quintic boundary terms"
    );
    assert!(
        minimum_snap.mean_tracking_rmse_m >= local_minimum_snap.mean_tracking_rmse_m,
        "coupled continuity currently trades tracking RMSE for smoother minimum-snap boundary terms"
    );
    assert!(
        quintic_uniform.mean_total_duration_s > 0.0
            && quintic_distance.mean_total_duration_s > 0.0
            && minimum_snap.mean_total_duration_s > 0.0,
        "coupled continuity runs should produce valid durations"
    );
}

#[test]
fn drone_controller_variants_report_coupled_continuity_tradeoffs() {
    let cases = coupled_continuity_drone_trajectory_cases();
    let trajectory_variants = default_drone_trajectory_variants();
    let controller_variants = default_drone_controller_variants();
    let (metrics, summaries) =
        evaluate_drone_controller_variants(&cases, &trajectory_variants, &controller_variants);

    println!(
        "controller variant   style                           mean rmse  mean maxerr  mean thrust  mean maxthrust  mean torque"
    );
    println!(
        "----------------------------------------------------------------------------------------------------------------------"
    );
    for summary in &summaries {
        println!(
            "{:<20} {:<31} {:>10.4} {:>12.4} {:>12.4} {:>15.4} {:>12.4}",
            summary.controller_variant.id(),
            summary.controller_variant.design_style(),
            summary.mean_tracking_rmse_m,
            summary.mean_max_position_error_m,
            summary.mean_thrust_n,
            summary.mean_max_thrust_n,
            summary.mean_torque_norm_nm,
        );
    }
    println!();
    println!(
        "family              trajectory                  controller          rmse m    max err   thrust   maxthrust   torque"
    );
    println!(
        "----------------------------------------------------------------------------------------------------------------"
    );
    for row in &metrics {
        println!(
            "{:<18} {:<27} {:<19} {:>8.4} {:>10.4} {:>8.4} {:>11.4} {:>8.4}",
            row.family_name,
            row.trajectory_variant.id(),
            row.controller_variant.id(),
            row.tracking_rmse_m,
            row.max_position_error_m,
            row.mean_thrust_n,
            row.max_thrust_n,
            row.mean_torque_norm_nm,
        );
    }

    let baseline = find_controller_summary(&summaries, DroneControllerVariant::BaselinePd);
    let damped = find_controller_summary(&summaries, DroneControllerVariant::AttitudeRateDampedPd);
    let lateral = find_controller_summary(
        &summaries,
        DroneControllerVariant::AttitudeRateDampedLateralPd,
    );
    let figure_eight_distance_damped = find_controller_metric(
        &metrics,
        "figure-eight-climb-coupled",
        DroneTrajectoryVariant::QuinticDistanceScaled,
        DroneControllerVariant::AttitudeRateDampedPd,
    );
    let figure_eight_distance_lateral = find_controller_metric(
        &metrics,
        "figure-eight-climb-coupled",
        DroneTrajectoryVariant::QuinticDistanceScaled,
        DroneControllerVariant::AttitudeRateDampedLateralPd,
    );
    let figure_eight_minimum_snap_damped = find_controller_metric(
        &metrics,
        "figure-eight-climb-coupled",
        DroneTrajectoryVariant::MinimumSnapDistanceScaled,
        DroneControllerVariant::AttitudeRateDampedPd,
    );
    let figure_eight_minimum_snap_lateral = find_controller_metric(
        &metrics,
        "figure-eight-climb-coupled",
        DroneTrajectoryVariant::MinimumSnapDistanceScaled,
        DroneControllerVariant::AttitudeRateDampedLateralPd,
    );
    let oval_distance_damped = find_controller_metric(
        &metrics,
        "oval-cruise-coupled",
        DroneTrajectoryVariant::QuinticDistanceScaled,
        DroneControllerVariant::AttitudeRateDampedPd,
    );
    let oval_distance_lateral = find_controller_metric(
        &metrics,
        "oval-cruise-coupled",
        DroneTrajectoryVariant::QuinticDistanceScaled,
        DroneControllerVariant::AttitudeRateDampedLateralPd,
    );
    let banked_distance_damped = find_controller_metric(
        &metrics,
        "banked-diamond-coupled",
        DroneTrajectoryVariant::QuinticDistanceScaled,
        DroneControllerVariant::AttitudeRateDampedPd,
    );
    let banked_distance_lateral = find_controller_metric(
        &metrics,
        "banked-diamond-coupled",
        DroneTrajectoryVariant::QuinticDistanceScaled,
        DroneControllerVariant::AttitudeRateDampedLateralPd,
    );

    assert!(
        damped.mean_tracking_rmse_m < baseline.mean_tracking_rmse_m,
        "attitude-rate damping should still reduce mean tracking RMSE against the baseline PD controller on coupled continuity cases"
    );
    assert!(
        lateral.mean_tracking_rmse_m < damped.mean_tracking_rmse_m,
        "bounded lateral feedback should improve mean tracking RMSE against the pure attitude-rate damping variant on coupled continuity cases"
    );
    assert!(
        lateral.mean_max_position_error_m < damped.mean_max_position_error_m,
        "bounded lateral feedback should recover the mean max position error increase introduced by the pure attitude-rate damping variant"
    );
    assert!(
        lateral.mean_torque_norm_nm < baseline.mean_torque_norm_nm,
        "bounded lateral feedback should keep torque demand well below the baseline controller on coupled continuity cases"
    );
    assert!(
        lateral.mean_max_thrust_n < baseline.mean_max_thrust_n,
        "bounded lateral feedback should keep peak thrust demand below the baseline controller on coupled continuity cases"
    );
    assert!(
        figure_eight_distance_lateral.tracking_rmse_m < figure_eight_distance_damped.tracking_rmse_m
            && figure_eight_minimum_snap_lateral.tracking_rmse_m
                < figure_eight_minimum_snap_damped.tracking_rmse_m,
        "bounded lateral feedback should recover the figure-eight regressions seen under pure attitude-rate damping"
    );
    assert!(
        oval_distance_lateral.tracking_rmse_m < oval_distance_damped.tracking_rmse_m
            && banked_distance_lateral.tracking_rmse_m < banked_distance_damped.tracking_rmse_m,
        "bounded lateral feedback should preserve or improve the oval and banked wins from the damped controller family"
    );
}

#[test]
fn drone_controller_variants_report_noncoupled_tradeoffs() {
    let trajectory_variants = default_drone_trajectory_variants();
    let controller_variants = default_drone_controller_variants();
    let presets = vec![
        ("stop-go", default_drone_trajectory_cases()),
        ("pass-through", pass_through_drone_trajectory_cases()),
        (
            "pass-through-accel",
            pass_through_accel_drone_trajectory_cases(),
        ),
        (
            "pass-through-accel-jerk",
            pass_through_accel_jerk_drone_trajectory_cases(),
        ),
    ];

    for (preset_name, cases) in presets {
        let (metrics, summaries) =
            evaluate_drone_controller_variants(&cases, &trajectory_variants, &controller_variants);
        println!();
        println!("preset: {preset_name}");
        println!(
            "controller variant   style                           mean rmse  mean maxerr  mean thrust  mean maxthrust  mean torque"
        );
        println!(
            "----------------------------------------------------------------------------------------------------------------------"
        );
        for summary in &summaries {
            println!(
                "{:<20} {:<31} {:>10.4} {:>12.4} {:>12.4} {:>15.4} {:>12.4}",
                summary.controller_variant.id(),
                summary.controller_variant.design_style(),
                summary.mean_tracking_rmse_m,
                summary.mean_max_position_error_m,
                summary.mean_thrust_n,
                summary.mean_max_thrust_n,
                summary.mean_torque_norm_nm,
            );
        }
        println!();
        println!(
            "family              trajectory                  controller          rmse m    max err   thrust   maxthrust   torque"
        );
        println!(
            "----------------------------------------------------------------------------------------------------------------"
        );
        for row in &metrics {
            println!(
                "{:<18} {:<27} {:<19} {:>8.4} {:>10.4} {:>8.4} {:>11.4} {:>8.4}",
                row.family_name,
                row.trajectory_variant.id(),
                row.controller_variant.id(),
                row.tracking_rmse_m,
                row.max_position_error_m,
                row.mean_thrust_n,
                row.max_thrust_n,
                row.mean_torque_norm_nm,
            );
        }

        let baseline = find_controller_summary(&summaries, DroneControllerVariant::BaselinePd);
        let damped =
            find_controller_summary(&summaries, DroneControllerVariant::AttitudeRateDampedPd);
        let lateral = find_controller_summary(
            &summaries,
            DroneControllerVariant::AttitudeRateDampedLateralPd,
        );

        assert!(
            lateral.mean_tracking_rmse_m < baseline.mean_tracking_rmse_m,
            "bounded lateral feedback should beat the baseline controller on preset `{preset_name}`"
        );
        assert!(
            lateral.mean_tracking_rmse_m < damped.mean_tracking_rmse_m,
            "bounded lateral feedback should beat the pure damped controller on preset `{preset_name}`"
        );
        assert!(
            lateral.mean_max_position_error_m < baseline.mean_max_position_error_m,
            "bounded lateral feedback should lower mean max error against the baseline on preset `{preset_name}`"
        );
        assert!(
            lateral.mean_torque_norm_nm < baseline.mean_torque_norm_nm,
            "bounded lateral feedback should keep torque demand below the baseline on preset `{preset_name}`"
        );
    }
}

#[test]
fn drone_controller_generator_pairings_report_tradeoffs() {
    let trajectory_variants = default_drone_trajectory_variants();
    let controller_variants = default_drone_controller_variants();
    let presets = vec![
        ("stop-go", default_drone_trajectory_cases()),
        ("pass-through", pass_through_drone_trajectory_cases()),
        (
            "pass-through-accel",
            pass_through_accel_drone_trajectory_cases(),
        ),
        (
            "pass-through-accel-jerk",
            pass_through_accel_jerk_drone_trajectory_cases(),
        ),
        (
            "coupled-continuity",
            coupled_continuity_drone_trajectory_cases(),
        ),
    ];

    for (preset_name, cases) in presets {
        let (metrics, _summaries) =
            evaluate_drone_controller_variants(&cases, &trajectory_variants, &controller_variants);

        println!();
        println!("preset pairing: {preset_name}");
        println!(
            "controller            trajectory                     mean us   mean rmse  mean maxerr  mean thrust  mean maxthrust  mean torque"
        );
        println!(
            "--------------------------------------------------------------------------------------------------------------------------------"
        );

        for controller_variant in controller_variants {
            for trajectory_variant in trajectory_variants {
                let summary = summarize_controller_trajectory(
                    &metrics,
                    trajectory_variant,
                    controller_variant,
                );
                println!(
                    "{:<21} {:<27} {:>8.2} {:>10.4} {:>12.4} {:>12.4} {:>15.4} {:>12.4}",
                    summary.controller_variant.id(),
                    summary.trajectory_variant.id(),
                    summary.mean_generation_us,
                    summary.mean_tracking_rmse_m,
                    summary.mean_max_position_error_m,
                    summary.mean_thrust_n,
                    summary.mean_max_thrust_n,
                    summary.mean_torque_norm_nm,
                );
            }
        }

        let lateral_quintic_uniform = summarize_controller_trajectory(
            &metrics,
            DroneTrajectoryVariant::QuinticUniform,
            DroneControllerVariant::AttitudeRateDampedLateralPd,
        );
        let lateral_quintic_distance = summarize_controller_trajectory(
            &metrics,
            DroneTrajectoryVariant::QuinticDistanceScaled,
            DroneControllerVariant::AttitudeRateDampedLateralPd,
        );
        let lateral_minimum_snap = summarize_controller_trajectory(
            &metrics,
            DroneTrajectoryVariant::MinimumSnapDistanceScaled,
            DroneControllerVariant::AttitudeRateDampedLateralPd,
        );

        assert!(
            lateral_quintic_distance.mean_generation_us < lateral_minimum_snap.mean_generation_us,
            "distance-scaled quintic should remain cheaper than minimum-snap under the bounded lateral-feedback controller on preset `{preset_name}`"
        );

        match preset_name {
            "coupled-continuity" => {
                assert!(
                    lateral_minimum_snap.mean_tracking_rmse_m
                        < lateral_quintic_distance.mean_tracking_rmse_m,
                    "coupled continuity should now make minimum-snap slightly better than distance-scaled quintic on mean RMSE under the bounded lateral-feedback controller"
                );
                assert!(
                    lateral_minimum_snap.mean_max_position_error_m
                        < lateral_quintic_distance.mean_max_position_error_m,
                    "coupled continuity should now make minimum-snap slightly better than distance-scaled quintic on mean max error under the bounded lateral-feedback controller"
                );
            }
            _ => {
                assert!(
                    lateral_quintic_distance.mean_tracking_rmse_m
                        < lateral_minimum_snap.mean_tracking_rmse_m,
                    "minimum-snap should still trail distance-scaled quintic on mean RMSE under the bounded lateral-feedback controller on preset `{preset_name}`"
                );
                assert!(
                    lateral_quintic_distance.mean_max_position_error_m
                        < lateral_minimum_snap.mean_max_position_error_m,
                    "minimum-snap should still trail distance-scaled quintic on mean max error under the bounded lateral-feedback controller on preset `{preset_name}`"
                );
            }
        }

        assert!(
            lateral_quintic_distance.mean_tracking_rmse_m
                <= lateral_quintic_uniform.mean_tracking_rmse_m,
            "distance-scaled quintic should remain at least as good as quintic-uniform on mean RMSE under the bounded lateral-feedback controller on preset `{preset_name}`"
        );
    }
}

#[test]
fn drone_lateral_gain_sensitivity_on_coupled_continuity() {
    let cases = coupled_continuity_drone_trajectory_cases();
    let trajectory_variants = [
        DroneTrajectoryVariant::QuinticDistanceScaled,
        DroneTrajectoryVariant::MinimumSnapDistanceScaled,
    ];
    let default_gains = LateralGainSet::default();

    // Sweep lateral_position_gain and lateral_velocity_gain around the default
    let position_gains = [0.08, 0.12, 0.15, 0.20, 0.25];
    let velocity_gains = [0.25, 0.35, 0.45, 0.55, 0.65];

    println!(
        "{:<28} {:<30} {:>8} {:>8} {:>10} {:>10} {:>10}",
        "family", "gain_label", "pos_g", "vel_g", "rmse_m", "max_err", "torque",
    );
    println!("{}", "-".repeat(110));

    let mut best_rmse_by_variant: std::collections::HashMap<&str, (f64, String)> =
        std::collections::HashMap::new();

    for tv in &trajectory_variants {
        for &pg in &position_gains {
            for &vg in &velocity_gains {
                let gains = LateralGainSet {
                    lateral_position_gain: pg,
                    lateral_velocity_gain: vg,
                    ..default_gains
                };
                let label = format!("{}-pg{:.2}-vg{:.2}", tv.id(), pg, vg);

                let mut total_rmse = 0.0;
                let mut total_max_err = 0.0;
                let mut total_torque = 0.0;
                let n = cases.len() as f64;

                for case in &cases {
                    let m = evaluate_lateral_gain_case(case, *tv, &gains, &label);
                    total_rmse += m.tracking_rmse_m;
                    total_max_err += m.max_position_error_m;
                    total_torque += m.mean_torque_norm_nm;
                }

                let mean_rmse = total_rmse / n;
                let mean_max_err = total_max_err / n;
                let mean_torque = total_torque / n;

                println!(
                    "{:<28} {:<30} {:>8.2} {:>8.2} {:>10.6} {:>10.6} {:>10.6}",
                    tv.id(),
                    label,
                    pg,
                    vg,
                    mean_rmse,
                    mean_max_err,
                    mean_torque,
                );

                let entry = best_rmse_by_variant
                    .entry(tv.id())
                    .or_insert((f64::INFINITY, String::new()));
                if mean_rmse < entry.0 {
                    *entry = (mean_rmse, label);
                }
            }
        }
    }

    println!();
    println!("best mean RMSE per trajectory variant:");
    for (variant_id, (rmse, label)) in &best_rmse_by_variant {
        println!("  {variant_id}: {rmse:.6} ({label})");
    }

    // Assert that the default gains are not dramatically worse than the best
    for tv in &trajectory_variants {
        let default_label = format!(
            "{}-pg{:.2}-vg{:.2}",
            tv.id(),
            default_gains.lateral_position_gain,
            default_gains.lateral_velocity_gain,
        );
        let mut default_rmse = 0.0;
        for case in &cases {
            let m = evaluate_lateral_gain_case(case, *tv, &default_gains, &default_label);
            default_rmse += m.tracking_rmse_m;
        }
        default_rmse /= cases.len() as f64;

        let (best_rmse, _) = &best_rmse_by_variant[tv.id()];
        assert!(
            default_rmse < best_rmse * 1.5,
            "default gains should be within 50% of the best RMSE for {}: default={default_rmse:.6}, best={best_rmse:.6}",
            tv.id(),
        );
    }
}

#[test]
fn drone_controller_generator_pairings_on_expanded_coupled_families() {
    let cases = coupled_continuity_drone_trajectory_cases();
    let trajectory_variants = default_drone_trajectory_variants();
    let controller_variants = default_drone_controller_variants();

    println!(
        "expanded coupled-continuity pairings ({} families, {} generators, {} controllers):",
        cases.len(),
        trajectory_variants.len(),
        controller_variants.len(),
    );

    let (metrics, _summaries) =
        evaluate_drone_controller_variants(&cases, &trajectory_variants, &controller_variants);

    println!(
        "{:<28} {:<28} {:<40} {:>10} {:>10} {:>10}",
        "family", "generator", "controller", "rmse_m", "max_err", "torque",
    );
    println!("{}", "-".repeat(130));

    for metric in &metrics {
        println!(
            "{:<28} {:<28} {:<40} {:>10.6} {:>10.6} {:>10.6}",
            metric.family_name,
            metric.trajectory_variant.id(),
            metric.controller_variant.id(),
            metric.tracking_rmse_m,
            metric.max_position_error_m,
            metric.mean_torque_norm_nm,
        );
    }

    // Check that bounded-lateral-feedback controller still wins on the
    // expanded coupled-continuity family set
    let lateral_metrics: Vec<_> = metrics
        .iter()
        .filter(|m| m.controller_variant == DroneControllerVariant::AttitudeRateDampedLateralPd)
        .collect();
    let baseline_metrics: Vec<_> = metrics
        .iter()
        .filter(|m| m.controller_variant == DroneControllerVariant::BaselinePd)
        .collect();

    let lateral_mean_rmse = lateral_metrics
        .iter()
        .map(|m| m.tracking_rmse_m)
        .sum::<f64>()
        / lateral_metrics.len() as f64;
    let baseline_mean_rmse = baseline_metrics
        .iter()
        .map(|m| m.tracking_rmse_m)
        .sum::<f64>()
        / baseline_metrics.len() as f64;

    println!();
    println!(
        "mean RMSE: baseline-pd={:.6}, lateral-pd={:.6}",
        baseline_mean_rmse, lateral_mean_rmse,
    );

    assert!(
        lateral_mean_rmse < baseline_mean_rmse,
        "bounded lateral-feedback controller should still beat baseline PD on expanded coupled families: lateral={lateral_mean_rmse:.6}, baseline={baseline_mean_rmse:.6}"
    );
}
