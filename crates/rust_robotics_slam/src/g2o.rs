//! Minimal g2o SE(2)/SE(3) pose-graph text I/O.

use core::fmt;
use std::collections::BTreeMap;

use nalgebra::{Matrix3, Matrix4, Quaternion, UnitQuaternion, Vector3};
use rust_robotics_core::Matrix6;

use crate::pose_graph_optimization::{Edge2D, Pose2DNode};
use crate::pose_graph_optimization_3d::{Edge3D, Pose3DNode};

/// A supported g2o pose graph.
#[derive(Debug, Clone)]
pub enum G2oGraph {
    Se2 {
        vertices: BTreeMap<usize, Pose2DNode>,
        edges: Vec<Edge2D>,
    },
    Se3 {
        vertices: BTreeMap<usize, Pose3DNode>,
        edges: Vec<Edge3D>,
    },
}

/// Error returned for malformed or mixed-dimension g2o input.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct G2oError {
    pub line: usize,
    pub message: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Dimension {
    Se2,
    Se3,
}

impl fmt::Display for G2oError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "g2o line {}: {}", self.line, self.message)
    }
}

impl std::error::Error for G2oError {}

/// Parses `VERTEX_SE2`, `EDGE_SE2`, `VERTEX_SE3:QUAT`, and
/// `EDGE_SE3:QUAT` records.
pub fn parse_g2o(input: &str) -> Result<G2oGraph, G2oError> {
    let mut dimension = None;
    let mut vertices_2d = BTreeMap::new();
    let mut edges_2d = Vec::new();
    let mut vertices_3d = BTreeMap::new();
    let mut edges_3d = Vec::new();

    for (line_index, raw_line) in input.lines().enumerate() {
        let line_number = line_index + 1;
        let line = raw_line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let fields = line.split_whitespace().collect::<Vec<_>>();
        match fields[0] {
            "VERTEX_SE2" => {
                require_dimension(&mut dimension, Dimension::Se2, line_number)?;
                require_len(&fields, 5, line_number)?;
                let id = parse_usize(fields[1], line_number)?;
                vertices_2d.insert(
                    id,
                    Pose2DNode::new(
                        parse_f64(fields[2], line_number)?,
                        parse_f64(fields[3], line_number)?,
                        parse_f64(fields[4], line_number)?,
                    ),
                );
            }
            "EDGE_SE2" => {
                require_dimension(&mut dimension, Dimension::Se2, line_number)?;
                require_len(&fields, 12, line_number)?;
                let mut information = Matrix3::zeros();
                fill_upper_triangle(&mut information, &fields[6..], line_number)?;
                edges_2d.push(Edge2D {
                    from: parse_usize(fields[1], line_number)?,
                    to: parse_usize(fields[2], line_number)?,
                    measurement: Pose2DNode::new(
                        parse_f64(fields[3], line_number)?,
                        parse_f64(fields[4], line_number)?,
                        parse_f64(fields[5], line_number)?,
                    ),
                    information,
                });
            }
            "VERTEX_SE3:QUAT" => {
                require_dimension(&mut dimension, Dimension::Se3, line_number)?;
                require_len(&fields, 9, line_number)?;
                let id = parse_usize(fields[1], line_number)?;
                vertices_3d.insert(id, pose_3d_from_fields(&fields[2..], line_number)?);
            }
            "EDGE_SE3:QUAT" => {
                require_dimension(&mut dimension, Dimension::Se3, line_number)?;
                require_len(&fields, 31, line_number)?;
                let mut information = Matrix6::zeros();
                fill_upper_triangle(&mut information, &fields[10..], line_number)?;
                edges_3d.push(Edge3D {
                    from: parse_usize(fields[1], line_number)?,
                    to: parse_usize(fields[2], line_number)?,
                    measurement: pose_3d_from_fields(&fields[3..10], line_number)?,
                    information,
                });
            }
            tag => {
                return Err(G2oError {
                    line: line_number,
                    message: format!("unsupported record {tag}"),
                });
            }
        }
    }

    match dimension {
        Some(Dimension::Se2) => Ok(G2oGraph::Se2 {
            vertices: vertices_2d,
            edges: edges_2d,
        }),
        Some(Dimension::Se3) => Ok(G2oGraph::Se3 {
            vertices: vertices_3d,
            edges: edges_3d,
        }),
        None => Err(G2oError {
            line: 0,
            message: "input contains no supported records".into(),
        }),
    }
}

/// Serializes a supported graph using standard upper-triangular information
/// matrix ordering.
pub fn write_g2o(graph: &G2oGraph) -> String {
    let mut output = String::new();
    match graph {
        G2oGraph::Se2 { vertices, edges } => {
            for (id, pose) in vertices {
                output.push_str(&format!(
                    "VERTEX_SE2 {id} {} {} {}\n",
                    pose.x, pose.y, pose.yaw
                ));
            }
            for edge in edges {
                output.push_str(&format!(
                    "EDGE_SE2 {} {} {} {} {}",
                    edge.from,
                    edge.to,
                    edge.measurement.x,
                    edge.measurement.y,
                    edge.measurement.yaw
                ));
                append_upper_triangle(&mut output, &edge.information);
                output.push('\n');
            }
        }
        G2oGraph::Se3 { vertices, edges } => {
            for (id, pose) in vertices {
                append_pose_3d(&mut output, "VERTEX_SE3:QUAT", &id.to_string(), pose);
            }
            for edge in edges {
                append_pose_3d(
                    &mut output,
                    "EDGE_SE3:QUAT",
                    &format!("{} {}", edge.from, edge.to),
                    &edge.measurement,
                );
                output.pop();
                append_upper_triangle(&mut output, &edge.information);
                output.push('\n');
            }
        }
    }
    output
}

fn pose_3d_from_fields(fields: &[&str], line: usize) -> Result<Pose3DNode, G2oError> {
    let translation = Vector3::new(
        parse_f64(fields[0], line)?,
        parse_f64(fields[1], line)?,
        parse_f64(fields[2], line)?,
    );
    let quaternion = UnitQuaternion::new_normalize(Quaternion::new(
        parse_f64(fields[6], line)?,
        parse_f64(fields[3], line)?,
        parse_f64(fields[4], line)?,
        parse_f64(fields[5], line)?,
    ));
    let mut transform = Matrix4::identity();
    transform
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(quaternion.to_rotation_matrix().matrix());
    transform
        .fixed_view_mut::<3, 1>(0, 3)
        .copy_from(&translation);
    Ok(Pose3DNode { transform })
}

fn append_pose_3d(output: &mut String, tag: &str, ids: &str, pose: &Pose3DNode) {
    let translation = pose.transform.fixed_view::<3, 1>(0, 3);
    let rotation =
        UnitQuaternion::from_matrix(&pose.transform.fixed_view::<3, 3>(0, 0).into_owned());
    let quaternion = rotation.quaternion();
    output.push_str(&format!(
        "{tag} {ids} {} {} {} {} {} {} {}\n",
        translation.x,
        translation.y,
        translation.z,
        quaternion.i,
        quaternion.j,
        quaternion.k,
        quaternion.w
    ));
}

fn append_upper_triangle<const N: usize>(
    output: &mut String,
    matrix: &nalgebra::SMatrix<f64, N, N>,
) {
    for row in 0..N {
        for column in row..N {
            output.push_str(&format!(" {}", matrix[(row, column)]));
        }
    }
}

fn fill_upper_triangle<const N: usize>(
    matrix: &mut nalgebra::SMatrix<f64, N, N>,
    fields: &[&str],
    line: usize,
) -> Result<(), G2oError> {
    let mut index = 0;
    for row in 0..N {
        for column in row..N {
            let value = parse_f64(fields[index], line)?;
            matrix[(row, column)] = value;
            matrix[(column, row)] = value;
            index += 1;
        }
    }
    Ok(())
}

fn require_dimension(
    current: &mut Option<Dimension>,
    requested: Dimension,
    line: usize,
) -> Result<(), G2oError> {
    if current.is_some_and(|value| value != requested) {
        return Err(G2oError {
            line,
            message: "mixed SE(2) and SE(3) records are not supported".into(),
        });
    }
    *current = Some(requested);
    Ok(())
}

fn require_len(fields: &[&str], expected: usize, line: usize) -> Result<(), G2oError> {
    if fields.len() == expected {
        Ok(())
    } else {
        Err(G2oError {
            line,
            message: format!("expected {expected} fields, found {}", fields.len()),
        })
    }
}

fn parse_f64(field: &str, line: usize) -> Result<f64, G2oError> {
    field.parse().map_err(|_| G2oError {
        line,
        message: format!("invalid number {field}"),
    })
}

fn parse_usize(field: &str, line: usize) -> Result<usize, G2oError> {
    field.parse().map_err(|_| G2oError {
        line,
        message: format!("invalid vertex id {field}"),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn se2_round_trip_preserves_graph() {
        let input = "\
VERTEX_SE2 0 0 0 0
VERTEX_SE2 1 1 0 0
EDGE_SE2 0 1 1 0 0 10 0 0 10 0 20
";
        let graph = parse_g2o(input).unwrap();
        let encoded = write_g2o(&graph);
        let decoded = parse_g2o(&encoded).unwrap();
        match decoded {
            G2oGraph::Se2 { vertices, edges } => {
                assert_eq!(vertices.len(), 2);
                assert_eq!(edges.len(), 1);
                assert_eq!(edges[0].information[(2, 2)], 20.0);
            }
            _ => panic!("expected SE(2) graph"),
        }
    }

    #[test]
    fn se3_quaternion_round_trip_preserves_pose() {
        let input = "\
VERTEX_SE3:QUAT 0 1 2 3 0 0 0 1
VERTEX_SE3:QUAT 1 2 2 3 0 0 0 1
EDGE_SE3:QUAT 0 1 1 0 0 0 0 0 1 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
";
        let graph = parse_g2o(input).unwrap();
        let decoded = parse_g2o(&write_g2o(&graph)).unwrap();
        match decoded {
            G2oGraph::Se3 { vertices, edges } => {
                assert_eq!(vertices.len(), 2);
                assert_eq!(edges.len(), 1);
                assert!((vertices[&0].transform[(2, 3)] - 3.0).abs() < 1.0e-12);
            }
            _ => panic!("expected SE(3) graph"),
        }
    }
}
