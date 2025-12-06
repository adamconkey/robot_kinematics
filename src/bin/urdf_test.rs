use nalgebra as na;
use rerun::external::re_data_loader::UrdfTree;
use rerun::external::urdf_rs;
use std::path::PathBuf;

#[derive(Debug)]
struct Node {
    pose: na::Isometry3<f64>,
}

impl Node {
    // TODO need to figure out type, should be any f64 you can index with 3
    fn from_pos_rpy(pos: [f64; 3], rpy: [f64; 3]) -> Self {
        let trans = na::Translation3::new(pos[0], pos[1], pos[2]);
        let rot = na::UnitQuaternion::from_euler_angles(rpy[0], rpy[1], rpy[2]);
        let pose = na::Isometry3::from_parts(trans, rot);
        Node { pose }
    }
}

fn main() -> anyhow::Result<()> {
    let mut urdf_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    urdf_path.push("robot-assets");
    urdf_path.push("urdfs");
    urdf_path.push("robots");
    urdf_path.push("kuka_iiwa");
    urdf_path.push("model.urdf");

    let urdf_robot = urdf_rs::read_file(urdf_path.clone()).unwrap();
    let mut pose = na::Isometry::identity();
    for joint in urdf_robot.joints.iter() {
        let node = Node::from_pos_rpy(*joint.origin.xyz, *joint.origin.rpy);
        pose *= node.pose;
    }

    let urdf = UrdfTree::from_file_path(urdf_path.clone())?;
    let rec = rerun::RecordingStreamBuilder::new("urdf_test").connect_grpc()?;
    rec.log_file_from_path(urdf_path, None, true)?;

    rec.set_time_sequence("step", 1);

    rec.log(
        "pose",
        &rerun::Transform3D::from_translation_rotation(
            [pose.translation.x, pose.translation.y, pose.translation.z],
            rerun::Quaternion::from_xyzw([
                pose.rotation.i as f32,
                pose.rotation.j as f32,
                pose.rotation.k as f32,
                pose.rotation.w as f32,
            ]),
        )
        .with_axis_length(0.2),
    )?;

    let angle = 0.0;
    for joint in urdf.joints().into_iter() {
        if joint.joint_type == urdf_rs::JointType::Revolute {
            // NOTE: each joint already has a fixed origin pose (logged with the URDF file),
            // and Rerun won't allow us to override or add to that transform here.
            // So instead we apply the dynamic rotation to the child link of the joint:
            let child_link = urdf.get_joint_child(joint);
            let link_path = urdf.get_link_path(child_link);
            rec.log(
                link_path,
                &rerun::Transform3D::from_rotation(rerun::RotationAxisAngle::new(
                    joint.axis.xyz.0,
                    angle as f32,
                )),
            )?;
        }
    }

    Ok(())
}
