use nalgebra as na;
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

    let urdf_robot = urdf_rs::read_file(urdf_path).unwrap();
    // let links = urdf_robot.links;
    let joints = urdf_robot.joints;
    for joint in joints.iter() {
        let node = Node::from_pos_rpy(*joint.origin.xyz, *joint.origin.rpy);
        println!("{:?}", node);
    }

    // let rec = rerun::RecordingStreamBuilder::new("urdf_test").connect_grpc()?;
    // rec.log_file_from_path(urdf_path, None, true)?;
    //
    // rec.set_time_sequence("step", 1);
    //
    // let angle = 0.0;
    // for (joint_index, joint) in urdf.joints().enumerate() {
    //     if joint.joint_type == urdf_rs::JointType::Revolute {
    //         let fixed_axis = joint.axis.xyz.0;
    //
    //         // // Usually this angle would come from a measurement - here we just fake something:
    //         // let dynamic_angle = emath::remap(
    //         //     (step as f64 * (0.02 + joint_index as f64 / 100.0)).sin(),
    //         //     -1.0..=1.0,
    //         //     joint.limit.lower..=joint.limit.upper,
    //         // );
    //
    //         // NOTE: each joint already has a fixed origin pose (logged with the URDF file),
    //         // and Rerun won't allow us to override or add to that transform here.
    //         // So instead we apply the dynamic rotation to the child link of the joint:
    //         let child_link = urdf.get_joint_child(joint);
    //         let link_path = urdf.get_link_path(child_link);
    //         rec.log(
    //             link_path,
    //             &rerun::Transform3D::from_rotation(rerun::RotationAxisAngle::new(
    //                 fixed_axis,
    //                 angle as f32,
    //             )),
    //         )?;
    //     }
    // }
    //
    Ok(())
}
