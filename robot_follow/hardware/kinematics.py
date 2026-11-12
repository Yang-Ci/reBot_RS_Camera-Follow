"""Pinocchio-specific TCP pose and Jacobian calculation."""
from robot_follow.control.kinematics import skew


def tcp_pose_and_jacobian(robot, joint_positions, tool_offset):
    """Compute TCP pose and LOCAL_WORLD_ALIGNED Jacobian for an adapter."""
    import pinocchio as pin

    q_full = robot._build_q_full(joint_positions)
    pin.forwardKinematics(robot.model, robot.data, q_full)
    pin.updateFramePlacements(robot.model, robot.data)
    pin.computeJointJacobians(robot.model, robot.data, q_full)

    last_jid = robot.model.getJointId(robot.joint_names[-1])
    placement = robot.data.oMi[last_jid]
    offset_world = placement.rotation @ tool_offset
    tcp_position = placement.translation + offset_world
    tcp_rotation = placement.rotation.copy()
    jacobian = pin.getJointJacobian(robot.model, robot.data, last_jid,
                                    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    tcp_jacobian = jacobian.copy()
    tcp_jacobian[:3, :] -= skew(offset_world) @ jacobian[3:, :]
    columns = [robot.model.joints[robot.model.getJointId(name)].idx_v
               for name in robot.joint_names]
    return tcp_position.copy(), tcp_rotation, tcp_jacobian[:, columns]
