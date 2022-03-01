//
// Created by advcc on 1/24/22.
//

#include "PandaState.hpp"

PandaState::RobotState::RobotState(const franka::RobotState &rs)
{
    parse(rs);
}

void PandaState::RobotState::parse(const franka::RobotState &rs)
{
    robotState = rs;
    robotMode = rs.robot_mode;

    duration = rs.time;

    currentErrors = rs.current_errors;
    lastMotionErrors = rs.last_motion_errors;

    endEffectorMass = rs.m_ee;
    loadMass = rs.m_load;
    totalPayloadMass = rs.m_total;

    elbow = rs.elbow[0]*rs.elbow[1];
    elbowDesired = rs.elbow_d[0]*rs.elbow_d[1];
    elbowCommanded = rs.elbow_c[0]*rs.elbow_c[1];
    elbowVelocityCommanded = rs.delbow_c[0]*rs.delbow_c[1];
    elbowAccelerationCommanded = rs.ddelbow_c[0]*rs.ddelbow_c[1];

    motorPosition = Eigen::Vector7d(rs.theta.data());
    motorVelocity = Eigen::Vector7d(rs.dtheta.data());

    jointTorque = Eigen::Vector7d(rs.tau_J.data());
    jointTorqueRate = Eigen::Vector7d(rs.dtau_J.data());
    jointTorqueDesired = Eigen::Vector7d(rs.tau_J_d.data());

    jointPosition = Eigen::Vector7d(rs.q.data());
    jointVelocity = Eigen::Vector7d(rs.dq.data());
    jointVelocityDesired = Eigen::Vector7d(rs.dq_d.data());
    jointAccelerationDesired = Eigen::Vector7d(rs.ddq_d.data());

    jointContact = Eigen::Vector7d(rs.joint_contact.data());
    jointCollision = Eigen::Vector7d(rs.joint_collision.data());
    cartesianContact = Eigen::Vector6d(rs.cartesian_collision.data());

    externalTorque = Eigen::Vector7d(rs.tau_ext_hat_filtered.data());

    externalWrenchStiffness_Base = Eigen::Vector6d(rs.O_F_ext_hat_K.data());
    externalWrenchStiffness_Stiffness = Eigen::Vector6d(rs.K_F_ext_hat_K.data());

    endEffectorTwist_Base_Desired = Eigen::Vector6d(rs.O_dP_EE_d.data());
    endEffectorTwist_Base_Commanded = Eigen::Vector6d(rs.O_dP_EE_c.data());
    endEffectorTwistRate_Base_Commanded = Eigen::Vector6d(rs.O_ddP_EE_c.data());




    endEffectorInertia = Eigen::Matrix3d(rs.I_ee.data());
    loadInertia = Eigen::Matrix3d(rs.I_load.data());
    totalPayloadInertia = Eigen::Matrix3d(rs.I_total.data());

    endEffectorCoM_Flange = Eigen::Vector3d(rs.F_x_Cee.data());
    loadCoM_Flange = Eigen::Vector3d(rs.F_x_Cload.data());
    totalPayloadCoM_Flange = Eigen::Vector3d(rs.F_x_Ctotal.data());

    endEffectorPose_Base = Eigen::Matrix4d(rs.O_T_EE.data());
    endEffectorPose_Base_Desired = Eigen::Matrix4d(rs.O_T_EE_d.data());
    endEffectorPose_Base_Commanded = Eigen::Matrix4d(rs.O_T_EE_c.data());

    endEffectorPose_Flange = Eigen::Matrix4d(rs.F_T_EE.data());
    nominalEndEffectorPose_Flange = Eigen::Matrix4d(rs.F_T_NE.data());

    endEffectorPose_NominalEndEffector = Eigen::Matrix4d(rs.NE_T_EE.data());
    stiffnessPose_EndEffector = Eigen::Matrix4d(rs.EE_T_K.data());
}

PandaState::GripperState::GripperState(const franka::GripperState &gs)
{
    parse(gs);
}

void PandaState::GripperState::parse(const franka::GripperState &gs)
{
    gripperState = gs;

    maxWidth = gs.max_width;
    width = gs.width;

    temperature = gs.temperature;

    isGrasped = gs.is_grasped;
}
