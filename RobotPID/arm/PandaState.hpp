//
// Created by advcc on 1/24/22.
//

#ifndef KNIFEFISH_PANDASTATE_HPP
#define KNIFEFISH_PANDASTATE_HPP

#include <Eigen/Dense>
#include <utility>
#include <franka/robot.h>
#include <franka/gripper.h>

namespace Eigen {
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef Eigen::Matrix<double, 7, 1> Vector7d;
}

class PandaState {
public:
    PandaState() : robot{}, gripper{} {}

    explicit PandaState(const franka::RobotState &rs) : robot(rs), gripper{}
    {}

    PandaState(const franka::RobotState &rs, const franka::GripperState &gs) :
            robot(rs), gripper(gs)
    {}

    class GripperState {
    public:
        GripperState() = default;
        explicit GripperState(const franka::GripperState &gs);

        void parse(const franka::GripperState &gs);

        franka::GripperState gripperState; /// Raw gripper state.

        double maxWidth{}; /// Maximum gripper width (determined after homing).
        double width{}; /// Current gripper width.

        std::uint16_t temperature{}; /// Gripper temperature in degrees Celsius.

        bool isGrasped{}; /// True if gripper is grasping.
    };

    class RobotState {
    public:
        RobotState() = default;
        explicit RobotState(const franka::RobotState &rs);

        void parse(const franka::RobotState &rs);

        franka::RobotState robotState; /// Raw robot state.
        franka::RobotMode robotMode = franka::RobotMode::kOther; /// Robot mode.

        franka::Duration duration; /// Monotonically increasing timestamp.

        franka::Errors currentErrors; /// Current errors.
        franka::Errors lastMotionErrors; /// Errors encountered in the last motion.

        double endEffectorMass{}; /// Configured mass of end effector.
        double loadMass{}; /// Configured mass of external load.
        double totalPayloadMass{}; /// Mass of end effector plus external load.

        double elbow{}; /// Measured signed elbow configuration.
        double elbowDesired{}; /// Desired signed elbow configuration.
        double elbowCommanded{}; /// Commanded signed elbow configuration.
        double elbowVelocityCommanded{}; /// Commanded signed elbow velocity.
        double elbowAccelerationCommanded{}; /// Commanded signed elbow acceleration.

        Eigen::Vector7d motorPosition; /// Joint motor position.
        Eigen::Vector7d motorVelocity; /// Joint motor velocity.

        Eigen::Vector7d jointTorque; /// Measured joint torques.
        Eigen::Vector7d jointTorqueRate; /// Measured joint torque rates.
        Eigen::Vector7d jointTorqueDesired; /// Desired joint torques without gravity.

        Eigen::Vector7d jointPosition; /// Measured joint positions.
        Eigen::Vector7d jointVelocity; /// Measured joint velocities.
        Eigen::Vector7d jointPositionDesired; /// Desired joint positions.
        Eigen::Vector7d jointVelocityDesired; /// Desired joint velocities.
        Eigen::Vector7d jointAccelerationDesired; /// Desired joint accelerations.

        Eigen::Vector7d jointContact; /// Measured contact levels at each joint.
        Eigen::Vector7d jointCollision; /// Measured contact levels (with collision settings applied) at each joint.
        Eigen::Vector6d cartesianContact; /// Measured contact levels in Cartesian coordinates.

        Eigen::Vector7d externalTorque; /// Measured external torque after filtering.

        Eigen::Vector6d externalWrenchStiffness_Base; /// Filtered external wrench (force, torque) applied to the stiffness frame, expressed in the base frame.
        Eigen::Vector6d externalWrenchStiffness_Stiffness; /// Filtered external wrench (force, torque) applied to the stiffness frame, expressed in the stiffness frame.

        Eigen::Vector6d endEffectorTwist_Base_Desired; /// Desired end effector twist (Cartesian linear and angular velocity) in the base frame.
        Eigen::Vector6d endEffectorTwist_Base_Commanded; /// Commanded end effector twist (Cartesian linear and angular velocity) in the base frame.
        Eigen::Vector6d endEffectorTwistRate_Base_Commanded; /// Commanded end effector twist rate (Cartesian linear and angular acceleration) in the base frame.

        Eigen::Matrix3d endEffectorInertia; /// Rotational inertia of end effector with respect to center of mass.
        Eigen::Matrix3d loadInertia; /// Rotational inertia of external load with respect to center of mass.
        Eigen::Matrix3d totalPayloadInertia; /// Combined rotational inertia of end effector and external load with respect to center of mass.

        Eigen::Vector3d endEffectorCoM_Flange; /// Configured center of mass of the end effector in the flange frame.
        Eigen::Vector3d loadCoM_Flange; /// Configured center of mass of the external load in the flange frame.
        Eigen::Vector3d totalPayloadCoM_Flange; /// Combined center of mass of the end effector and external load in the flange frame.

        Eigen::Matrix4d endEffectorPose_Base; /// Measured end effector pose in the base frame.
        Eigen::Matrix4d endEffectorPose_Base_Desired; /// Last desired end effector pose in the base frame.
        Eigen::Matrix4d endEffectorPose_Base_Commanded; /// Last commanded end effector pose in the base frame.

        Eigen::Matrix4d endEffectorPose_Flange; /// Measured end effector pose in the flange frame.
        Eigen::Matrix4d nominalEndEffectorPose_Flange; /// Measured nominal end effector pose in the flange frame.
        Eigen::Matrix4d endEffectorPose_NominalEndEffector; /// Measured end effector pose in the nominal end effector frame.

        Eigen::Matrix4d stiffnessPose_EndEffector; /// Measured stiffness frame pose in the end effector frame.
    };

    RobotState robot;
    GripperState gripper;

    PandaState(RobotState rs, const GripperState &gs) : robot(std::move(rs)), gripper(gs)
    {}

    explicit PandaState(RobotState rs) : robot(std::move(rs))
    {}
};

#endif //KNIFEFISH_PANDASTATE_HPP
