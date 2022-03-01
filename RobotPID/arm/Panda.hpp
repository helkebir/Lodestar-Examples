//
// Created by advcc on 1/24/22.
//

#ifndef KNIFEFISH_PANDA_HPP
#define KNIFEFISH_PANDA_HPP

#define KF_CONTROL_IP "172.16.0.2"

#include "PandaState.hpp"
#include "ArmConnectionStatus.hpp"

#include <string>
#include <iostream>
#include <utility>
#include <thread>

#include <Lodestar/core/StatusOr.hpp>

#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/exception.h>
#include <franka/model.h>

using namespace ls::core;

class Panda {
public:
    Panda() : ipAddress_(KF_CONTROL_IP), hasGripper(true)
    {}

    Panda(bool hasGripper_) : ipAddress_(KF_CONTROL_IP), hasGripper(hasGripper_)
    {}

    Panda(std::string ipAddress, bool hasGripper_ = true) : ipAddress_(std::move(ipAddress)), hasGripper(hasGripper_)
    {}

    StatusOr<ArmConnectionStatus> initConnection();

    StatusOr<PandaState> getState();

    StatusOr<PandaState::RobotState> getRobotState() const;

    StatusOr<PandaState::GripperState> getGripperState() const;

    // Gripper
    bool hasGripper = false;

    /**
     * Homes the gripper to estimate maximum grip width.
     *
     * @return True if successful, status message if an exception is encountered.
     */
    StatusOr<bool> gripperHoming();

    /**
     * Grasps object with specified action parameters.
     *
     * An object is considered grasped if the distance between gripper fingers satisfies
     *   (width - epsInner) < (width + epsOuter)
     *
     * @param width Object width to be grasped (m)
     * @param speed Speed of pinching motion (m/s)
     * @param force Allowable grasping force (N)
     * @param epsInner Inner tolerance (m)
     * @param epsOuter Outer tolerance (m)
     *
     * @return True if an object is grasped, false if not. Returns status message if an exception is encountered.
     */
    StatusOr<bool>
    gripperGrasp(double width, double speed, double force, double epsInner = 5e-3, double epsOuter = 5e-3);

    /**
     * Stops the current gripper action (grasping or moving).
     *
     * @return Bool reflecting action success, or status with error message.
     */
    StatusOr<bool> gripperStop();

    /**
     * Moves the gripper to a specified width.
     *
     * @param width Desired width between gripper fingers (m).
     * @param speed Desired movement speed (m/s).
     *
     * @return True if movement was successful, false if not, and a status message if an exception is encountered.
     */
    StatusOr<bool> gripperMove(double width, double speed);

    // Arm
    StatusOr<bool> resetNominalToEndEffectorTransformation();

    StatusOr<bool> setNominalToEndEffectorTransformation(const Eigen::Matrix4d &mat);

    StatusOr<bool> setNominalToEndEffectorTransformation(std::array<double, 16> arr);

    StatusOr<bool>
    setNominalToEndEffectorTransformation(const Eigen::AngleAxisd &axisAngle, const Eigen::Vector3d &translation);

    /**
     * Sets guiding mode to allow movement in all degrees of freedom.
     *
     * @return True if successful, status message otherwise.
     */
    StatusOr<bool> setGuidingMode();

    /**
     * Sets guiding mode to allow movement in specified degrees of freedom.
     *
     * @param x Flag for translation in x-direction.
     * @param y Flag for translation in y-direction.
     * @param z Flag for translation in z-direction.
     * @param R Flag for rotation in roll axis.
     * @param P Flag for rotation in pitch axis.
     * @param Y Flag for rotation in yaw axis.
     * @param elbow Flag for elbow movement.
     *
     * @return True if successful, status message otherwise.
     */
    StatusOr<bool>
    setGuidingMode(bool x, bool y, bool z, bool R, bool P, bool Y, bool elbow);

    /**
     * Sets guiding mode to allow movement in specified degrees of freedom.
     *
     * @param axes Vector of boolean flags for movement in each degree of freedom.
     * @param elbow Flag for elbow movement.
     *
     * @return True if successful, status message otherwise.
     */
    StatusOr<bool> setGuidingMode(const Eigen::Matrix<bool, 6, 1> &axes, bool elbow);

    /**
     * Stops all robot movement.
     *
     * @return True if successfully stopped, status message otherwise.
     */
    StatusOr<bool> stop();

    StatusOr<bool>
    control(std::function<franka::CartesianVelocities(const franka::RobotState &, franka::Duration)> generator,
            franka::ControllerMode controllerMode = franka::ControllerMode::kJointImpedance, bool limitRate = true,
            double cutOffFrequency = franka::kMaxCutoffFrequency);

    StatusOr<bool>
    control(std::function<franka::Torques(const franka::RobotState &, franka::Duration)> generator,
            bool limitRate = true,
            double cutOffFrequency = franka::kMaxCutoffFrequency);

    StatusOr<bool> initContactForce();
    StatusOr<Eigen::Vector6d> getContactForce();
    StatusOr<Eigen::Vector6d> getContactForce(const PandaState::RobotState &robot);

    // Miscellaneous
    /**
     * Pause execution in the current thread for `ms` milliseconds.
     *
     * @param ms Sleep time in milliseconds.
     */
    void sleep(double ms) const;

    franka::Model getModel()
    {
        return robot_->loadModel();
    }

    PandaState latestState;

    Eigen::Vector7d initialTorqueMeasured, initialGravity, initialExternalTorque;
//protected:
    franka::Robot *robot_{};
    franka::Gripper *gripper_{};

    std::string ipAddress_;
};

#endif //KNIFEFISH_PANDA_HPP
