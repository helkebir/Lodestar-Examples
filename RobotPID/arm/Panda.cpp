//
// Created by advcc on 1/24/22.
//

#include "Panda.hpp"

StatusOr<ArmConnectionStatus> Panda::initConnection()
{
    try {
        robot_ = new franka::Robot(ipAddress_);
    } catch (franka::NetworkException &e) {
        return {StatusCode::NetworkError};
    }

    if (hasGripper) {
        try {
            gripper_ = new franka::Gripper(ipAddress_);
        } catch (franka::NetworkException &e) {
            return {StatusCode::NetworkError};
        }
    }

    return {{ipAddress_, robot_->serverVersion()}};
}

StatusOr<PandaState> Panda::getState()
{
    if (robot_ == nullptr || (gripper_ == nullptr && hasGripper))
        return {StatusCode::NotInitializedError};

    auto rs = getRobotState();

    if (!rs.ok())
        return {rs.status()};

    if (hasGripper) {
        auto gs = getGripperState();
        if (!gs.ok())
            return {gs.status()};

        latestState = PandaState{rs.value(), gs.value()};
    } else {
        latestState = PandaState{rs.value()};
    }

    return {latestState};
}

StatusOr<bool> Panda::gripperHoming()
{
    if (!hasGripper)
        return {StatusCode::HardwareNotPresentError};
    else if (gripper_ == nullptr)
        return {StatusCode::NotInitializedError};
    else {
        if (robot_ == nullptr)
            return {StatusCode::NotInitializedError};

        auto state = getState();

        if (!state.ok())
            return {state.status()};

        switch (state.value().robot.robotMode) {
            case franka::RobotMode::kReflex:
            case franka::RobotMode::kAutomaticErrorRecovery:
                return {StatusCode::SafetyViolationError};
            case franka::RobotMode::kIdle:
                try {
                    if (gripper_->homing())
                        return {{true}};
                    else
                        return {StatusCode::UnknownError};
                } catch (franka::NetworkException &e) {
                    return {StatusCode::NetworkError};
                } catch (franka::CommandException &e) {
                    return {StatusCode::CommandExecutionException};
                }
            case franka::RobotMode::kUserStopped:
                return {StatusCode::NotArmedError};
            default:
                return {StatusCode::ResourceBusyError};
        }
    }
}

StatusOr<PandaState::RobotState> Panda::getRobotState() const
{
    if (robot_ == nullptr)
        return {StatusCode::NotInitializedError};

    franka::RobotState rs;

    try {
        rs = robot_->readOnce();
    } catch (franka::NetworkException &e) {
        return {StatusCode::NetworkError};
    } catch (franka::InvalidOperationException &e) {
        return {StatusCode::ConcurrentProcessError};
    }

    return {PandaState::RobotState{rs}};
}

StatusOr<PandaState::GripperState> Panda::getGripperState() const
{
    if (!hasGripper)
        return {StatusCode::HardwareNotPresentError};

    if (gripper_ == nullptr)
        return {StatusCode::NotInitializedError};

    franka::GripperState gs;

    try {
        gs = gripper_->readOnce();
    } catch (franka::NetworkException &e) {
        return {StatusCode::NetworkError};
    } catch (franka::InvalidOperationException &e) {
        return {StatusCode::ConcurrentProcessError};
    }

    return {PandaState::GripperState{gs}};
}

StatusOr<bool> Panda::gripperGrasp(double width, double speed, double force, double epsInner, double epsOuter)
{
    if (!hasGripper)
        return {StatusCode::HardwareNotPresentError};
    else if (gripper_ == nullptr)
        return {StatusCode::NotInitializedError};
    else {
        if (robot_ == nullptr)
            return {StatusCode::NotInitializedError};

        auto state = getState();

        if (!state.ok())
            return {state.status()};

        switch (state.value().robot.robotMode) {
            case franka::RobotMode::kReflex:
            case franka::RobotMode::kAutomaticErrorRecovery:
                return {StatusCode::SafetyViolationError};
            case franka::RobotMode::kIdle:
                try {
                    if (gripper_->grasp(width, speed, force, epsInner, epsOuter))
                        return {{true}};
                    else
                        return {{false}};
                } catch (franka::NetworkException &e) {
                    return {StatusCode::NetworkError};
                } catch (franka::CommandException &e) {
                    return {StatusCode::CommandExecutionException};
                }
            case franka::RobotMode::kUserStopped:
                return {StatusCode::NotArmedError};
            default:
                return {StatusCode::ResourceBusyError};
        }
    }
}

void Panda::sleep(double ms) const
{
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(ms));
}

StatusOr<bool> Panda::gripperStop()
{
    if (!hasGripper)
        return {StatusCode::HardwareNotPresentError};
    else if (gripper_ == nullptr)
        return {StatusCode::NotInitializedError};
    else {
        if (robot_ == nullptr)
            return {StatusCode::NotInitializedError};

        auto state = getState();

        if (!state.ok())
            return {state.status()};

        switch (state.value().robot.robotMode) {
            case franka::RobotMode::kReflex:
            case franka::RobotMode::kAutomaticErrorRecovery:
                return {StatusCode::SafetyViolationError};
            case franka::RobotMode::kIdle:
                try {
                    if (gripper_->stop())
                        return {{true}};
                    else
                        return {{false}};
                } catch (franka::NetworkException &e) {
                    return {StatusCode::NetworkError};
                } catch (franka::CommandException &e) {
                    return {StatusCode::CommandExecutionException};
                }
            case franka::RobotMode::kUserStopped:
                return {StatusCode::NotArmedError};
            default:
                return {StatusCode::ResourceBusyError};
        }
    }
}

StatusOr<bool> Panda::gripperMove(double width, double speed)
{
    if (!hasGripper)
        return {StatusCode::HardwareNotPresentError};
    else if (gripper_ == nullptr)
        return {StatusCode::NotInitializedError};
    else {
        if (robot_ == nullptr)
            return {StatusCode::NotInitializedError};

        auto state = getState();

        if (!state.ok())
            return {state.status()};

        switch (state.value().robot.robotMode) {
            case franka::RobotMode::kReflex:
            case franka::RobotMode::kAutomaticErrorRecovery:
                return {StatusCode::SafetyViolationError};
            case franka::RobotMode::kIdle:
                try {
                    if (gripper_->move(width, speed))
                        return {{true}};
                    else
                        return {{false}};
                } catch (franka::NetworkException &e) {
                    return {StatusCode::NetworkError};
                } catch (franka::CommandException &e) {
                    return {StatusCode::CommandExecutionException};
                }
            case franka::RobotMode::kUserStopped:
                return {StatusCode::NotArmedError};
            default:
                return {StatusCode::ResourceBusyError};
        }
    }
}

StatusOr<bool> Panda::setNominalToEndEffectorTransformation(const Eigen::Matrix4d &mat)
{
    if (robot_ == nullptr)
        return {StatusCode::NotInitializedError};

    try {
        std::array<double, 16> arr{};
        std::copy_n(mat.data(), arr.size(), arr.begin());
        robot_->setEE(arr);
    } catch (franka::NetworkException &e) {
        return {StatusCode::NetworkError};
    } catch (franka::CommandException &e) {
        return {StatusCode::CommandExecutionException};
    }

    return {{true}};
}

StatusOr<bool> Panda::setNominalToEndEffectorTransformation(std::array<double, 16> arr)
{
    if (robot_ == nullptr)
        return {StatusCode::NotInitializedError};

    try {
        robot_->setEE(arr);
    } catch (franka::NetworkException &e) {
        return {StatusCode::NetworkError};
    } catch (franka::CommandException &e) {
        return {StatusCode::CommandExecutionException};
    }

    return {{true}};
}

StatusOr<bool>
Panda::setNominalToEndEffectorTransformation(const Eigen::AngleAxisd &axisAngle, const Eigen::Vector3d &translation)
{
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = axisAngle.toRotationMatrix();
    pose.block<3, 1>(0, 3) = translation;

    return setNominalToEndEffectorTransformation(pose);
}

StatusOr<bool> Panda::resetNominalToEndEffectorTransformation()
{
    return setNominalToEndEffectorTransformation(Eigen::Matrix4d::Identity());
}

StatusOr<bool>
Panda::setGuidingMode(bool x, bool y, bool z, bool R, bool P, bool Y, bool elbow)
{
    if (robot_ == nullptr)
        return {StatusCode::NotInitializedError};

    try {
        std::array<bool, 6> arr{x, y, z, R, P, Y};
        robot_->setGuidingMode(arr, elbow);
    } catch (franka::NetworkException &e) {
        return {StatusCode::NetworkError};
    } catch (franka::CommandException &e) {
        return {StatusCode::CommandExecutionException};
    }

    return {{true}};
}

StatusOr<bool> Panda::setGuidingMode(const Eigen::Matrix<bool, 6, 1> &axes, bool elbow)
{
    return setGuidingMode(axes[0], axes[1], axes[2], axes[3], axes[4], axes[5], elbow);
}

StatusOr<bool> Panda::setGuidingMode()
{
    return setGuidingMode(true, true, true, true, true, true, true);
}

StatusOr<bool> Panda::stop()
{
    if (robot_ == nullptr)
        return {StatusCode::NotInitializedError};

    try {
        robot_->stop();
    } catch (franka::NetworkException &e) {
        return {StatusCode::NetworkError};
    } catch (franka::CommandException &e) {
        return {StatusCode::CommandExecutionException};
    }

    return {{true}};
}

StatusOr<bool>
Panda::control(std::function<franka::CartesianVelocities(const franka::RobotState &, franka::Duration)> generator,
               franka::ControllerMode controllerMode, bool limitRate, double cutOffFrequency)
{
    if (robot_ == nullptr)
        return {StatusCode::NotInitializedError};

    auto state = getState();

    if (!state.ok())
        return {state.status()};

    switch (state.value().robot.robotMode) {
        case franka::RobotMode::kReflex:
        case franka::RobotMode::kAutomaticErrorRecovery:
            return {StatusCode::SafetyViolationError};
        case franka::RobotMode::kIdle:
            try {
                robot_->control(std::move(generator), controllerMode, limitRate, cutOffFrequency);
            } catch (franka::ControlException &e) {
                std::vector<franka::Record> records = e.log;
                ;
                return {StatusCode::ControllerError};
            } catch (franka::InvalidOperationException &) {
                return {StatusCode::ConcurrentProcessError};
            } catch (franka::NetworkException &) {
                return {StatusCode::NetworkError};
            } catch (franka::RealtimeException &) {
                return {StatusCode::RealtimeError};
            } catch (std::invalid_argument &) {
                return {StatusCode::InvalidInputError};
            }
        case franka::RobotMode::kUserStopped:
            return {StatusCode::NotArmedError};
        default:
            return {StatusCode::ResourceBusyError};
    }

    return {{true}};
}

StatusOr<bool>
Panda::control(std::function<franka::Torques(const franka::RobotState &, franka::Duration)> generator,
               bool limitRate, double cutOffFrequency)
{
    if (robot_ == nullptr)
        return {StatusCode::NotInitializedError};

    auto state = getState();

    if (!state.ok())
        return {state.status()};

    switch (state.value().robot.robotMode) {
        case franka::RobotMode::kReflex:
        case franka::RobotMode::kAutomaticErrorRecovery:
            return {StatusCode::SafetyViolationError};
        case franka::RobotMode::kIdle:
            try {
                robot_->control(std::move(generator), limitRate, cutOffFrequency);
            } catch (franka::ControlException &e) {
                std::vector<franka::Record> records = e.log;
                ;
                return {StatusCode::ControllerError};
            } catch (franka::InvalidOperationException &) {
                return {StatusCode::ConcurrentProcessError};
            } catch (franka::NetworkException &) {
                return {StatusCode::NetworkError};
            } catch (franka::RealtimeException &) {
                return {StatusCode::RealtimeError};
            } catch (std::invalid_argument &) {
                return {StatusCode::InvalidInputError};
            }
        case franka::RobotMode::kUserStopped:
            return {StatusCode::NotArmedError};
        default:
            return {StatusCode::ResourceBusyError};
    }

    return {{true}};
}

StatusOr<Eigen::Vector6d> Panda::getContactForce()
{
    if (robot_ == nullptr)
        return {StatusCode::NotInitializedError};
    try {
        auto model = robot_->loadModel();

        auto stateStatus = getState();

        if (!stateStatus.ok())
            return {stateStatus.status()};

        auto state = stateStatus.value();

        Eigen::Vector7d externalTorque;
        externalTorque << state.robot.jointTorque - initialGravity - initialExternalTorque;

        std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state.robot.robotState);
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

        return {jacobian * externalTorque};
    } catch (franka::ModelException &) {
        return {StatusCode::InternalError};
    } catch (franka::NetworkException &) {
        return {StatusCode::NetworkError};
    }
}

StatusOr<bool> Panda::initContactForce()
{
    auto stateStatus = getState();

    if (!stateStatus.ok())
        return {stateStatus.status()};

    auto state = stateStatus.value();

    if (robot_ == nullptr)
        return {StatusCode::NotInitializedError};
    try {
        auto model = robot_->loadModel();

        std::array<double, 7> gravity_array = model.gravity(state.robot.robotState);
        Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_tau_measured(state.robot.jointTorque.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_gravity(gravity_array.data());

        initialTorqueMeasured = initial_tau_measured;
        initialGravity = initial_gravity;
        initialExternalTorque = initialTorqueMeasured - initialGravity;

        return {{true}};
    } catch (franka::ModelException &) {
        return {StatusCode::InternalError};
    } catch (franka::NetworkException &) {
        return {StatusCode::NetworkError};
    }
}

StatusOr<Eigen::Vector6d> Panda::getContactForce(const PandaState::RobotState &robot)
{
    Eigen::Vector7d externalTorque;

    franka::Model model = getModel();

    externalTorque << robot.jointTorque - initialGravity - initialExternalTorque;

    std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot.robotState);
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

    return {jacobian * externalTorque};
}