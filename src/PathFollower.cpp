#include "PathFollower.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace
{
/*
    Simple proportional gains.

    The controller converts pose error into velocity:
        linear velocity  = kPositionGain * position error
        angular velocity = kRotationGain * angle error
*/
constexpr double kPositionGain = 2.5;
constexpr double kRotationGain = 3.0;

// Limit for angular velocity in rad/s.
constexpr double kMaxRotationSpeed = 3.0;

/*
    The goal is considered reached when both position and angle errors
    are below these tolerances.
*/
constexpr double kGoalPositionTolerance = 1e-3;
constexpr double kGoalAngleTolerance = 1e-3;

/*
    Instead of tracking the closest path pose directly, track a pose a few
    indices ahead. This gives smoother behavior on curves.
*/
constexpr std::size_t kLookaheadSteps = 5;
} // namespace

PathFollower::PathFollower(const Path& path,
                           double maxWheelSpeed,
                           const Eigen::Vector2d& parcelSize)
    : path_(path),
      maxWheelSpeed_(std::max(0.0, maxWheelSpeed)),
      halfParcelSize_(0.5 * parcelSize),
      currentPose_(path.poses.empty() ? Eigen::Isometry2d::Identity()
                                      : path.poses.front())
{
}

void PathFollower::update(const Eigen::Isometry2d& curPose,
                          std::chrono::time_point<std::chrono::system_clock>& updateTime)
{
    /*
        Store the latest measured pose. computeNextCmd() uses this pose
        to calculate the next velocity command.
    */
    currentPose_ = curPose;
    hasCurrentPose_ = true;
    lastUpdateTime_ = updateTime;

    /*
        Update progress along the path by finding the closest pose.
        The actual target will be selected ahead of this closest pose.
    */
    if (!path_.poses.empty())
    {
        closestPoseIndex_ = findClosestPoseIndex();
    }
}

Twist2d PathFollower::computeNextCmd(std::chrono::time_point<std::chrono::system_clock>& curTime)
{
    /*
        This controller returns velocities. The time step is applied by
        the caller/simulator when integrating the pose.
    */
    (void)curTime;

    /*
        If we do not have enough information, or if the final pose is reached,
        command zero motion.
    */
    if (!hasCurrentPose_ || path_.poses.empty() || isGoalReached())
    {
        return Twist2d{0.0, Eigen::Vector2d::Zero()};
    }

    /*
        Pick a lookahead target instead of the closest pose directly:
            targetIndex = min(closestIndex + lookahead, lastIndex)
    */
    const std::size_t targetIndex =
        std::min(closestPoseIndex_ + kLookaheadSteps, path_.poses.size() - 1);

    const Eigen::Isometry2d& targetPose = path_.poses[targetIndex];

    // Position error in global frame: ep = target_position - current_position.
    const Eigen::Vector2d positionError =
        targetPose.translation() - currentPose_.translation();

    /*
        Yaw error:
            e_yaw = wrap(target_yaw - current_yaw)

        normalizeAngle() ensures the shortest rotation direction is used.
    */
    const double targetYaw = extractYaw(targetPose);
    const double currentYaw = extractYaw(currentPose_);
    const double angleError = normalizeAngle(targetYaw - currentYaw);

    Twist2d cmd;

    /*
        Convert position error to translational velocity:
            v = Kp * ep

        The vector is then limited to avoid excessive center speed.
    */
    cmd.tv = clampNorm(kPositionGain * positionError, maxWheelSpeed_);

    // Convert yaw error to angular velocity: w = Kr * e_yaw.
    double rotationCmd = kRotationGain * angleError;

    // Clamp angular velocity manually for compatibility with older compilers.
    if (rotationCmd > kMaxRotationSpeed)
    {
        rotationCmd = kMaxRotationSpeed;
    }

    if (rotationCmd < -kMaxRotationSpeed)
    {
        rotationCmd = -kMaxRotationSpeed;
    }

    cmd.rv = rotationCmd;

    /*
        When the lookahead target is the final path pose, reduce the command
        near the goal. This helps avoid overshooting the final position/angle.
    */
    if (targetIndex == path_.poses.size() - 1)
    {
        const double distanceToGoal = positionError.norm();

        if (distanceToGoal < 0.1)
        {
            cmd.tv *= distanceToGoal / 0.1;
        }

        if (std::abs(angleError) < 0.25)
        {
            cmd.rv *= std::abs(angleError) / 0.25;
        }
    }

    /*
        Ensure that the parcel corners/wheels do not exceed the configured
        maximum speed.
    */
    return applyWheelSpeedLimit(cmd);
}

double PathFollower::normalizeAngle(double angle)
{
    // Wrap any angle to the range [-pi, pi].
    return std::atan2(std::sin(angle), std::cos(angle));
}

double PathFollower::extractYaw(const Eigen::Isometry2d& pose)
{
    /*
        Eigen::Isometry2d stores translation and rotation.
        This extracts the 2D rotation angle.
    */
    return Eigen::Rotation2Dd(pose.rotation()).angle();
}

Eigen::Vector2d PathFollower::clampNorm(const Eigen::Vector2d& vector, double maxNorm)
{
    /*
        If the vector magnitude is larger than maxNorm, scale it down
        while preserving direction.
    */
    const double norm = vector.norm();

    if (norm <= maxNorm || norm == 0.0)
    {
        return vector;
    }

    return vector * (maxNorm / norm);
}

std::size_t PathFollower::findClosestPoseIndex() const
{
    double bestScore = std::numeric_limits<double>::infinity();
    std::size_t bestIndex = closestPoseIndex_;

    const double currentYaw = extractYaw(currentPose_);

    /*
        Search forward from the previous closest index. This assumes normal
        forward progress along the path and avoids jumping back to old poses.
    */
    for (std::size_t index = closestPoseIndex_; index < path_.poses.size(); ++index)
    {
        const Eigen::Vector2d delta =
            path_.poses[index].translation() - currentPose_.translation();

        // Position part of the score: squared distance to this path pose.
        const double positionScore = delta.squaredNorm();

        // Orientation part of the score: absolute wrapped yaw difference.
        const double yawScore =
            std::abs(normalizeAngle(extractYaw(path_.poses[index]) - currentYaw));

        /*
            Position is the dominant term; yaw is included with a small weight
            to prefer poses with similar orientation when distances are similar.
        */
        const double combinedScore =
            positionScore + 0.02 * yawScore * yawScore;

        if (combinedScore < bestScore)
        {
            bestScore = combinedScore;
            bestIndex = index;
        }
    }

    return bestIndex;
}

Twist2d PathFollower::applyWheelSpeedLimit(const Twist2d& cmd) const
{
    /*
        Define the four parcel corners in the parcel's local frame.

        If parcel size is (width, height), the corners are:
            (+w/2, +h/2), (+w/2, -h/2),
            (-w/2, +h/2), (-w/2, -h/2)
    */
    const std::array<Eigen::Vector2d, 4> localCorners = {
        Eigen::Vector2d(halfParcelSize_.x(), halfParcelSize_.y()),
        Eigen::Vector2d(halfParcelSize_.x(), -halfParcelSize_.y()),
        Eigen::Vector2d(-halfParcelSize_.x(), halfParcelSize_.y()),
        Eigen::Vector2d(-halfParcelSize_.x(), -halfParcelSize_.y())};

    /*
        Convert corner vectors from parcel frame to world frame using the
        current parcel orientation.
    */
    const Eigen::Rotation2Dd worldFromParcel(extractYaw(currentPose_));

    double maxPointSpeed = 0.0;

    for (const Eigen::Vector2d& localCorner : localCorners)
    {
        const Eigen::Vector2d globalCorner = worldFromParcel * localCorner;

        // Velocity of a point caused by rotation: v_rot = (-w * y, w * x).
        const Eigen::Vector2d rotationalVelocity(
            -cmd.rv * globalCorner.y(),
             cmd.rv * globalCorner.x());

        // Total corner velocity is center translation plus rotational velocity.
        const double pointSpeed =
            (cmd.tv + rotationalVelocity).norm();

        maxPointSpeed = std::max(maxPointSpeed, pointSpeed);
    }

    // If the fastest corner is within the limit, keep the command unchanged.
    if (maxPointSpeed <= maxWheelSpeed_ || maxPointSpeed == 0.0)
    {
        return cmd;
    }

    /*
        Otherwise scale both translation and rotation by the same factor.
        This keeps the motion direction the same but makes it slower.
    */
    const double scale = maxWheelSpeed_ / maxPointSpeed;

    return Twist2d{cmd.rv * scale, cmd.tv * scale};
}

bool PathFollower::isGoalReached() const
{
    // Without a valid pose or path, avoid commanding motion.
    if (!hasCurrentPose_ || path_.poses.empty())
    {
        return true;
    }

    const Eigen::Isometry2d& goalPose = path_.poses.back();

    // Position error to the final pose.
    const Eigen::Vector2d translationError =
        goalPose.translation() - currentPose_.translation();

    // Yaw error to the final pose.
    const double angleError =
        normalizeAngle(extractYaw(goalPose) - extractYaw(currentPose_));

    // Goal is reached only when both position and orientation are close enough.
    return translationError.norm() < kGoalPositionTolerance &&
           std::abs(angleError) < kGoalAngleTolerance;
}