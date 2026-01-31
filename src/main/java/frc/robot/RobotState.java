/*
 * Copyright (C) 2026 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If
 * not, see <https://www.gnu.org/licenses/>.
 */

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Distance;
import frc.lib.posestimator.PoseEstimator;
import frc.lib.posestimator.PoseEstimator.VisionPoseObservation;
import frc.lib.posestimator.SwerveOdometry.OdometryObservation;
import frc.robot.subsystems.drive.Drive;
import lombok.AccessLevel;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class RobotState {

    private static final double LINEAR_ODOMETRY_STD_DEV = 0.01;
    private static final double ANGULAR_ODOMETRY_STD_DEV = 0.01;


    @Setter
    @AutoLogOutput(key = "Drive/DrivetrainAngled")
    private boolean drivetrainAngled = false;

    @Getter(lazy = true)
    private static final RobotState instance = new RobotState();

    private final PoseEstimator poseEstimator = new PoseEstimator(
        new SwerveDriveKinematics(Drive.getModuleTranslations()),
        Seconds.of(2),
        LINEAR_ODOMETRY_STD_DEV,
        ANGULAR_ODOMETRY_STD_DEV);

    @Getter
    @Setter
    private ChassisSpeeds velocity = new ChassisSpeeds();

    @AutoLogOutput(key = "Odometry/OdometryPose")
    public Pose2d getOdometryPose()
    {
        return poseEstimator.odometryPose();
    }

    @AutoLogOutput(key = "Odometry/EstimatedPose")
    public Pose2d getEstimatedPose()
    {
        return poseEstimator.estimatedPose();
    }

    public void addOdometryObservation(OdometryObservation observation)
    {
        poseEstimator.addOdometryObservation(observation);
    }

    public void addVisionObservation(VisionPoseObservation observation)
    {
        // Only add vision observation if robot is not angled (i.e. when going over a bump)
        if (drivetrainAngled) {
            return;
        }
        poseEstimator.addVisionObservation(observation);
    }

    public Optional<Pose2d> getPoseAtTime(double timestampSeconds)
    {
        return poseEstimator.getPoseAtTime(timestampSeconds);
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation()
    {
        return getEstimatedPose().getRotation();
    }

    public ChassisSpeeds getFieldRelativeVelocity()
    {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            velocity.vxMetersPerSecond,
            velocity.vyMetersPerSecond,
            velocity.omegaRadiansPerSecond,
            getRotation());
    }

    @Getter
    @Setter
    private Pose3d rotaryPose = new Pose3d();

    @Getter
    @Setter
    private Pose3d linearPose = new Pose3d();

    /**
     * Publishes the mechanism poses to the logger for 3d visualization. This should be changed to
     * match the mechanical kinematics of the robot.
     */

    public void publishMechanismPoses()
    {
        Logger.recordOutput("Odometry/LinearPose", linearPose);
        Logger.recordOutput("Odometry/RotaryPose", new Pose3d(
            getRotaryPose().getX(),
            getRotaryPose().getY(),
            getRotaryPose().getZ() + getLinearPose().getZ(),
            getRotaryPose().getRotation()));
    }

    public void resetPose(Pose2d pose)
    {
        poseEstimator.resetPose(pose);
    }

    @AllArgsConstructor
    public enum Target {
        // NAME(Pose, Height)
        HUB(new Pose2d(FieldConstants.Hub.INNER_CENTER_POINT.getX(),
            FieldConstants.Hub.INNER_CENTER_POINT.getY(), Rotation2d.kZero),
            Meters.of(FieldConstants.Hub.HEIGHT)),

        FEED_DEPOT(new Pose2d(FieldConstants.Depot.rightCorner.getX(),
            FieldConstants.Depot.rightCorner.getY(), Rotation2d.kZero),
            Meters.of(0)),

        FEED_OUTPOST(new Pose2d(FieldConstants.Outpost.CENTER_POINT.getX(),
            FieldConstants.Outpost.CENTER_POINT.getY(), Rotation2d.kZero),
            Meters.of(0));

        @Getter
        private final Pose2d pose;

        @Getter
        private final Distance height;

    }


    /** Keeps track of current target to aim for */
    @AutoLogOutput(key = "Robot/CurrentTarget")
    @Setter
    public static Target target = Target.HUB;

    /** Returns 2d distance from robot to target in meters */
    public Distance getDistanceToTarget(Pose2d robotPose)
    {
        Translation2d robotTranslation = robotPose.getTranslation();
        Translation2d targetTranslation = target.pose.getTranslation();
        return Meters.of(robotTranslation.getDistance(targetTranslation));
    }

    /** Returns 2d distance from robot to target in meters */
    public static Distance getDistanceToTarget(Translation2d robotPose)
    {
        Translation2d targetTranslation = target.pose.getTranslation();
        return Meters.of(robotPose.getDistance(targetTranslation));
    }

    /** Returns angle from robot to target */
    public static Rotation2d getAngleToTarget(Translation2d robotPose)
    {
        return target.pose.getTranslation().minus(robotPose).getAngle();
    }

    /**
     * Get the pose of the input target
     * 
     * @param target The target to get the pose from
     * @return The pose from the target
     */
    public static Pose2d getTargetPose(Target target)
    {
        return target.pose;
    }

    /**
     * Computes the vertical distance from the robot's current mechanism position to the target
     * using the robot's rotary and linear poses.
     *
     * @return height difference from mechanism to target, in meters
     */
    public Distance getHeightToTarget()
    {
        double mechanismHeight = getRotaryPose().getZ() + getLinearPose().getZ();
        return target.getHeight().minus(Meters.of(mechanismHeight));
    }

    /**
     * 
     * @param mechanismHeight height of the mechanism from which the projectile is launched
     * @return height difference from mechanism to target, in meters
     */
    public static Distance getHeightToTarget(Distance mechanismHeight)
    {
        return target.getHeight().minus(mechanismHeight);
    }
}
