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
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.posestimator.PoseEstimator;
import frc.lib.posestimator.PoseEstimator.VisionPoseObservation;
import frc.lib.posestimator.SwerveOdometry.OdometryObservation;
import frc.lib.util.FieldUtil;
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


    public BooleanSupplier auto = () -> false;
    @Getter
    private Trigger isAuto = new Trigger(auto);

    /**
     * Returns the robot's odometry-only pose (without vision corrections).
     * 
     * @return the odometry-only pose
     */
    @AutoLogOutput(key = "Odometry/OdometryPose")
    public Pose2d getOdometryPose()
    {
        return poseEstimator.odometryPose();
    }

    /**
     * Returns the robot's estimated pose with vision corrections applied.
     * 
     * @return the estimated pose
     */
    @AutoLogOutput(key = "Odometry/EstimatedPose")
    public Pose2d getEstimatedPose()
    {
        return poseEstimator.estimatedPose();
    }

    /**
     * Adds a new odometry observation to the pose estimator.
     * 
     * @param observation the odometry observation to add
     */
    public void addOdometryObservation(OdometryObservation observation)
    {
        poseEstimator.addOdometryObservation(observation);
    }

    /**
     * Adds a new vision observation to the pose estimator. Vision observations are ignored when the
     * drivetrain is tilted (e.g., going over a bump).
     * 
     * @param observation the vision observation to add
     */
    public void addVisionObservation(VisionPoseObservation observation)
    {
        // Only add vision observation if robot is not angled (i.e. when going over a bump)
        if (drivetrainAngled) {
            return;
        }
        poseEstimator.addVisionObservation(observation);
    }

    /**
     * Returns the robot's estimated pose at a specific timestamp.
     * 
     * @param timestampSeconds the timestamp in seconds
     * @return the estimated pose at the given timestamp, or empty if unavailable
     */
    public Optional<Pose2d> getPoseAtTime(double timestampSeconds)
    {
        return poseEstimator.getPoseAtTime(timestampSeconds);
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation()
    {
        return getEstimatedPose().getRotation();
    }

    /**
     * Returns the nearest cardinal angle (multiple of 90 degrees) to the current robot angle.
     */
    private Rotation2d getNearestCardinalAngle()
    {
        double currentAngle = getRotation().getDegrees();
        double nearestCardinalAngle = Math.round(currentAngle / 90.0) * 90.0;
        if (nearestCardinalAngle >= 180.0) {
            nearestCardinalAngle -= 360.0;
        } else if (nearestCardinalAngle < -180.0) {
            nearestCardinalAngle += 360.0;
        }
        return Rotation2d.fromDegrees(nearestCardinalAngle);
    }

    /**
     * Gets the heading the robot should maintain while driving in a trench.
     * <p>
     * The robot will either align itself square with the trench if it is inside or partially inside
     * the trench, or point towards the opposite side of the trench it is nearest to if it is fully
     * outside the trench.
     * </p>
     * <p>
     * Two main cases: driving under our alliance trenches, and driving under the opposing alliance
     * trenches.
     * </p>
     * 
     * <h4>Case 1: Our Alliance trenches</h4>
     * <p>
     * If the robot is in the trench, return the nearest cardinal direction so the robot is square
     * with the trench.
     * </p>
     * <p>
     * If the robot is outside of the trench but on the ALLIANCE side, return the angle towards the
     * CENTER line.
     * </p>
     * <p>
     * If the robot is outside of the trench but on the CENTER side, return the angle towards the
     * ALLIANCE side.
     * </p>
     * 
     * <h4>Case 2: Opposing Alliance trenches</h4>
     * <p>
     * If the robot is in the trench, return the nearest cardinal direction so the robot is square
     * with the trench.
     * </p>
     * <p>
     * If the robot is outside of the trench but on the OPPONENT ALLIANCE side, return the angle
     * towards the CENTER line.
     * </p>
     * <p>
     * If the robot is outside of the trench but on the NEUTRAL ZONE, return the angle towards the
     * OPPONENT ALLIANCE side.
     * </p>
     * 
     * @return the desired heading
     */
    public Rotation2d getTunnelAssistHeading()
    {
        // FieldUtil.apply() transforms the pose into the blue-side field frame (flips X and Y and
        // rotates 180° when on red alliance)
        Pose2d pose = FieldUtil.apply(getEstimatedPose());
        double halfRobotLength = Constants.FULL_ROBOT_LENGTH.in(Meters) / 2.0;
        // Check which side of the field the robot is on
        if (pose.getX() < FieldConstants.FIELD_CENTER.getX()) {
            // On our alliance's half of the field
            if (pose.getX() < FieldConstants.LeftBump.NEAR_LEFT_CORNER.getX() - halfRobotLength) {
                // The robot is outside of the trench, on alliance side --> point towards center
                return FieldUtil.apply(Rotation2d.kZero);
            } else if (pose.getX() > FieldConstants.LeftBump.FAR_LEFT_CORNER.getX()
                + halfRobotLength) {
                // Robot is outside of the trench, in the NEUTRAL area --> point towards our
                // alliance
                return FieldUtil.apply(Rotation2d.k180deg);
            } else {
                // Robot in trench --> return the nearest cardinal direction
                return getNearestCardinalAngle();
            }
        } else {
            // On the opposing alliance's half of the field - mirror the logic
            if (pose.getX() > FieldConstants.FIELD_LENGTH
                - (FieldConstants.LeftBump.NEAR_LEFT_CORNER.getX() - halfRobotLength)) {
                // The robot is outside of the trench, on opposing alliance side --> point towards
                // center/our alliance
                return FieldUtil.apply(Rotation2d.k180deg);
            } else if (pose.getX() < FieldConstants.FIELD_LENGTH
                - (FieldConstants.LeftBump.FAR_LEFT_CORNER.getX() + halfRobotLength)) {
                // Robot is outside of the trench, in the NEUTRAL area --> point towards opposing
                // alliance
                return FieldUtil.apply(Rotation2d.kZero);
            } else {
                // Robot in trench --> return the nearest cardinal direction
                return getNearestCardinalAngle();
            }
        }
    }

    /**
     * Returns the robot's field-relative velocity.
     * 
     * @return the field-relative chassis speeds
     */
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

    /**
     * Resets the robot's pose to the specified position.
     * 
     * @param pose the new pose to set
     */
    public void resetPose(Pose2d pose)
    {
        poseEstimator.resetPose(pose);
    }

    @AllArgsConstructor
    @SuppressWarnings("ImmutableEnumChecker")
    public enum Target {
        HUB(new Pose2d(
            FieldConstants.Hub.TOP_CENTER_POINT.getX(),
            FieldConstants.Hub.TOP_CENTER_POINT.getY(), Rotation2d.kZero),
            Meters.of(FieldConstants.Hub.HEIGHT)),

        FEED_DEPOT(new Pose2d(FieldConstants.Depot.rightCorner.getX(),
            FieldConstants.Depot.rightCorner.getY(), Rotation2d.kZero),
            Meters.of(0)),

        FEED_OUTPOST(new Pose2d(FieldConstants.Outpost.CENTER_POINT.getX(),
            FieldConstants.Outpost.CENTER_POINT.getY(), Rotation2d.kZero),
            Meters.of(0));

        private final Pose2d bluePose;

        @Getter
        private final Distance height;

        /**
         * Returns the target pose with alliance flip applied.
         * 
         * @return the target pose adjusted for the current alliance
         */
        public Pose2d getAlliancePose()
        {
            return FieldUtil.apply(bluePose);
        }

    }


    /** Keeps track of current target to aim for */
    @AutoLogOutput(key = "Robot/CurrentTarget")
    @Setter
    public static Target target = Target.HUB;

    /**
     * Returns 2D distance from robot to target.
     * 
     * @return the distance to the target
     */
    public Distance getDistanceToTarget()
    {
        Translation2d robotTranslation = getEstimatedPose().getTranslation();
        Translation2d targetTranslation = target.getAlliancePose().getTranslation();
        return Meters.of(robotTranslation.getDistance(targetTranslation));
    }

    /**
     * Returns 2D distance from a given translation to the current target.
     * 
     * @param robotTranslation the robot's current translation
     * @return the distance to the target
     */
    public static Distance getDistanceToTarget(Translation2d robotTranslation)
    {
        Translation2d targetTranslation = target.getAlliancePose().getTranslation();
        return Meters.of(robotTranslation.getDistance(targetTranslation));
    }

    /**
     * Returns the angle from the robot to the current target.
     * 
     * @return the angle to the target
     */
    public Rotation2d getAngleToTarget()
    {
        return target.getAlliancePose().getTranslation().minus(getEstimatedPose().getTranslation())
            .getAngle();
    }

    /**
     * Get the pose of the input target
     * 
     * @param target The target to get the pose from
     * @return The pose from the target
     */
    public static Pose2d getTargetPose(Target target)
    {
        return target.getAlliancePose();
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
     * Returns height difference from mechanism to target, in meters.
     * 
     * @param mechanismHeight height of the mechanism from which the projectile is launched
     * @return height difference from mechanism to target, in meters
     */
    public static Distance getHeightToTarget(Distance mechanismHeight)
    {
        return target.getHeight().minus(mechanismHeight);
    }
}
