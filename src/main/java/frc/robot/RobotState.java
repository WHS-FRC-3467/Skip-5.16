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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
        // NAME(BLUE, RED, HEIGHT)
        ONE(new Pose2d(Meters.of(0), Meters.of(0), Rotation2d.kZero),
            new Pose2d(FieldConstants.FIELD_LENGTH, FieldConstants.FIELD_WIDTH,
                Rotation2d.k180deg),
            Meters.of(2.64)),
        TWO(new Pose2d(), new Pose2d(), Meters.of(0.0));

        @Getter
        private final Pose2d bluePose;

        @Getter
        private final Pose2d redPose;

        @Getter
        private final Distance height;

    }

    /** Keeps track of current target to aim for */
    @AutoLogOutput(key = "Robot/CurrentTarget")
    @Setter
    public static Target target = Target.ONE;

    /** Returns 2d distance from robot to target in meters */
    public Distance getDistanceToTarget(Pose2d robotPose)
    {
        Translation2d robotTranslation = robotPose.getTranslation();
        Translation2d targetTranslation =
            (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? target.bluePose.getTranslation()
                : target.redPose.getTranslation());
        return Meters.of(robotTranslation.getDistance(targetTranslation));
    }

    /** Returns 2d distance from robot to target in meters */
    public static Distance getDistanceToTarget(Translation2d robotPose)
    {
        Translation2d targetTranslation =
            (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? target.bluePose.getTranslation()
                : target.redPose.getTranslation());
        return Meters.of(robotPose.getDistance(targetTranslation));
    }

    /** Returns angle from robot to target */
    public static Rotation2d getAngleToTarget(Translation2d robotPose)
    {
        return (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? target.bluePose.getTranslation()
            : target.redPose.getTranslation()).minus(robotPose).getAngle();
    }

    /** Returns target pose based on alliance color */
    public static Pose2d getTargetPose(Target target)
    {
        return (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? target.getBluePose()
            : target.getRedPose());
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

    public Trigger SelfHubActiveTrigger()
    {

        return new Trigger(() -> (isHubActive().isPresent() ? isHubActive().get() : false));
        // TODO true false "empty"
    }

    public Optional<Boolean> isHubActive()
    {
        String gameData;
        gameData = DriverStation.getGameSpecificMessage();
        Alliance alliance = DriverStation.getAlliance().get();
        if (gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case 'B':
                    // Blue case code
                    switch (alliance) {
                        case Blue:
                            return Optional.of(true);

                        case Red:
                            return Optional.of(false);

                        default:
                            return Optional.empty();

                    }

                case 'R':
                    // Red case code
                    switch (alliance) {
                        case Blue:
                            return Optional.of(false);

                        case Red:
                            return Optional.of(true);

                        default:
                            return Optional.empty();

                    }

                default:
                    // This is corrupt data
                    return Optional.empty();

            }

        } else {
            return Optional.empty();
        }
    }
}
