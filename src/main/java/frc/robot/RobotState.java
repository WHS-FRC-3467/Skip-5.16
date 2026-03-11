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

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.posestimator.PoseEstimator;
import frc.lib.posestimator.PoseEstimator.VisionPoseObservation;
import frc.lib.posestimator.SwerveOdometry.OdometryObservation;
import frc.lib.util.FieldUtil;
import frc.lib.util.LoggedTrigger;
import frc.lib.util.LoggedTunableBoolean;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import lombok.AccessLevel;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class RobotState {

    private static final LoggedTunableNumber SHOOT_TOLERANCE_DEGREES =
            new LoggedTunableNumber("RobotState/ShootToleranceDegrees", 5.0);
    private static final LoggedTunableNumber MAX_HOOD_RETRACT_TIME =
            new LoggedTunableNumber("RobotState/MaxHoodRetractTime", 0.2);

    private static final double LINEAR_ODOMETRY_STD_DEV = 0.01;
    private static final double ANGULAR_ODOMETRY_STD_DEV = 0.01;

    @Getter(lazy = true)
    private static final RobotState instance = new RobotState();

    @Setter @Getter private boolean drivetrainAngled = false;

    @AutoLogOutput(key = "Drive/ActiveTrajectoryPose")
    @Getter
    @Setter
    private Pose2d activeTrajPose = new Pose2d();

    public LoggedTunableBoolean forcePathFind =
            new LoggedTunableBoolean("RobotState/ForcePathFind", false);

    @AutoLogOutput(key = "Drive/ActiveTrajectoryError")
    public Distance getActiveTrajectoryError() {
        return Meters.of(
                getEstimatedPose().getTranslation().getDistance(activeTrajPose.getTranslation()));
    }

    @AutoLogOutput(key = "Drive/DrivetrainAngled")
    private final Trigger drivetrainAngledTrigger =
            new Trigger(() -> drivetrainAngled).debounce(0.5, DebounceType.kFalling);

    @Getter
    @AutoLogOutput(key = "Drive/FacingTarget")
    public final Trigger facingTarget =
            new Trigger(
                    () ->
                            Math.abs(
                                            getAngleToTarget()
                                                    .minus(getEstimatedPose().getRotation())
                                                    .getDegrees())
                                    < SHOOT_TOLERANCE_DEGREES.get());

    /**
     * Whether or not the robot is entering the trench in {@code MAX_HOOD_RETRACT_TIME}. For use to
     * check whether we need to force the hood to retract to prevent decapitation.
     */
    public final LoggedTrigger enteringTrench =
            new LoggedTrigger(
                    "RobotState/EnteringTrench",
                    () -> {
                        // Test if the robot is in motion
                        ChassisSpeeds chassisVelocity = getFieldRelativeVelocity();
                        double linearVelocityMPS =
                                Math.hypot(
                                        chassisVelocity.vxMetersPerSecond,
                                        chassisVelocity.vyMetersPerSecond);
                        double angularVelocityRadsPS = chassisVelocity.omegaRadiansPerSecond;

                        // Arbitrary
                        boolean inMotion = linearVelocityMPS > 0.02 || angularVelocityRadsPS > 0.03;

                        // Predict future pose
                        Pose2d futurePose =
                                getEstimatedPose()
                                        .exp(
                                                getFieldRelativeVelocity()
                                                        .toTwist2d(MAX_HOOD_RETRACT_TIME.get()));

                        // Normalize to alliance frame
                        Pose2d pose = FieldUtil.apply(futurePose);

                        double x = pose.getX();
                        double y = pose.getY();

                        double halfRobotLength = Constants.FULL_ROBOT_LENGTH.in(Meters) / 2.0;

                        // Alliance trench corridor
                        double trenchMinX =
                                FieldConstants.LeftBump.NEAR_LEFT_CORNER.getX() - halfRobotLength;
                        double trenchMaxX =
                                FieldConstants.LeftBump.FAR_LEFT_CORNER.getX() + halfRobotLength;

                        // Opponent trench corridor (mirrored)
                        double oppTrenchMinX = FieldConstants.FIELD_LENGTH - trenchMaxX;
                        double oppTrenchMaxX = FieldConstants.FIELD_LENGTH - trenchMinX;

                        boolean inAllianceCorridor = x >= trenchMinX && x <= trenchMaxX;

                        boolean inOpponentCorridor = x >= oppTrenchMinX && x <= oppTrenchMaxX;

                        boolean inRightTrench =
                                y >= FieldConstants.LinesHorizontal.RIGHT_TRENCH_OPEN_END
                                        && y
                                                <= FieldConstants.LinesHorizontal
                                                        .RIGHT_TRENCH_OPEN_START;

                        boolean inLeftTrench =
                                y >= FieldConstants.LinesHorizontal.LEFT_TRENCH_OPEN_END
                                        && y
                                                <= FieldConstants.LinesHorizontal
                                                        .LEFT_TRENCH_OPEN_START;

                        return (inAllianceCorridor || inOpponentCorridor)
                                && (inLeftTrench || inRightTrench)
                                // Allow hood actuation while stationary
                                && inMotion;
                    });

    /** Trigger determining whether hood is safe to actuate for a HUB shot in auto. */
    public final LoggedTrigger hoodSafe =
            new LoggedTrigger(
                    "RobotState/hoodSafe",
                    () ->
                            getFieldRegion() == FieldRegion.ALLIANCE_ZONE
                                    && enteringTrench.negate().getAsBoolean());

    // -------- POSE ESTIMATION --------

    private final PoseEstimator poseEstimator =
            new PoseEstimator(
                    new SwerveDriveKinematics(
                            Drive.MODULE_TRANSLATIONS.toArray(Translation2d[]::new)),
                    Drive.MODULE_TRANSLATIONS.toArray(Translation2d[]::new),
                    Seconds.of(2),
                    LINEAR_ODOMETRY_STD_DEV,
                    ANGULAR_ODOMETRY_STD_DEV);

    @Setter private ChassisSpeeds velocity = new ChassisSpeeds();

    /**
     * Returns the robot's odometry-only pose (without vision corrections).
     *
     * @return the odometry-only pose
     */
    @AutoLogOutput(key = "Odometry/OdometryPose")
    public Pose2d getOdometryPose() {
        return poseEstimator.odometryPose();
    }

    /**
     * Returns the robot's estimated pose with vision corrections applied.
     *
     * @return the estimated pose
     */
    @AutoLogOutput(key = "Odometry/EstimatedPose")
    public Pose2d getEstimatedPose() {
        return poseEstimator.estimatedPose();
    }

    /**
     * Adds a new odometry observation to the pose estimator.
     *
     * @param observation the odometry observation to add
     */
    public void addOdometryObservation(OdometryObservation observation) {
        if (DriverStation.isDisabled()) return;

        if (drivetrainAngledTrigger.getAsBoolean()) {
            poseEstimator.addGyroObservation(observation);
            return;
        }

        poseEstimator.addOdometryObservation(observation);
    }

    /**
     * Adds a new vision observation to the pose estimator. Vision observations are ignored when the
     * drivetrain is tilted (e.g., going over a bump).
     *
     * @param observation the vision observation to add
     */
    public void addVisionObservation(VisionPoseObservation observation) {
        // Only add vision observation if robot is not angled (i.e. when going over a bump)

        if (DriverStation.isDisabled() || drivetrainAngledTrigger.getAsBoolean()) {
            resetPose(observation.robotPose());
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
    public Optional<Pose2d> getPoseAtTime(double timestampSeconds) {
        return poseEstimator.getPoseAtTime(timestampSeconds);
    }

    /**
     * Returns the robot's field-relative velocity.
     *
     * @return the field-relative chassis speeds
     */
    public ChassisSpeeds getFieldRelativeVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                velocity.vxMetersPerSecond,
                velocity.vyMetersPerSecond,
                velocity.omegaRadiansPerSecond,
                getEstimatedPose().getRotation());
    }

    /**
     * Resets the robot's pose to the specified position.
     *
     * @param pose the new pose to set
     */
    public void resetPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    // -------- ZONES --------

    /** Zones used for strategy/automation decisions (from the active alliance perspective). */
    public enum FieldRegion {
        ALLIANCE_ZONE,
        NEUTRAL_ZONE,
        OPPONENT_ALLIANCE_ZONE,
        LEFT_BUMP_TRENCH,
        RIGHT_BUMP_TRENCH
    }

    /**
     * Classifies the supplied pose into a broad field region.
     *
     * <p>The pose is transformed into the current alliance field frame via {@code
     * FieldUtil.apply(...)} so that {@link FieldRegion#ALLIANCE_ZONE} always refers to the current
     * alliance's side of the field (and {@link FieldRegion#OPPONENT_ALLIANCE_ZONE} to the far
     * side), regardless of whether the robot is actually on blue or red.
     *
     * <p>Bump/trench lanes are checked first so they take precedence over the coarse X-based zone
     * classification.
     *
     * @param currentPose The pose to check
     * @return The field region {@code currentPose} is in
     */
    public FieldRegion getFieldRegion(Pose2d currentPose) {
        // Pose in blue-side field frame (i.e., "alliance side" is always the current alliance).
        Pose2d pose = FieldUtil.apply(getEstimatedPose());
        double x = pose.getX();
        double y = pose.getY();

        // Identify whether the robot is within the longitudinal corridor that spans the
        // bump/trench.
        double halfRobotLength = Constants.FULL_ROBOT_LENGTH.in(Meters) / 2.0;
        double trenchMinX = FieldConstants.LeftBump.NEAR_LEFT_CORNER.getX() - halfRobotLength;
        double trenchMaxX = FieldConstants.LeftBump.FAR_LEFT_CORNER.getX() + halfRobotLength;
        boolean inTrenchXBand = x >= trenchMinX && x <= trenchMaxX;

        if (inTrenchXBand) {
            // Right lane occupies the low-Y side of the field.
            if (y >= FieldConstants.LinesHorizontal.RIGHT_TRENCH_OPEN_END
                    && y <= FieldConstants.LinesHorizontal.RIGHT_BUMP_START) {
                return FieldRegion.RIGHT_BUMP_TRENCH;
            }

            // Left lane occupies the high-Y side of the field.
            if (y >= FieldConstants.LinesHorizontal.LEFT_BUMP_END
                    && y <= FieldConstants.LinesHorizontal.LEFT_TRENCH_OPEN_START) {
                return FieldRegion.LEFT_BUMP_TRENCH;
            }
        }

        // Classify remaining space by the official X-based zone boundaries.
        if (x < FieldConstants.LinesVertical.NEUTRAL_ZONE_NEAR) {
            return FieldRegion.ALLIANCE_ZONE;
        } else if (x <= FieldConstants.LinesVertical.NEUTRAL_ZONE_FAR) {
            return FieldRegion.NEUTRAL_ZONE;
        } else {
            return FieldRegion.OPPONENT_ALLIANCE_ZONE;
        }
    }

    /**
     * Classifies the current pose into a broad field region.
     *
     * <p>The pose is transformed into the current alliance field frame via {@code
     * FieldUtil.apply(...)} so that {@link FieldRegion#ALLIANCE_ZONE} always refers to the current
     * alliance's side of the field (and {@link FieldRegion#OPPONENT_ALLIANCE_ZONE} to the far
     * side), regardless of whether the robot is actually on blue or red.
     *
     * <p>Bump/trench lanes are checked first so they take precedence over the coarse X-based zone
     * classification.
     *
     * @return The field region the robot is in
     */
    @AutoLogOutput(key = "RobotState/FieldRegion")
    public FieldRegion getFieldRegion() {
        return getFieldRegion(getEstimatedPose());
    }

    /** Returns the nearest cardinal angle (multiple of 90 degrees) to the current robot angle. */
    private Rotation2d getNearestCardinalAngle() {
        double currentAngle = getEstimatedPose().getRotation().getDegrees();
        double nearestCardinalAngle = Math.round(currentAngle / 90.0) * 90.0;
        return Rotation2d.fromDegrees(nearestCardinalAngle);
    }

    /**
     * Gets the heading the robot should maintain while driving through (or approaching) a trench
     * lane.
     *
     * <p>The desired heading depends on where the robot is relative to the trench corridor:
     *
     * <ul>
     *   <li>When in a bump/trench lane, the heading snaps to the nearest cardinal direction so the
     *       robot stays square with the lane.
     *   <li>When outside a lane, the heading points toward the nearer side of the lane corridor:
     *       toward midfield when on the alliance wall side, and back toward the alliance wall when
     *       on the midfield side (mirrored on the opponent half).
     * </ul>
     *
     * <p>All logic is evaluated in the blue-alliance field frame via {@code FieldUtil.apply(...)}
     * so the behavior is consistent on both alliances.
     *
     * @return the desired field-relative heading
     */
    public Rotation2d getTunnelAssistHeading() {
        // Evaluate position in blue-side frame for consistent "alliance/opponent" semantics.
        Pose2d pose = FieldUtil.apply(getEstimatedPose());
        double x = pose.getX();

        double halfRobotLength = Constants.FULL_ROBOT_LENGTH.in(Meters) / 2.0;

        // X extents of the trench corridor on the alliance half (blue-side frame).
        double trenchMinX = FieldConstants.LeftBump.NEAR_LEFT_CORNER.getX() - halfRobotLength;
        double trenchMaxX = FieldConstants.LeftBump.FAR_LEFT_CORNER.getX() + halfRobotLength;

        // Mirrored corridor extents on the opponent half (blue-side frame).
        double oppTrenchMinX = FieldConstants.FIELD_LENGTH - trenchMinX;
        double oppTrenchMaxX = FieldConstants.FIELD_LENGTH - trenchMaxX;

        return switch (getFieldRegion()) {
            case LEFT_BUMP_TRENCH, RIGHT_BUMP_TRENCH -> getNearestCardinalAngle();

            case ALLIANCE_ZONE -> {
                if (x < trenchMinX) {
                    // Alliance-wall side of the corridor: point toward midfield.
                    yield FieldUtil.apply(Rotation2d.kZero);
                } else if (x > trenchMaxX) {
                    // Midfield side of the corridor: point back toward the alliance wall.
                    yield FieldUtil.apply(Rotation2d.k180deg);
                } else {
                    // Inside the corridor but not classified into a lane: stay square.
                    yield getNearestCardinalAngle();
                }
            }

            case NEUTRAL_ZONE -> {
                // In the official neutral zone, bias toward the nearer half's direction.
                yield FieldUtil.apply(
                        x < FieldConstants.FIELD_CENTER.getX()
                                ? Rotation2d.kZero
                                : Rotation2d.k180deg);
            }

            case OPPONENT_ALLIANCE_ZONE -> {
                if (x > oppTrenchMinX) {
                    // Opponent-wall side of the corridor: point toward midfield/our side.
                    yield FieldUtil.apply(Rotation2d.k180deg);
                } else if (x < oppTrenchMaxX) {
                    // Midfield side of the corridor: point toward the opponent wall.
                    yield FieldUtil.apply(Rotation2d.kZero);
                } else {

                    // Inside the corridor but not classified into a lane: stay square.
                    yield getNearestCardinalAngle();
                }
            }
        };
    }

    // -------- TARGETS --------

    @AllArgsConstructor
    @SuppressWarnings("ImmutableEnumChecker")
    public enum Target {
        /** The Hub */
        HUB(
                new Translation3d(
                        FieldConstants.Hub.TOP_CENTER_POINT.getX(),
                        FieldConstants.Hub.TOP_CENTER_POINT.getY(),
                        FieldConstants.Hub.HEIGHT)),

        /**
         * Center of the left portion of the alliance zone (upper-Y half in the blue-alliance
         * frame).
         */
        FEED_LEFT(
                new Translation3d(
                        FieldConstants.LinesVertical.NEUTRAL_ZONE_NEAR / 2.0,
                        (FieldConstants.FIELD_CENTER.getY() + FieldConstants.FIELD_WIDTH) / 2.0,
                        0)),

        /**
         * Center of the right portion of the alliance zone (lower-Y half in the blue-alliance
         * frame).
         */
        FEED_RIGHT(
                new Translation3d(
                        FieldConstants.LinesVertical.NEUTRAL_ZONE_NEAR / 2.0,
                        (0.0 + FieldConstants.FIELD_CENTER.getY()) / 2.0,
                        0));

        private final Translation3d blueTranslation;

        /**
         * Returns the target translation with alliance flip applied.
         *
         * @return the target translation adjusted for the current alliance
         */
        public Translation3d getAllianceTranslation() {
            return FieldUtil.apply(blueTranslation);
        }
    }

    @AutoLogOutput(key = "Robot/CurrentTarget")
    public Target getTarget() {
        FieldRegion region = getFieldRegion();

        // Target the hub anywhere on our side *including* the near bump/trench lanes.
        if (region == FieldRegion.ALLIANCE_ZONE
                || region == FieldRegion.LEFT_BUMP_TRENCH
                || region == FieldRegion.RIGHT_BUMP_TRENCH) {
            return Target.HUB;
        }

        Pose2d pose = FieldUtil.apply(getEstimatedPose());
        return pose.getY() >= FieldConstants.FIELD_CENTER.getY()
                ? Target.FEED_LEFT
                : Target.FEED_RIGHT;
    }

    /**
     * Returns 2D distance from robot to target.
     *
     * @return the distance to the target
     */
    public Distance getDistanceToTarget() {
        Translation2d robotTranslation = getEstimatedPose().getTranslation();
        Translation2d targetTranslation = getTarget().getAllianceTranslation().toTranslation2d();
        return Meters.of(robotTranslation.getDistance(targetTranslation));
    }

    /**
     * Returns the angle from the robot to the current target.
     *
     * @return the angle to the target
     */
    public Rotation2d getAngleToTarget() {
        return getTarget()
                .getAllianceTranslation()
                .toTranslation2d()
                .minus(getEstimatedPose().getTranslation())
                .getAngle();
    }
}
