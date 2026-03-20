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

package frc.lib.posestimator;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;

import frc.lib.posestimator.SwerveOdometry.OdometryObservation;

import lombok.Getter;
import lombok.experimental.Accessors;

import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Predicate;

@Accessors(fluent = true)
public class PoseEstimator {
    private static final String LOG_PREFIX = "PoseEstimator/";

    public static record VisionPoseObservation(
            double timestampSeconds,
            Pose2d robotPose,
            double avgTagDistance,
            int numTagsUsed,
            double linearStdDev,
            double angularStdDev) {}

    private enum OdometryObservationReason {
        ACCEPTED,
        POSE_VALIDATOR_REJECTED
    }

    private enum VisionObservationReason {
        ACCEPTED,
        MISSING_ODOMETRY_SAMPLE,
        BELOW_MIN_N_SIGMA,
        POSE_VALIDATOR_REJECTED
    }

    private static final double DEFAULT_ODOMETRY_BUFFER_SIZE_SECONDS = 2;

    /*
     * When wheels are skidding, wheel odometry is less reliable. We inflate the odometry standard
     * deviation used during vision fusion so vision has more authority during skid.
     *
     * These are multipliers on standard deviation. Variance is scaled by multiplier^2.
     */
    private static final double ODOMETRY_STDDEV_MULTIPLIER_PER_BAD_WHEEL_LINEAR = 0.75;
    private static final double ODOMETRY_STDDEV_MULTIPLIER_PER_BAD_WHEEL_ANGULAR = 0.50;
    private static final double ODOMETRY_STDDEV_MULTIPLIER_MAX_LINEAR = 4.0;
    private static final double ODOMETRY_STDDEV_MULTIPLIER_MAX_ANGULAR = 3.0;

    // Vision gating: ignore noise and reject large outliers.
    private static final double VISION_MIN_N_SIGMA_LINEAR = 0.6;
    private static final double VISION_MIN_N_SIGMA_ANGULAR = 0.6;

    private final SwerveOdometry odometry;

    /** Base odometry variances */
    private final double[] odometryVariances;

    private double odometryStdDevMultiplierLinear = 1.0;
    private double odometryStdDevMultiplierAngular = 1.0;

    private Predicate<Pose2d> poseValidator = pose -> true;

    @Getter private Pose2d estimatedPose = Pose2d.kZero;

    public PoseEstimator(
            SwerveDriveKinematics kinematics,
            Time odometryBufferSize,
            double linearOdometryStdDev,
            double angularOdometryStdDev) {
        this(kinematics, null, odometryBufferSize, linearOdometryStdDev, angularOdometryStdDev);
    }

    /**
     * If module translations are provided, odometry can ignore skidding wheels using the {@code
     * badWheels} array in {@link OdometryObservation}.
     */
    public PoseEstimator(
            SwerveDriveKinematics kinematics,
            Translation2d[] moduleTranslations,
            Time odometryBufferSize,
            double linearOdometryStdDev,
            double angularOdometryStdDev) {
        double linearOdometryVariance = Math.pow(linearOdometryStdDev, 2);
        double angularOdometryVariance = Math.pow(angularOdometryStdDev, 2);

        odometryVariances =
                new double[] {
                    linearOdometryVariance, // X axis
                    linearOdometryVariance, // Y axis
                    angularOdometryVariance // Rotation
                };

        odometry = new SwerveOdometry(kinematics, moduleTranslations, odometryBufferSize);
        refreshOdometryPoseValidator();
    }

    public PoseEstimator(
            SwerveDriveKinematics kinematics,
            double linearOdometryStdDev,
            double angularOdometryStdDev) {
        this(
                kinematics,
                Seconds.of(DEFAULT_ODOMETRY_BUFFER_SIZE_SECONDS),
                linearOdometryStdDev,
                angularOdometryStdDev);
    }

    public Pose2d odometryPose() {
        return odometry.odometryPose();
    }

    /**
     * Adds a new odometry observation to the pose estimator and updates the estimated pose.
     *
     * <p>This method retrieves the last odometry pose, applies the new odometry observation, and
     * calculates the change in pose (twist) between the last and new odometry poses. The estimated
     * pose is then updated by applying the calculated twist.
     *
     * @param observation The new odometry observation to be added. This observation typically
     *     contains information about the robot's movement such as displacement and rotation.
     */
    public void addOdometryObservation(OdometryObservation observation) {
        updateOdometryStdDevMultipliersFromSkid(observation.badWheels());

        Pose2d lastOdometryPose = odometry.odometryPose();
        odometry.addOdometryObservation(observation);
        Pose2d newOdometryPose = odometry.odometryPose();

        Twist2d twist = lastOdometryPose.log(newOdometryPose);

        estimatedPose = estimatedPose.exp(twist);
    }

    /**
     * Adds a new odometry observation to the pose estimator, ignores everything except gyro, and
     * updates the estimated pose.
     *
     * <p>This method retrieves the last odometry pose, applies the new odometry observation, and
     * calculates the change in pose (twist) between the last and new odometry poses. The estimated
     * pose is then updated by applying the calculated twist.
     *
     * @param observation The new odometry observation to be added. This observation typically
     *     contains information about the robot's movement such as displacement and rotation.
     */
    public void addGyroObservation(OdometryObservation observation) {
        Pose2d lastOdometryPose = odometry.odometryPose();
        odometry.addGyroObservation(observation);
        Pose2d newOdometryPose = odometry.odometryPose();

        Twist2d twist = lastOdometryPose.log(newOdometryPose);

        estimatedPose = estimatedPose.exp(twist);
    }

    private void updateOdometryStdDevMultipliersFromSkid(boolean[] badWheels) {
        if (badWheels == null) {
            odometryStdDevMultiplierLinear = 1.0;
            odometryStdDevMultiplierAngular = 1.0;
            return;
        }

        int badCount = 0;
        for (boolean bad : badWheels) {
            if (bad) {
                badCount++;
            }
        }

        double linear = 1.0 + (badCount * ODOMETRY_STDDEV_MULTIPLIER_PER_BAD_WHEEL_LINEAR);
        double angular = 1.0 + (badCount * ODOMETRY_STDDEV_MULTIPLIER_PER_BAD_WHEEL_ANGULAR);

        odometryStdDevMultiplierLinear = Math.min(linear, ODOMETRY_STDDEV_MULTIPLIER_MAX_LINEAR);
        odometryStdDevMultiplierAngular = Math.min(angular, ODOMETRY_STDDEV_MULTIPLIER_MAX_ANGULAR);
    }

    /**
     * Calculates the change in pose (delta) between the odometry pose at a given timestamp and the
     * current odometry pose. This is mainly used for latency compensation for vision.
     *
     * @param timestampSeconds The timestamp (in seconds) for which to retrieve the odometry pose.
     * @return An {@link Optional} containing the {@link Transform2d} representing the pose delta if
     *     the odometry pose at the given timestamp exists; otherwise, an empty {@link Optional}.
     */
    public Optional<Transform2d> getPoseDeltaThenToNow(double timestampSeconds) {
        var optionalOdometryPoseAtTime = odometry.odometryBuffer().getSample(timestampSeconds);
        if (optionalOdometryPoseAtTime.isEmpty()) {
            return Optional.empty();
        }
        Pose2d odometryPoseAtTime = optionalOdometryPoseAtTime.get();

        Transform2d thenToNow = odometry.odometryPose().minus(odometryPoseAtTime);

        return Optional.of(thenToNow);
    }

    /**
     * Retrieves the estimated pose of the robot at a specific timestamp.
     *
     * <p>This method calculates the pose at the given timestamp by determining the pose delta from
     * the current estimated pose and applying the inverse of that delta to the current pose.
     *
     * @param timestampSeconds The timestamp (in seconds) for which the pose is requested.
     * @return An {@code Optional<Pose2d>} containing the estimated pose at the specified timestamp,
     *     or an empty {@code Optional} if the pose cannot be determined.
     */
    public Optional<Pose2d> getPoseAtTime(double timestampSeconds) {
        return getPoseDeltaThenToNow(timestampSeconds)
                .map(thenToNow -> estimatedPose.plus(thenToNow.inverse()));
    }

    /**
     * Incorporates a vision-based pose observation into the pose estimator. This method uses a
     * Kalman filter approach to blend the vision-based observation with the current odometry-based
     * pose estimate, accounting for the variances in both sources of data.
     *
     * @param observation The vision pose observation
     */
    public void addVisionObservation(VisionPoseObservation observation) {
        // Attempt to get heading. Fails if the odometer has not recorded
        // a measurement near this timestamp
        Transform2d poseDeltaThenToNow =
                getPoseDeltaThenToNow(observation.timestampSeconds).orElse(null);
        if (poseDeltaThenToNow == null) {
            logVisionObservation(
                    observation,
                    null,
                    false,
                    VisionObservationReason.MISSING_ODOMETRY_SAMPLE,
                    Double.NaN,
                    Double.NaN);
            return;
        }

        Pose2d oldPose = estimatedPose.plus(poseDeltaThenToNow.inverse());
        Pose2d newVisionPose = observation.robotPose;
        // Utility listener characterizing vision measurement deviation from state
        // prediction for Kalman gain tuning

        // VisionOdometryCharacterizer.recordVisionCorrection(oldPose, observation);

        // Utility listener characterizing odometry prediction deviation from high
        // confidence "vision ground truth" measurement for Kalman gain tuning

        // VisionOdometryCharacterizer.recordOdometryCorrection(
        //         odometryPose().plus(poseDeltaThenToNow.inverse()), observation);

        double visionLinearVariance = observation.linearStdDev * observation.linearStdDev;
        double visionAngularVariance = observation.angularStdDev * observation.angularStdDev;

        // Solve Kalman gain matrix given observation standard deviations
        // Logic is copied from:
        // https://github.com/wpilibsuite/allwpilib/blob/b8d6bc2eb1b6cea10d1179939114d041945e172a/wpimath/src/main/java/edu/wpi/first/math/estimator/PoseEstimator.java#L93-L109
        double[] visionVariances = {
            visionLinearVariance, // X axis
            visionLinearVariance, // Y axis
            visionAngularVariance
        }; // Rotation

        double linearVarianceMultiplier =
                odometryStdDevMultiplierLinear * odometryStdDevMultiplierLinear;
        double angularVarianceMultiplier =
                odometryStdDevMultiplierAngular * odometryStdDevMultiplierAngular;

        double[] effectiveOdometryVariances = {
            odometryVariances[0] * linearVarianceMultiplier, // X axis
            odometryVariances[1] * linearVarianceMultiplier, // Y axis
            odometryVariances[2] * angularVarianceMultiplier // Rotation
        };

        // Transform between our best estimated pose at the time the frame was captured to where the
        // camera is saying we should be, unscaled (without any Kalman gain applied)
        // Logic is copied from:
        // https://github.com/wpilibsuite/allwpilib/blob/b8d6bc2eb1b6cea10d1179939114d041945e172a/wpimath/src/main/java/edu/wpi/first/math/estimator/PoseEstimator.java#L276-L292
        Transform2d unscaledVisionCorrection = new Transform2d(oldPose, newVisionPose);

        double translationSigma =
                Math.sqrt(Math.max(visionLinearVariance + effectiveOdometryVariances[0], 1.0e-9));
        double rotationSigma =
                Math.sqrt(Math.max(visionAngularVariance + effectiveOdometryVariances[2], 1.0e-9));

        double translationMag =
                Math.hypot(unscaledVisionCorrection.getX(), unscaledVisionCorrection.getY());
        double rotationMag = Math.abs(unscaledVisionCorrection.getRotation().getRadians());

        double translationNSigma = translationMag / translationSigma;
        double rotationNSigma = rotationMag / rotationSigma;

        if (translationNSigma < VISION_MIN_N_SIGMA_LINEAR
                && rotationNSigma < VISION_MIN_N_SIGMA_ANGULAR) {
            logVisionObservation(
                    observation,
                    oldPose,
                    false,
                    VisionObservationReason.BELOW_MIN_N_SIGMA,
                    translationNSigma,
                    rotationNSigma);
            return;
        }

        Matrix<N3, N3> visionKalmanGain = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; ++row) {
            double odometryVariance = effectiveOdometryVariances[row];
            if (odometryVariance == 0.0) {
                visionKalmanGain.set(row, row, 0.0);
            } else {
                visionKalmanGain.set(
                        row,
                        row,
                        odometryVariance
                                / (odometryVariance
                                        + Math.sqrt(odometryVariance * visionVariances[row])));
            }
        }

        // Scale the vision correction by the Kalman gain
        var scaledVisionCorrectionVector =
                visionKalmanGain.times(
                        VecBuilder.fill(
                                unscaledVisionCorrection.getX(),
                                unscaledVisionCorrection.getY(),
                                unscaledVisionCorrection.getRotation().getRadians()));

        // Convert to Transform2d
        Transform2d scaledVisionCorrection =
                new Transform2d(
                        scaledVisionCorrectionVector.get(0, 0),
                        scaledVisionCorrectionVector.get(1, 0),
                        Rotation2d.fromRadians(scaledVisionCorrectionVector.get(2, 0)));

        Pose2d candidatePose =
                oldPose.transformBy(scaledVisionCorrection) // Adjust by the correction
                        .transformBy(
                                poseDeltaThenToNow); // Bring back to present time (latency comp)

        if (!poseValidator.test(candidatePose)) {
            logVisionObservation(
                    observation,
                    candidatePose,
                    false,
                    VisionObservationReason.POSE_VALIDATOR_REJECTED,
                    translationNSigma,
                    rotationNSigma);
            return;
        }

        estimatedPose = candidatePose;
        logVisionObservation(
                observation,
                candidatePose,
                true,
                VisionObservationReason.ACCEPTED,
                translationNSigma,
                rotationNSigma);
    }

    /**
     * Resets the pose estimator to a known field-relative pose.
     *
     * <p>This method should be called when the robot's absolute position on the field is known —
     * for example, at the start of autonomous or after a vision-based correction. It reinitializes
     * both the odometry integrator and the estimator’s internal pose state to the specified pose.
     *
     * @param pose the known {@link Pose2d} representing the robot’s field-relative position
     */
    public void resetPose(Pose2d pose) {
        odometry.resetPose(pose);
        estimatedPose = pose;

        odometryStdDevMultiplierLinear = 1.0;
        odometryStdDevMultiplierAngular = 1.0;
    }

    /**
     * Sets a validator that applies to both odometry and vision-fused poses.
     *
     * @param validator predicate that returns {@code true} for acceptable poses
     * @return this
     */
    public PoseEstimator withPoseValidator(Predicate<Pose2d> validator) {
        poseValidator = validator;
        refreshOdometryPoseValidator();
        return this;
    }

    private void refreshOdometryPoseValidator() {
        odometry.setPoseValidator(
                pose -> {
                    boolean accepted = poseValidator.test(pose);
                    OdometryObservationReason reason =
                            accepted
                                    ? OdometryObservationReason.ACCEPTED
                                    : OdometryObservationReason.POSE_VALIDATOR_REJECTED;
                    logOdometryObservation(pose, accepted, reason);
                    return accepted;
                });
    }

    private void logOdometryObservation(
            Pose2d candidatePose, boolean accepted, OdometryObservationReason reason) {
        Logger.recordOutput(LOG_PREFIX + "Odometry/Accepted", accepted);
        Logger.recordOutput(LOG_PREFIX + "Odometry/Reason", reason.name());
        Logger.recordOutput(LOG_PREFIX + "Odometry/CandidatePose", candidatePose);
        Logger.recordOutput(
                LOG_PREFIX + "Odometry/AcceptedPose",
                accepted ? new Pose2d[] {candidatePose} : new Pose2d[] {});
        Logger.recordOutput(
                LOG_PREFIX + "Odometry/RejectedPose",
                accepted ? new Pose2d[] {} : new Pose2d[] {candidatePose});
    }

    private void logVisionObservation(
            VisionPoseObservation observation,
            Pose2d candidatePose,
            boolean accepted,
            VisionObservationReason reason,
            double translationNSigma,
            double rotationNSigma) {
        Logger.recordOutput(LOG_PREFIX + "Vision/Accepted", accepted);
        Logger.recordOutput(LOG_PREFIX + "Vision/Reason", reason.name());
        Logger.recordOutput(LOG_PREFIX + "Vision/ObservationPose", observation.robotPose());
        Logger.recordOutput(LOG_PREFIX + "Vision/TimestampSeconds", observation.timestampSeconds());
        Logger.recordOutput(
                LOG_PREFIX + "Vision/AvgTagDistanceMeters", observation.avgTagDistance());
        Logger.recordOutput(LOG_PREFIX + "Vision/NumTagsUsed", observation.numTagsUsed());
        Logger.recordOutput(LOG_PREFIX + "Vision/LinearStdDev", observation.linearStdDev());
        Logger.recordOutput(LOG_PREFIX + "Vision/AngularStdDev", observation.angularStdDev());
        Logger.recordOutput(LOG_PREFIX + "Vision/TranslationNSigma", translationNSigma);
        Logger.recordOutput(LOG_PREFIX + "Vision/RotationNSigma", rotationNSigma);
        Logger.recordOutput(
                LOG_PREFIX + "Vision/CandidatePose",
                candidatePose != null ? candidatePose : Pose2d.kZero);
        Logger.recordOutput(
                LOG_PREFIX + "Vision/AcceptedPose",
                accepted && candidatePose != null ? new Pose2d[] {candidatePose} : new Pose2d[] {});
        Logger.recordOutput(
                LOG_PREFIX + "Vision/RejectedPose",
                !accepted
                        ? new Pose2d[] {
                            candidatePose != null ? candidatePose : observation.robotPose()
                        }
                        : new Pose2d[] {});
    }
}
