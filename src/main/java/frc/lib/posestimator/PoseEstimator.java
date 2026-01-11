/*
 * Copyright (C) 2025 Windham Windup
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

import java.util.Optional;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import frc.lib.posestimator.SwerveOdometry.OdometryObservation;
import lombok.Getter;
import lombok.experimental.Accessors;

@Accessors(fluent = true)
public class PoseEstimator {
    public static record VisionPoseObservation(
        double timestampSeconds,
        Pose2d robotPose,
        double linearStdDev,
        double angularStdDev) {
    }

    private static final double DEFAULT_ODOMETRY_BUFFER_SIZE_SECONDS = 2;

    private final SwerveOdometry odometry;

    private final double[] odometryVariances;

    @Getter
    private Pose2d estimatedPose = Pose2d.kZero;

    public PoseEstimator(
        SwerveDriveKinematics kinematics,
        Time odometryBufferSize,
        double linearOdometryStdDev,
        double angularOdometryStdDev)
    {
        double linearOdometryVariance = Math.pow(linearOdometryStdDev, 2);
        double angularOdometryVariance = Math.pow(angularOdometryStdDev, 2);

        odometryVariances = new double[] {
                linearOdometryVariance, // X axis
                linearOdometryVariance, // Y axis
                angularOdometryVariance // Rotation
        };

        odometry = new SwerveOdometry(kinematics, odometryBufferSize);
    }

    public PoseEstimator(
        SwerveDriveKinematics kinematics,
        double linearOdometryStdDev,
        double angularOdometryStdDev)
    {
        this(
            kinematics,
            Seconds.of(DEFAULT_ODOMETRY_BUFFER_SIZE_SECONDS),
            linearOdometryStdDev,
            angularOdometryStdDev);
    }

    public Pose2d odometryPose()
    {
        return odometry.odometryPose();
    }

    /**
     * Adds a new odometry observation to the pose estimator and updates the estimated pose.
     * 
     * This method retrieves the last odometry pose, applies the new odometry observation, and
     * calculates the change in pose (twist) between the last and new odometry poses. The estimated
     * pose is then updated by applying the calculated twist.
     * 
     * @param observation The new odometry observation to be added. This observation typically
     *        contains information about the robot's movement such as displacement and rotation.
     */
    public void addOdometryObservation(OdometryObservation observation)
    {
        Pose2d lastOdometryPose = odometry.odometryPose();
        odometry.addOdometryObservation(observation);
        Pose2d newOdometryPose = odometry.odometryPose();

        Twist2d twist = lastOdometryPose.log(newOdometryPose);

        estimatedPose = estimatedPose.exp(twist);
    }

    /**
     * Calculates the change in pose (delta) between the odometry pose at a given timestamp and the
     * current odometry pose. This is mainly used for latency compensation for vision.
     *
     * @param timestampSeconds The timestamp (in seconds) for which to retrieve the odometry pose.
     * @return An {@link Optional} containing the {@link Transform2d} representing the pose delta if
     *         the odometry pose at the given timestamp exists; otherwise, an empty
     *         {@link Optional}.
     */
    public Optional<Transform2d> getPoseDeltaThenToNow(double timestampSeconds)
    {
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
     * This method calculates the pose at the given timestamp by determining the pose delta from the
     * current estimated pose and applying the inverse of that delta to the current pose.
     *
     * @param timestampSeconds The timestamp (in seconds) for which the pose is requested.
     * @return An {@code Optional<Pose2d>} containing the estimated pose at the specified timestamp,
     *         or an empty {@code Optional} if the pose cannot be determined.
     */
    public Optional<Pose2d> getPoseAtTime(double timestampSeconds)
    {
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
    public void addVisionObservation(VisionPoseObservation observation)
    {
        // Attempt to get heading. Fails if the odometer has not recorded
        // a measurement near this timestamp
        Transform2d poseDeltaThenToNow =
            getPoseDeltaThenToNow(observation.timestampSeconds).orElse(null);
        if (poseDeltaThenToNow == null) {
            return;
        }

        Pose2d oldPose = estimatedPose.plus(poseDeltaThenToNow.inverse());
        Pose2d newVisionPose = observation.robotPose;

        double visionLinearVariance = observation.linearStdDev * observation.linearStdDev;
        double visionAngularVariance = observation.angularStdDev * observation.angularStdDev;

        // Solve Kalman gain matrix given observation standard deviations
        // Logic is copied from:
        // https://github.com/wpilibsuite/allwpilib/blob/b8d6bc2eb1b6cea10d1179939114d041945e172a/wpimath/src/main/java/edu/wpi/first/math/estimator/PoseEstimator.java#L93-L109
        double[] visionVariances = {
                visionLinearVariance, // X axis
                visionLinearVariance, // Y axis
                visionAngularVariance}; // Rotation

        Matrix<N3, N3> visionKalmanGain = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; ++row) {
            double odometryVariance = odometryVariances[row];
            if (odometryVariance == 0.0) {
                visionKalmanGain.set(row, row, 0.0);
            } else {
                visionKalmanGain.set(row, row, odometryVariance
                    / (odometryVariance + Math.sqrt(odometryVariance * visionVariances[row])));
            }
        }

        // Transform between our best estimated pose at the time the frame was captured to where the
        // camera is saying we should be, unscaled (without any Kalman gain applied)
        // Logic is copied from:
        // https://github.com/wpilibsuite/allwpilib/blob/b8d6bc2eb1b6cea10d1179939114d041945e172a/wpimath/src/main/java/edu/wpi/first/math/estimator/PoseEstimator.java#L276-L292
        Transform2d unscaledVisionCorrection =
            new Transform2d(oldPose, newVisionPose);

        // Scale the vision correction by the Kalman gain
        var scaledVisionCorrectionVector = visionKalmanGain.times(
            VecBuilder.fill(
                unscaledVisionCorrection.getX(),
                unscaledVisionCorrection.getY(),
                unscaledVisionCorrection.getRotation().getRadians()));

        // Convert to Transform2d
        Transform2d scaledVisionCorrection = new Transform2d(
            scaledVisionCorrectionVector.get(0, 0),
            scaledVisionCorrectionVector.get(1, 0),
            Rotation2d.fromRadians(scaledVisionCorrectionVector.get(2, 0)));

        estimatedPose = oldPose
            .transformBy(scaledVisionCorrection) // Adjust by the correction
            .transformBy(poseDeltaThenToNow); // Bring back to present time (latency comp)
    }

    /**
     * Resets the pose estimator to a known field-relative pose.
     *
     * <p>
     * This method should be called when the robot's absolute position on the field is known — for
     * example, at the start of autonomous or after a vision-based correction. It reinitializes both
     * the odometry integrator and the estimator’s internal pose state to the specified pose.
     *
     * @param pose the known {@link Pose2d} representing the robot’s field-relative position
     */
    public void resetPose(Pose2d pose)
    {
        odometry.resetPose(pose);
        estimatedPose = pose;
    }
}
