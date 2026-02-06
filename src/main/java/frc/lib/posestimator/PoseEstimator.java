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
import java.util.Optional;
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

    private final SwerveOdometry odometry;

    /** Base odometry variances */
    private final double[] odometryVariances;

    private double odometryStdDevMultiplierLinear = 1.0;
    private double odometryStdDevMultiplierAngular = 1.0;

    @Getter
    private Pose2d estimatedPose = Pose2d.kZero;

    public PoseEstimator(
        SwerveDriveKinematics kinematics,
        Time odometryBufferSize,
        double linearOdometryStdDev,
        double angularOdometryStdDev)
    {
        this(kinematics, null, odometryBufferSize, linearOdometryStdDev, angularOdometryStdDev);
    }

    /**
     * If module translations are provided, odometry can ignore skidding wheels using the
     * {@code badWheels} array in {@link OdometryObservation}.
     */
    public PoseEstimator(
        SwerveDriveKinematics kinematics,
        Translation2d[] moduleTranslations,
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

        odometry = new SwerveOdometry(kinematics, moduleTranslations, odometryBufferSize);
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

    public void addOdometryObservation(OdometryObservation observation)
    {
        updateOdometryStdDevMultipliersFromSkid(observation.badWheels());

        Pose2d lastOdometryPose = odometry.odometryPose();
        odometry.addOdometryObservation(observation);
        Pose2d newOdometryPose = odometry.odometryPose();

        Twist2d twist = lastOdometryPose.log(newOdometryPose);

        estimatedPose = estimatedPose.exp(twist);
    }

    private void updateOdometryStdDevMultipliersFromSkid(boolean[] badWheels)
    {
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

    public Optional<Pose2d> getPoseAtTime(double timestampSeconds)
    {
        return getPoseDeltaThenToNow(timestampSeconds)
            .map(thenToNow -> estimatedPose.plus(thenToNow.inverse()));
    }

    public void addVisionObservation(VisionPoseObservation observation)
    {
        Transform2d poseDeltaThenToNow =
            getPoseDeltaThenToNow(observation.timestampSeconds).orElse(null);
        if (poseDeltaThenToNow == null) {
            return;
        }

        Pose2d oldPose = estimatedPose.plus(poseDeltaThenToNow.inverse());
        Pose2d newVisionPose = observation.robotPose;

        double visionLinearVariance = observation.linearStdDev * observation.linearStdDev;
        double visionAngularVariance = observation.angularStdDev * observation.angularStdDev;

        double[] visionVariances = {
                visionLinearVariance, // X axis
                visionLinearVariance, // Y axis
                visionAngularVariance // Rotation
        };

        double linearVarianceMultiplier =
            odometryStdDevMultiplierLinear * odometryStdDevMultiplierLinear;
        double angularVarianceMultiplier =
            odometryStdDevMultiplierAngular * odometryStdDevMultiplierAngular;

        double[] effectiveOdometryVariances = {
                odometryVariances[0] * linearVarianceMultiplier, // X axis
                odometryVariances[1] * linearVarianceMultiplier, // Y axis
                odometryVariances[2] * angularVarianceMultiplier // Rotation
        };

        Matrix<N3, N3> visionKalmanGain = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; ++row) {
            double odometryVariance = effectiveOdometryVariances[row];
            if (odometryVariance == 0.0) {
                visionKalmanGain.set(row, row, 0.0);
            } else {
                visionKalmanGain.set(row, row, odometryVariance
                    / (odometryVariance + Math.sqrt(odometryVariance * visionVariances[row])));
            }
        }

        Transform2d unscaledVisionCorrection =
            new Transform2d(oldPose, newVisionPose);

        var scaledVisionCorrectionVector = visionKalmanGain.times(
            VecBuilder.fill(
                unscaledVisionCorrection.getX(),
                unscaledVisionCorrection.getY(),
                unscaledVisionCorrection.getRotation().getRadians()));

        Transform2d scaledVisionCorrection = new Transform2d(
            scaledVisionCorrectionVector.get(0, 0),
            scaledVisionCorrectionVector.get(1, 0),
            Rotation2d.fromRadians(scaledVisionCorrectionVector.get(2, 0)));

        estimatedPose = oldPose
            .transformBy(scaledVisionCorrection)
            .transformBy(poseDeltaThenToNow);
    }

    public void resetPose(Pose2d pose)
    {
        odometry.resetPose(pose);
        estimatedPose = pose;

        odometryStdDevMultiplierLinear = 1.0;
        odometryStdDevMultiplierAngular = 1.0;
    }
}
