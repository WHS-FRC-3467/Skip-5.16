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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Time;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import lombok.Getter;
import lombok.experimental.Accessors;

/**
 * Integrates swerve-drive odometry observations to maintain a continuous estimate of the robot's
 * field-relative pose over time. This is essentially a {@link SwerveDriveOdometry} with more
 * outputs to be utilized in a {@link PoseEstimator}.
 *
 * <p>
 * This class tracks the robot's module positions, integrates motion deltas into a {@link Pose2d}
 * estimate, and maintains a time-buffered history of poses for interpolation. It is intended to be
 * used by a higher-level {@link PoseEstimator} that fuses odometry with vision or other sensors. Do
 * not use this class on it's own, use {@link SwerveDriveOdometry}.
 */
@Accessors(fluent = true)
public class SwerveOdometry {
    /**
     * Represents a single odometry observation from the swerve drive.
     *
     * <p>
     * {@code badWheels[i] == true} means module {@code i} should be ignored for this update.
     *
     * <p>
     * Decision: - The mask is carried per-observation so skid detection can be done at the same
     * rate/timestamps as odometry sampling (instead of at 50Hz in subsystem periodic).
     */
    @SuppressWarnings("ArrayRecordComponent")
    public static final record OdometryObservation(
        Time timestamp,
        SwerveModulePosition[] swervePositions,
        Optional<Rotation2d> gyroAngle,
        boolean[] badWheels) {

        /** Convenience constructor when you are not doing skid detection. */
        public OdometryObservation(Time timestamp,
            SwerveModulePosition[] swervePositions,
            Optional<Rotation2d> gyroAngle)
        {
            this(timestamp, swervePositions, gyroAngle, null);
        }
    }

    /** The swerve drive kinematics used to compute motion deltas. */
    private final SwerveDriveKinematics kinematics;

    /**
     * Module locations are required to solve chassis motion from a *subset* of wheels.
     *
     * Decision: - If module translations are not provided, we still run odometry normally, but we
     * cannot ignore skidding wheels because we can't build correct subset kinematics.
     */
    private final Translation2d[] moduleTranslationsOrNull;

    /**
     * Cache subset kinematics to avoid rebuilding objects when the same badWheels pattern repeats.
     *
     * Decision: - We keep the external API as boolean[] for clarity, but cache with an immutable
     * key that stores a defensive copy of the boolean[].
     */
    private final Map<BadWheelsKey, SwerveDriveKinematics> subsetKinematicsCache = new HashMap<>();

    /** Stores recent odometry poses for interpolation purposes. */
    @Getter
    private final TimeInterpolatableBuffer<Pose2d> odometryBuffer;

    /** Tracks last known module positions for computing motion deltas. */
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };

    /** The offset from the gyro's 0 at boot and the last reset frame of rotation */
    private Rotation2d gyroOffset = Rotation2d.kZero;

    /** The most recent odometry-based robot pose. */
    @Getter
    private Pose2d odometryPose = Pose2d.kZero;

    /**
     * Constructs a new {@code SwerveOdometry}.
     *
     * @param kinematics the swerve drive kinematics for computing motion deltas
     * @param odometryBufferSize the duration for which odometry poses are buffered for
     *        interpolation
     */
    public SwerveOdometry(SwerveDriveKinematics kinematics, Time odometryBufferSize)
    {
        this(kinematics, null, odometryBufferSize);
    }

    /**
     * Constructs a new {@code SwerveOdometry} with module locations.
     *
     * <p>
     * Provide {@code moduleTranslations} if you want to ignore skidding wheels. The translations
     * must be in the same module index order used everywhere else (FL, FR, BL, BR in your code).
     */
    public SwerveOdometry(
        SwerveDriveKinematics kinematics,
        Translation2d[] moduleTranslations,
        Time odometryBufferSize)
    {
        this.kinematics = kinematics;
        this.moduleTranslationsOrNull = moduleTranslations;
        odometryBuffer = TimeInterpolatableBuffer.createBuffer(odometryBufferSize.in(Seconds));
    }

    /**
     * Adds an odometry observation to the integrator.
     *
     * @param observation an {@link OdometryObservation} containing module positions, timestamp, an
     *        optional gyro angle, and an optional skid mask
     */
    public void addOdometryObservation(OdometryObservation observation)
    {
        double timestampSeconds = observation.timestamp().in(Seconds);
        SwerveModulePosition[] currentPositions = observation.swervePositions();

        Twist2d twist =
            computeTwist(lastModulePositions, currentPositions, observation.badWheels());

        // Copy positions so callers can't accidentally mutate our history.
        lastModulePositions = copyPositions(currentPositions);

        // Integrate twist into odometry pose
        odometryPose = odometryPose.exp(twist);

        // If gyro angle is available, correct heading drift
        observation.gyroAngle().ifPresent(
            angle -> odometryPose =
                new Pose2d(odometryPose.getTranslation(), angle.plus(gyroOffset)));

        // Store pose for later interpolation (e.g., vision sync)
        odometryBuffer.addSample(timestampSeconds, odometryPose);
    }

    private Twist2d computeTwist(
        SwerveModulePosition[] last,
        SwerveModulePosition[] current,
        boolean[] badWheels)
    {
        if (!canUseBadWheelMask(badWheels)) {
            return kinematics.toTwist2d(last, current);
        }

        int moduleCount = current.length;
        int goodCount = 0;
        for (int i = 0; i < moduleCount; i++) {
            if (!badWheels[i]) {
                goodCount++;
            }
        }

        // Decision:
        // - With fewer than 3 good modules, the solve can get unstable (especially omega).
        // - Falling back to all wheels is usually less noisy than trying to integrate from 1–2
        // modules.
        if (goodCount < 3) {
            return kinematics.toTwist2d(last, current);
        }

        Translation2d[] subsetTranslations = new Translation2d[goodCount];
        SwerveModulePosition[] subsetLast = new SwerveModulePosition[goodCount];
        SwerveModulePosition[] subsetCurrent = new SwerveModulePosition[goodCount];

        int j = 0;
        for (int i = 0; i < moduleCount; i++) {
            if (badWheels[i]) {
                continue;
            }
            subsetTranslations[j] = moduleTranslationsOrNull[i];
            subsetLast[j] = last[i];
            subsetCurrent[j] = current[i];
            j++;
        }

        SwerveDriveKinematics subsetKinematics =
            getOrCreateSubsetKinematics(badWheels, subsetTranslations);

        return subsetKinematics.toTwist2d(subsetLast, subsetCurrent);
    }

    private boolean canUseBadWheelMask(boolean[] badWheels)
    {
        if (moduleTranslationsOrNull == null) {
            return false;
        }
        if (badWheels == null) {
            return false;
        }
        return badWheels.length == moduleTranslationsOrNull.length;
    }

    private SwerveDriveKinematics getOrCreateSubsetKinematics(
        boolean[] badWheels,
        Translation2d[] subsetTranslations)
    {
        BadWheelsKey key = new BadWheelsKey(badWheels);
        return subsetKinematicsCache.computeIfAbsent(key,
            k -> new SwerveDriveKinematics(subsetTranslations));
    }

    /**
     * Immutable key for caching subset kinematics.
     *
     * Decision: - We defensively copy the boolean[] so accidental mutations in calling code won't
     * poison the cache. - Hash/equality are based on the contents of the boolean[].
     */
    private static final class BadWheelsKey {
        private final boolean[] badWheels;
        private final int hash;

        private BadWheelsKey(boolean[] badWheels)
        {
            this.badWheels = badWheels.clone();
            this.hash = Arrays.hashCode(this.badWheels);
        }

        @Override
        public boolean equals(Object obj)
        {
            if (this == obj) {
                return true;
            }
            if (!(obj instanceof BadWheelsKey other)) {
                return false;
            }
            return Arrays.equals(this.badWheels, other.badWheels);
        }

        @Override
        public int hashCode()
        {
            return hash;
        }
    }

    private static SwerveModulePosition[] copyPositions(SwerveModulePosition[] positions)
    {
        SwerveModulePosition[] copy = new SwerveModulePosition[positions.length];
        System.arraycopy(positions, 0, copy, 0, positions.length);
        return copy;
    }

    /**
     * Resets the internal odometry state to a known pose.
     *
     * @param pose the new field-relative {@link Pose2d} representing the robot's known position
     */
    public void resetPose(Pose2d pose)
    {
        gyroOffset = pose.getRotation().minus(odometryPose.getRotation().minus(gyroOffset));
        odometryPose = pose;
        odometryBuffer.clear();
    }
}
