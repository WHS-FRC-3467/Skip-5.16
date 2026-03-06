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
 * <p>This class tracks the robot's module positions, integrates motion deltas into a {@link Pose2d}
 * estimate, and maintains a time-buffered history of poses for interpolation. It is intended to be
 * used by a higher-level {@link PoseEstimator} that fuses odometry with vision or other sensors. Do
 * not use this class on it's own, use {@link SwerveDriveOdometry}.
 */
@Accessors(fluent = true)
public class SwerveOdometry {
    /**
     * Represents a single odometry observation from the swerve drive.
     *
     * <p>{@code badWheels[i] == true} means module {@code i} should be ignored for this update.
     *
     * <p>The mask is carried per-observation so skid detection can be done at the same
     * rate/timestamps as odometry sampling (instead of at 50Hz in subsystem periodic).
     */
    @SuppressWarnings("ArrayRecordComponent")
    public static final record OdometryObservation(
            Time timestamp,
            SwerveModulePosition[] swervePositions,
            Optional<Rotation2d> gyroAngle,
            boolean[] badWheels) {

        /** Convenience constructor when you are not doing skid detection. */
        public OdometryObservation(
                Time timestamp,
                SwerveModulePosition[] swervePositions,
                Optional<Rotation2d> gyroAngle) {
            this(timestamp, swervePositions, gyroAngle, null);
        }
    }

    /** The swerve drive kinematics used to compute motion deltas. */
    private final SwerveDriveKinematics kinematics;

    /**
     * Module locations are required to solve chassis motion from a subset of wheels.
     *
     * <p>If module translations are not provided, we still run odometry normally, but we cannot
     * ignore skidding wheels because we can't build correct subset kinematics.
     */
    private final Translation2d[] moduleTranslationsOrNull;

    /**
     * Cache of kinematics objects for "good wheel subsets".
     *
     * <p>{@link SwerveDriveKinematics} is constructed from a fixed set of module translation
     * vectors. When we ignore a skidding wheel, the set of translations changes, so we need a
     * different kinematics object to correctly solve chassis motion from only the remaining wheels.
     *
     * <p>Odometry can run at 100 to 250 Hz. When skid is active, repeatedly constructing new
     * kinematics objects creates unnecessary allocation and garbage collection work. The possible
     * patterns are small and repeat often (usually none, or one wheel), so caching keeps the
     * runtime steady.
     *
     * <p>The key is based on the contents of the badWheels array. We do not use the boolean[]
     * directly because arrays are mutable and use reference identity for hash/equality.
     */
    private final Map<Integer, SwerveDriveKinematics> subsetKinematicsCache = new HashMap<>();

    // Max module count assumed 4 (FL, FR, BL, BR)
    private final Translation2d[] scratchTranslations;
    private final SwerveModulePosition[] scratchLast;
    private final SwerveModulePosition[] scratchCurrent;

    /** Stores recent odometry poses for interpolation purposes. */
    @Getter private final TimeInterpolatableBuffer<Pose2d> odometryBuffer;

    /** Tracks last known module positions for computing motion deltas. */
    private SwerveModulePosition[] lastModulePositions =
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };

    /** The offset from the gyro's 0 at boot and the last reset frame of rotation */
    private Rotation2d gyroOffset = Rotation2d.kZero;

    /** The most recent odometry-based robot pose. */
    @Getter private Pose2d odometryPose = Pose2d.kZero;

    /**
     * Constructs a new {@code SwerveOdometry}.
     *
     * @param kinematics the swerve drive kinematics for computing motion deltas
     * @param odometryBufferSize the duration for which odometry poses are buffered for
     *     interpolation
     */
    public SwerveOdometry(SwerveDriveKinematics kinematics, Time odometryBufferSize) {
        this(kinematics, null, odometryBufferSize);
    }

    /**
     * Constructs a new {@code SwerveOdometry} with module locations.
     *
     * <p>Provide {@code moduleTranslations} if you want to ignore skidding wheels. The translations
     * must be in the same module index order used everywhere else (FL, FR, BL, BR in your code).
     */
    public SwerveOdometry(
            SwerveDriveKinematics kinematics,
            Translation2d[] moduleTranslations,
            Time odometryBufferSize) {
        this.kinematics = kinematics;
        this.moduleTranslationsOrNull = moduleTranslations;
        odometryBuffer = TimeInterpolatableBuffer.createBuffer(odometryBufferSize.in(Seconds));

        if (moduleTranslationsOrNull != null) {
            int moduleCount = moduleTranslationsOrNull.length;
            scratchTranslations = new Translation2d[moduleCount];
            scratchLast = new SwerveModulePosition[moduleCount];
            scratchCurrent = new SwerveModulePosition[moduleCount];
        } else {
            scratchTranslations = null;
            scratchLast = null;
            scratchCurrent = null;
        }
    }

    /**
     * Adds an odometry observation to the integrator.
     *
     * @param observation an {@link OdometryObservation} containing module positions, timestamp, an
     *     optional gyro angle, and an optional skid mask
     */
    public void addOdometryObservation(OdometryObservation observation) {
        double timestampSeconds = observation.timestamp().in(Seconds);
        SwerveModulePosition[] currentPositions = observation.swervePositions();

        Twist2d twist =
                computeTwist(lastModulePositions, currentPositions, observation.badWheels());

        // Copy positions so callers can't accidentally mutate our history.
        lastModulePositions = copyPositions(currentPositions);

        // Integrate twist into odometry pose
        odometryPose = odometryPose.exp(twist);

        // If gyro angle is available, correct heading drift
        observation
                .gyroAngle()
                .ifPresent(
                        angle ->
                                odometryPose =
                                        new Pose2d(
                                                odometryPose.getTranslation(),
                                                angle.plus(gyroOffset)));

        // Store pose for later interpolation (e.g., vision sync)
        odometryBuffer.addSample(timestampSeconds, odometryPose);
    }

    /**
     * Adds an odometry observation to the integrator, ignoring everything except gyro.
     *
     * @param observation an {@link OdometryObservation} containing an optional gyro angle
     */
    public void addGyroObservation(OdometryObservation observation) {
        double timestampSeconds = observation.timestamp().in(Seconds);

        // If gyro angle is available, correct heading drift
        observation
                .gyroAngle()
                .ifPresent(
                        angle ->
                                odometryPose =
                                        new Pose2d(
                                                odometryPose.getTranslation(),
                                                angle.plus(gyroOffset)));

        // Store pose for later interpolation (e.g., vision sync)
        odometryBuffer.addSample(timestampSeconds, odometryPose);
    }

    private Twist2d computeTwist(
            SwerveModulePosition[] last, SwerveModulePosition[] current, boolean[] badWheels) {

        if (!canUseBadWheelMask(badWheels)) {
            return kinematics.toTwist2d(last, current);
        }

        int moduleCount = current.length;

        // Build bitmask and count good modules
        int mask = 0;
        int goodCount = 0;

        for (int i = 0; i < moduleCount; i++) {
            if (badWheels[i]) {
                mask |= (1 << i);
            } else {
                goodCount++;
            }
        }

        // If too few good wheels, fall back
        if (goodCount < 3) {
            return kinematics.toTwist2d(last, current);
        }

        // Fill scratch arrays
        int j = 0;
        for (int i = 0; i < moduleCount; i++) {
            if ((mask & (1 << i)) != 0) {
                continue;
            }

            scratchTranslations[j] = moduleTranslationsOrNull[i];
            scratchLast[j] = last[i];
            scratchCurrent[j] = current[i];
            j++;
        }

        SwerveDriveKinematics subsetKinematics = getOrCreateSubsetKinematics(mask, goodCount);

        return subsetKinematics.toTwist2d(
                Arrays.copyOf(scratchLast, goodCount), Arrays.copyOf(scratchCurrent, goodCount));
    }

    private boolean canUseBadWheelMask(boolean[] badWheels) {
        if (moduleTranslationsOrNull == null) {
            return false;
        }
        if (badWheels == null) {
            return false;
        }
        return badWheels.length == moduleTranslationsOrNull.length;
    }

    private SwerveDriveKinematics getOrCreateSubsetKinematics(int mask, int goodCount) {

        return subsetKinematicsCache.computeIfAbsent(
                mask,
                m -> {
                    Translation2d[] translations = new Translation2d[goodCount];

                    int j = 0;
                    for (int i = 0; i < moduleTranslationsOrNull.length; i++) {
                        if ((m & (1 << i)) == 0) {
                            translations[j++] = moduleTranslationsOrNull[i];
                        }
                    }

                    return new SwerveDriveKinematics(translations);
                });
    }

    private static SwerveModulePosition[] copyPositions(SwerveModulePosition[] positions) {
        SwerveModulePosition[] copy = new SwerveModulePosition[positions.length];
        System.arraycopy(positions, 0, copy, 0, positions.length);
        return copy;
    }

    /**
     * Resets the internal odometry state to a known pose.
     *
     * @param pose the new field-relative {@link Pose2d} representing the robot's known position
     */
    public void resetPose(Pose2d pose) {
        gyroOffset = pose.getRotation().minus(odometryPose.getRotation().minus(gyroOffset));
        odometryPose = pose;
        odometryBuffer.clear();
    }
}
