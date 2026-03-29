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

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.devices.CamCamCoproc;
import frc.lib.devices.ObjectDetection.DetectionFrame;
import frc.lib.devices.ObjectDetection.DetectionTarget;
import frc.lib.posestimator.PoseEstimator.VisionPoseObservation;
import frc.robot.RobotState;
import frc.robot.generated.flatbuffers.ConfiguredData;
import frc.robot.generated.flatbuffers.PoseObservation;
import frc.robot.generated.flatbuffers.PoseObservations;
import frc.robot.generated.flatbuffers.YOLOObservation;
import frc.robot.generated.flatbuffers.YOLOObservations;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.objectdetector.ObjectDetector;

import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Consumes CamCam packets for robot localization and object-detection handoff. */
public class VisionSubsystem extends SubsystemBase {
    public static final String LOG_PREFIX = "VisionProcessor/";
    public static final double LINEAR_STDDEV_BASELINE = 0.03;
    public static final double ANGULAR_STDDEV_BASELINE = 0.05;
    public static final double MAX_Z_METERS = 0.75;
    public static final double MAX_DISTANCE_METERS = 4.0;
    public static final int MAX_UNREAD_RESULTS = 5;
    public static final double MAX_AMBIGUITY = 0.2;
    public static final double MIN_STDDEV_DISTANCE_METERS = 0.5;
    public static final double STDDEV_AMBIGUITY_WEIGHT = 2.5;
    public static final double STDDEV_TAGCOUNT_EXPONENT = 0.5;
    public static final double STDDEV_FACTOR_MIN = 0.35;
    public static final double STDDEV_FACTOR_MAX = 8.0;

    private final RobotState robotState = RobotState.getInstance();
    private final CamCamCoproc[] coprocessors;
    private final ObjectDetector objectDetector;

    /**
     * Creates a vision subsystem that only consumes AprilTag localization packets.
     *
     * @param coprocessors CamCam coprocessors to poll each cycle
     */
    public VisionSubsystem(CamCamCoproc... coprocessors) {
        this(null, coprocessors);
    }

    /**
     * Creates a vision subsystem that forwards AprilTag and object-detection data.
     *
     * @param objectDetector optional object detector to receive the newest object frame
     * @param coprocessors CamCam coprocessors to poll each cycle
     */
    public VisionSubsystem(ObjectDetector objectDetector, CamCamCoproc... coprocessors) {
        this.objectDetector = objectDetector;
        this.coprocessors = Arrays.copyOf(coprocessors, coprocessors.length, CamCamCoproc[].class);
    }

    @Override
    public void periodic() {
        DetectionFrame latestObjectFrame = null;

        for (CamCamCoproc coprocessor : coprocessors) {
            ConfiguredData[] unreadResults = coprocessor.getUnreadResults().orElse(null);
            if (unreadResults == null) {
                continue;
            }
            if (unreadResults.length > MAX_UNREAD_RESULTS) {
                unreadResults =
                        Arrays.copyOfRange(
                                unreadResults,
                                unreadResults.length - MAX_UNREAD_RESULTS,
                                unreadResults.length);
            }

            ArrayList<Pose2d> acceptedPoses = new ArrayList<>();
            ArrayList<Pose2d> rejectedPoses = new ArrayList<>();
            for (ConfiguredData result : unreadResults) {
                processPoseObservations(result.poseObservations(), acceptedPoses, rejectedPoses);
                latestObjectFrame =
                        newestFrame(
                                latestObjectFrame,
                                extractLatestObjectFrame(result.objectDetectionObservations()));
            }

            String cameraLogPrefix = LOG_PREFIX + coprocessor.getInstanceName() + "/";
            Logger.recordOutput(
                    cameraLogPrefix + "Poses/Accepted", acceptedPoses.toArray(Pose2d[]::new));
            Logger.recordOutput(
                    cameraLogPrefix + "Poses/Rejected", rejectedPoses.toArray(Pose2d[]::new));
        }

        if (objectDetector != null && latestObjectFrame != null) {
            objectDetector.acceptLatestFrame(latestObjectFrame);
        }
    }

    private void processPoseObservations(
            PoseObservations poseObservations,
            List<Pose2d> acceptedPoses,
            List<Pose2d> rejectedPoses) {
        if (poseObservations == null) {
            return;
        }

        for (int i = 0; i < poseObservations.observationsLength(); i++) {
            PoseObservation observation = poseObservations.observations(i);
            Pose3d pose = toPose3d(observation);
            Pose2d pose2d = pose.toPose2d();

            if (!preFilter(observation) || !postFilter(pose)) {
                rejectedPoses.add(pose2d);
                continue;
            }

            robotState.addVisionObservation(toVisionPoseObservation(observation, pose2d));
            acceptedPoses.add(pose2d);
        }
    }

    private DetectionFrame extractLatestObjectFrame(YOLOObservations observations) {
        if (observations == null || observations.observationsLength() == 0) {
            return null;
        }

        DetectionFrame latestFrame = null;
        for (int i = 0; i < observations.observationsLength(); i++) {
            latestFrame = newestFrame(latestFrame, toDetectionFrame(observations.observations(i)));
        }
        return latestFrame;
    }

    private DetectionFrame newestFrame(DetectionFrame current, DetectionFrame candidate) {
        if (candidate == null) {
            return current;
        }
        if (current == null || candidate.timestampSeconds() >= current.timestampSeconds()) {
            return candidate;
        }
        return current;
    }

    /**
     * Applies coarse quality gating before attempting to fuse a pose observation.
     *
     * @param observation candidate vision observation from CamCam
     * @return {@code true} when the observation is worth fusing
     */
    static boolean preFilter(PoseObservation observation) {
        if (observation.averageDistance() > MAX_DISTANCE_METERS) {
            return false;
        }

        return estimateNumTagsUsed(observation) != 1
                || observation.ambiguity() < 0.0
                || observation.ambiguity() <= MAX_AMBIGUITY;
    }

    /**
     * Validates that a pose observation falls within reasonable field and robot attitude bounds.
     *
     * @param pose candidate robot pose from vision
     * @return {@code true} when the pose should be accepted for localization
     */
    static boolean postFilter(Pose3d pose) {
        double z = pose.getZ();
        double pitch = Math.abs(pose.getRotation().getY());
        double roll = Math.abs(pose.getRotation().getX());
        Pose2d pose2d = pose.toPose2d();
        return !(z > MAX_Z_METERS
                || !RobotState.getInstance().isPoseWithinField(pose2d)
                || pitch > DriveConstants.ANGLED_TOLERANCE.in(Radians)
                || roll > DriveConstants.ANGLED_TOLERANCE.in(Radians));
    }

    /**
     * Converts a FlatBuffer pose observation into WPILib pose types.
     *
     * @param observation serialized pose observation from CamCam
     * @return equivalent {@link Pose3d}
     */
    static Pose3d toPose3d(PoseObservation observation) {
        var pose = observation.pose();
        var translation = pose.translation();
        var rotation = pose.rotation();
        return new Pose3d(
                translation.x(),
                translation.y(),
                translation.z(),
                new Rotation3d(
                        new Quaternion(rotation.w(), rotation.x(), rotation.y(), rotation.z())));
    }

    /**
     * Builds the estimator-facing observation record for one accepted CamCam pose.
     *
     * @param observation serialized pose observation from CamCam
     * @param pose2d field-relative pose derived from {@code observation}
     * @return estimator-ready vision observation with computed standard deviations
     */
    static VisionPoseObservation toVisionPoseObservation(
            PoseObservation observation, Pose2d pose2d) {
        double stdDevFactor = computeStdDevFactor(observation);
        return new VisionPoseObservation(
                observation.timestamp(),
                pose2d,
                observation.averageDistance(),
                estimateNumTagsUsed(observation),
                LINEAR_STDDEV_BASELINE * stdDevFactor,
                ANGULAR_STDDEV_BASELINE * stdDevFactor);
    }

    /**
     * Computes the standard-deviation scale factor used for this vision observation.
     *
     * @param observation serialized pose observation from CamCam
     * @return clamped scale factor applied to the linear and angular standard deviations
     */
    static double computeStdDevFactor(PoseObservation observation) {
        double distanceMeters = Math.max(observation.averageDistance(), MIN_STDDEV_DISTANCE_METERS);
        int tagCount = Math.max(1, estimateNumTagsUsed(observation));
        double ambiguity = observation.ambiguity();
        if (ambiguity < 0.0) {
            ambiguity = 0.0;
        }

        double distanceFactor = Math.pow(distanceMeters, 2.0);
        double tagFactor = 1.0 / Math.pow(tagCount, STDDEV_TAGCOUNT_EXPONENT);
        double ambiguityFactor =
                1.0 + STDDEV_AMBIGUITY_WEIGHT * Math.pow(ambiguity / MAX_AMBIGUITY, 2.0);
        return MathUtil.clamp(
                distanceFactor * tagFactor * ambiguityFactor, STDDEV_FACTOR_MIN, STDDEV_FACTOR_MAX);
    }

    /**
     * Estimates how many tags contributed to a pose solve based on the metadata CamCam provides.
     *
     * @param observation serialized pose observation from CamCam
     * @return estimated number of tags used in the solve
     */
    static int estimateNumTagsUsed(PoseObservation observation) {
        return Math.max(1, observation.numTags());
    }

    /**
     * Converts a serialized YOLO observation into the object-detection frame used by the subsystem.
     *
     * @param observation serialized object-detection observation from CamCam
     * @return object-detection frame containing all reported targets
     */
    static DetectionFrame toDetectionFrame(YOLOObservation observation) {
        ArrayList<DetectionTarget> targets = new ArrayList<>(observation.objectsLength());
        for (int i = 0; i < observation.objectsLength(); i++) {
            var object = observation.objects(i);
            targets.add(
                    new DetectionTarget(object.id(), object.pitch(), object.yaw(), object.area()));
        }
        return new DetectionFrame(observation.timestamp(), targets);
    }
}
