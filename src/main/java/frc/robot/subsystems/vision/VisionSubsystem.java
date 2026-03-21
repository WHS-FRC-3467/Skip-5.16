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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.devices.AprilTagCamera;
import frc.lib.posestimator.PoseEstimator.VisionPoseObservation;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveConstants;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

/**
 * The {@code VisionSubsystem} manages all vision-related processing for the robot.
 *
 * <p>It uses one or more {@link AprilTagCamera}s to detect field elements and estimate the robot's
 * pose on the field. Observations are processed through the MultiTagOnCoproc vision processor, with
 * a fallback to LowestAmbiguity if necessary. Valid observations are added to {@link RobotState}
 * for use in localization and navigation.
 *
 * <p>The subsystem periodically polls cameras for new results and logs both accepted and rejected
 * vision observations.
 */
public class VisionSubsystem extends SubsystemBase {

    public static final String LOG_PREFIX = "VisionProcessor/";

    /** Baseline linear standard deviation used for vision observations. */
    public static final double LINEAR_STDDEV_BASELINE = 0.03;

    /** Baseline angular standard deviation used for vision observations. */
    public static final double ANGULAR_STDDEV_BASELINE = 0.05;

    /** Maximum allowable height (Z-axis) of a detected pose to be considered valid. */
    public static final double MAX_Z_METERS = 0.75;

    /** Maximum allowable distance from a target to be considered valid. */
    public static final double MAX_DISTANCE_METERS = 4.0;

    /** Maximum ambiguity ratio allowed in a result */
    public static final double MAX_AMBIGUITY = 0.2;

    /** Minimum distance used when computing vision standard deviation scaling. */
    public static final double MIN_STDDEV_DISTANCE_METERS = 0.5;

    /** Maximum unread results processed per camera cycle. */
    public static final int MAX_UNREAD_RESULTS = 5;

    /** Weight applied to ambiguity when scaling vision standard deviation. */
    public static final double STDDEV_AMBIGUITY_WEIGHT = 2.5;

    /** Exponent for tag count influence on standard deviation scaling. */
    public static final double STDDEV_TAGCOUNT_EXPONENT = 0.5;

    /** Clamp range for the computed standard deviation factor. */
    public static final double STDDEV_FACTOR_MIN = 0.35;

    public static final double STDDEV_FACTOR_MAX = 8.0;

    /** Width of the field in meters. */
    public static final double FIELD_WIDTH = FieldConstants.FIELD_WIDTH;

    /** Length of the field in meters. */
    public static final double FIELD_LENGTH = FieldConstants.FIELD_LENGTH;

    public static final record VisionPoseRecord(
            Pose3d pose, List<Integer> tagsUsed, double averageDistanceMeters) {}

    /**
     * Quickly checks whether a {@link PhotonPipelineResult} is likely to be useful before full
     * processing.
     *
     * <p>Rejects results with no targets, ambiguous poses above 0.2, or targets farther than 4
     * meters.
     *
     * @param result the pipeline result to pre-filter
     * @return {@code true} if the result passes preliminary checks, {@code false} otherwise
     */
    public static boolean preFilter(PhotonPipelineResult result) {
        if (!result.hasTargets()) {
            return false;
        }

        if (result.getMultiTagResult().isPresent()) {
            if (result.getTargets().stream()
                            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                            .average()
                            .getAsDouble()
                    > MAX_DISTANCE_METERS) {
                return false;
            }

            return true;
        }

        PhotonTrackedTarget bestTarget = result.getBestTarget();
        if (bestTarget.getBestCameraToTarget().getTranslation().getNorm() > MAX_DISTANCE_METERS) {
            return false;
        }

        return bestTarget.getPoseAmbiguity() <= MAX_AMBIGUITY;
    }

    /**
     * Checks whether a given {@link Pose3d} is valid on the field.
     *
     * <p>A pose is considered valid if it is within field boundaries and below {@link
     * #MAX_Z_METERS}.
     *
     * @param pose the pose to validate
     * @return {@code true} if the pose is valid, {@code false} otherwise
     */
    public static boolean postFilter(Pose3d pose) {
        double z = pose.getZ();
        double pitch = Math.abs(pose.getRotation().getY());
        double roll = Math.abs(pose.getRotation().getX());
        Pose2d pose2d = pose.toPose2d();
        return !(z > MAX_Z_METERS
                || !RobotState.getInstance().isPoseWithinField(pose2d)
                || pitch > DriveConstants.ANGLED_TOLERANCE.in(Radians)
                || roll > DriveConstants.ANGLED_TOLERANCE.in(Radians));
    }

    private final RobotState robotState = RobotState.getInstance();
    private final AprilTagCamera[] cameras;
    private final PhotonPoseEstimator[] poseEstimators;

    /**
     * Constructs a new {@code VisionSubsystem} with the specified cameras.
     *
     * @param cameras the cameras to use for vision processing
     */
    public VisionSubsystem(AprilTagCamera... cameras) {
        this.cameras = cameras;
        this.poseEstimators = new PhotonPoseEstimator[cameras.length];
        for (int i = 0; i < cameras.length; i++) {
            this.poseEstimators[i] =
                    new PhotonPoseEstimator(
                            AprilTagLayoutType.OFFICIAL.getLayout(),
                            cameras[i].getProperties().robotToCamera());
        }
    }

    /**
     * Periodically processes vision results from all cameras. Filters, validates, and adds pose
     * observations to RobotState for localization.
     */
    @Override
    public void periodic() {
        for (int c = 0; c < cameras.length; c++) {
            AprilTagCamera camera = cameras[c];
            PhotonPoseEstimator poseEstimator = poseEstimators[c];
            String cameraLogPrefix = LOG_PREFIX + camera.getProperties().name() + "/";
            boolean isReplay = Constants.simMode == Constants.Mode.REPLAY;

            PhotonPipelineResult[] results = camera.getUnreadResults().orElse(null);
            if (results == null) {
                continue;
            }
            if (results.length > MAX_UNREAD_RESULTS) {
                results =
                        Arrays.copyOfRange(
                                results, results.length - MAX_UNREAD_RESULTS, results.length);
            }

            ArrayList<PhotonPipelineResult> acceptedResults = new ArrayList<>();
            ArrayList<PhotonPipelineResult> rejectedResults = new ArrayList<>();
            ArrayList<Pose2d> acceptedPoses = new ArrayList<>();
            ArrayList<Pose2d> rejectedPoses = new ArrayList<>();
            for (var result : results) {

                if (result.targets.size() == 1
                        && Constants.FILTERED_TAGS.contains(
                                result.targets.get(0).getFiducialId())) {
                    if (isReplay) {
                        rejectedResults.add(result);
                    }
                    continue;
                }

                if (!preFilter(result)) {
                    if (isReplay) {
                        rejectedResults.add(result);
                    }
                    continue;
                }

                Optional<EstimatedRobotPose> estPose = Optional.empty();

                if (result.multitagResult.isPresent()) {
                    estPose = poseEstimator.estimateCoprocMultiTagPose(result);
                    if (estPose.isEmpty()) {
                        estPose = poseEstimator.estimateLowestAmbiguityPose(result);
                    }
                } else {
                    estPose = poseEstimator.estimateLowestAmbiguityPose(result);
                }

                if (estPose.isEmpty()) {
                    if (isReplay) {
                        rejectedResults.add(result);
                    }
                    continue;
                }

                VisionPoseRecord poseRecord =
                        new VisionPoseRecord(
                                estPose.get().estimatedPose,
                                getTagsUsed(estPose.get().targetsUsed),
                                getAvgDistanceMeters(estPose.get().targetsUsed));

                if (!postFilter(poseRecord.pose())) {
                    if (isReplay) {
                        rejectedResults.add(result);
                        rejectedPoses.add(poseRecord.pose().toPose2d());
                    }
                    continue;
                }

                double stdDevFactor = computeStdDevFactor(camera, result, poseRecord);

                double linearStdDev = LINEAR_STDDEV_BASELINE * stdDevFactor;
                double angularStdDev = ANGULAR_STDDEV_BASELINE * stdDevFactor;

                robotState.addVisionObservation(
                        new VisionPoseObservation(
                                result.getTimestampSeconds(),
                                poseRecord.pose().toPose2d(),
                                poseRecord.averageDistanceMeters(),
                                poseRecord.tagsUsed().size(),
                                linearStdDev,
                                angularStdDev));

                acceptedResults.add(result);
                acceptedPoses.add(poseRecord.pose().toPose2d());
            }

            Set<Integer> tagsAccepted = new HashSet<>();

            Logger.recordOutput(
                    cameraLogPrefix + "/Results/AcceptedLength", acceptedResults.size());
            for (int i = 0; i < acceptedResults.size(); i++) {
                Logger.recordOutput(
                        cameraLogPrefix + "/Results/Accepted/" + i, acceptedResults.get(i));

                tagsAccepted.addAll(getTagsUsed(acceptedResults.get(i).targets));
            }

            Logger.recordOutput(
                    cameraLogPrefix + "/Poses/Accepted", acceptedPoses.toArray(Pose2d[]::new));

            List<Pose3d> tagPosesAccepted =
                    tagsAccepted.stream()
                            .map(this::getTagPose)
                            .filter(Optional::isPresent)
                            .map(Optional::get)
                            .toList();

            Logger.recordOutput(
                    cameraLogPrefix + "/TagPoses/Accepted",
                    tagPosesAccepted.toArray(Pose3d[]::new));

            if (isReplay) {
                Set<Integer> tagsRejected = new HashSet<>();
                Logger.recordOutput(
                        cameraLogPrefix + "/Results/RejectedLength", rejectedResults.size());
                for (int i = 0; i < rejectedResults.size(); i++) {
                    Logger.recordOutput(
                            cameraLogPrefix + "/Results/Rejected/" + i, rejectedResults.get(i));

                    tagsRejected.addAll(getTagsUsed(rejectedResults.get(i).targets));
                }
                Logger.recordOutput(
                        cameraLogPrefix + "/Poses/Rejected", rejectedPoses.toArray(Pose2d[]::new));

                // compute rejected tag poses only when replaying
                List<Pose3d> tagPosesRejected =
                        tagsRejected.stream()
                                .map(this::getTagPose)
                                .filter(Optional::isPresent)
                                .map(Optional::get)
                                .toList();

                Logger.recordOutput(
                        cameraLogPrefix + "/TagPoses/Rejected",
                        tagPosesRejected.toArray(Pose3d[]::new));
            }
        }

        // VisionOdometryCharacterizer.printResults();
        // SmartDashboard.putNumber(
        //         "Vision Characterization Sample Count",
        //         VisionOdometryCharacterizer.getVisionSampleSize());
        // SmartDashboard.putBoolean(
        //         "Vision Characterization Sufficient Samples",
        //         VisionOdometryCharacterizer.hasSufficientVisionSamples());
        // SmartDashboard.putNumber(
        //         "Odometry Characterization Sample Count",
        //         VisionOdometryCharacterizer.getOdoSampleSize());
        // SmartDashboard.putBoolean(
        //         "Odometry Characterization Sufficient Samples",
        //         VisionOdometryCharacterizer.hasSufficientOdoSamples());
    }

    private double getAvgDistanceMeters(List<PhotonTrackedTarget> targets) {
        return targets.stream()
                .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
                .average()
                .orElse(0.0);
    }

    /**
     * Computes a standard deviation scaling heuristic for a vision measurement.
     *
     * @param camera camera used to produce the measurement
     * @param result pipeline result
     * @param poseRecord estimated pose record
     * @return clamped standard deviation scaling factor
     */
    private double computeStdDevFactor(
            AprilTagCamera camera, PhotonPipelineResult result, VisionPoseRecord poseRecord) {
        double distanceMeters =
                Math.max(poseRecord.averageDistanceMeters(), MIN_STDDEV_DISTANCE_METERS);
        int tagCount = Math.max(1, poseRecord.tagsUsed().size());
        boolean hasMultiTag = result.getMultiTagResult().isPresent();
        double ambiguity = 0.0;
        if (!hasMultiTag && result.getTargets().size() == 1) {
            double rawAmbiguity = result.getBestTarget().getPoseAmbiguity();
            ambiguity = rawAmbiguity < 0.0 ? 0.0 : rawAmbiguity;
        }

        double distanceFactor = Math.pow(distanceMeters, 2.0);
        double tagFactor = 1.0 / Math.pow(tagCount, STDDEV_TAGCOUNT_EXPONENT);
        double ambiguityFactor =
                1.0 + STDDEV_AMBIGUITY_WEIGHT * Math.pow(ambiguity / MAX_AMBIGUITY, 2.0);
        double stdDevFactor =
                distanceFactor
                        * tagFactor
                        * ambiguityFactor
                        * camera.getProperties().stdDevFactor();

        return MathUtil.clamp(stdDevFactor, STDDEV_FACTOR_MIN, STDDEV_FACTOR_MAX);
    }

    private List<Integer> getTagsUsed(List<PhotonTrackedTarget> targets) {
        List<Integer> tagsUsed = new ArrayList<>(targets.size());
        for (var target : targets) {
            tagsUsed.add(target.getFiducialId());
        }
        return tagsUsed;
    }

    private Optional<Pose3d> getTagPose(int id) {
        return AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(id);
    }
}
