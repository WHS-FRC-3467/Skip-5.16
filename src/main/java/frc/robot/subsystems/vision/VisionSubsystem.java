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

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.devices.AprilTagCamera;
import frc.lib.posestimator.PoseEstimator.VisionPoseObservation;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.RobotState;

/**
 * The {@code VisionSubsystem} manages all vision-related processing for the robot.
 * 
 * <p>
 * It uses one or more {@link AprilTagCamera}s to detect field elements and estimate the robot's
 * pose on the field. Observations are processed through the {@link MultiTagOnCoproc} vision
 * processor, with a fallback to {@link LowestAmbiguity} if necessary. Valid observations are added
 * to {@link RobotState} for use in localization and navigation.
 * 
 * <p>
 * The subsystem periodically polls cameras for new results and logs both accepted and rejected
 * vision observations.
 */
public class VisionSubsystem extends SubsystemBase {

    public static final String LOG_PREFIX = "VisionProcessor/";

    /** Baseline linear standard deviation used for vision observations. */
    public static final double LINEAR_STDDEV_BASELINE = 0.4;

    /** Baseline angular standard deviation used for vision observations. */
    public static final double ANGULAR_STDDEV_BASELINE = 0.4;

    /** Maximum allowable height (Z-axis) of a detected pose to be considered valid. */
    public static final double MAX_Z_METERS = 0.75;

    /** Maximum allowable distance from a target to be considered valid. */
    public static final double MAX_DISTANCE_METERS = 10;

    /** Maximum ambiguity ratio allowed in a result */
    public static final double MAX_AMBIGUITY = 0.2;

    /** Width of the field in meters. */
    public static final double FIELD_WIDTH = FieldConstants.FIELD_WIDTH;

    /** Length of the field in meters. */
    public static final double FIELD_LENGTH = FieldConstants.FIELD_LENGTH;

    public static final record VisionPoseRecord(
        Pose3d pose,
        List<Integer> tagsUsed,
        double averageDistanceMeters) {
    }

    /**
     * Quickly checks whether a {@link PhotonPipelineResult} is likely to be useful before full
     * processing.
     * <p>
     * Rejects results with no targets, ambiguous poses above 0.2, or targets farther than 10
     * meters.
     * </p>
     *
     * @param result the pipeline result to pre-filter
     * @return {@code true} if the result passes preliminary checks, {@code false} otherwise
     */
    public static boolean preFilter(PhotonPipelineResult result)
    {
        if (!result.hasTargets()) {
            return false;
        }

        if (result.getMultiTagResult().isPresent()) {
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
     * <p>
     * A pose is considered valid if it is within field boundaries and below {@link #MAX_Z_METERS}.
     * </p>
     *
     * @param pose the pose to validate
     * @return {@code true} if the pose is valid, {@code false} otherwise
     */
    public static boolean postFilter(Pose3d pose)
    {
        double x = pose.getX();
        double y = pose.getY();
        double z = pose.getZ();
        return !(z > MAX_Z_METERS || x < 0.0 || x > FIELD_LENGTH || y < 0.0 || y > FIELD_WIDTH);
    }

    private final RobotState robotState = RobotState.getInstance();
    private final AprilTagCamera[] cameras;
    private final PhotonPoseEstimator[] poseEstimators;

    /**
     * Constructs a new {@code VisionSubsystem} with the specified cameras.
     *
     * @param cameras the cameras to use for vision processing
     */
    public VisionSubsystem(AprilTagCamera... cameras)
    {
        this.cameras = cameras;
        this.poseEstimators = new PhotonPoseEstimator[cameras.length];
        for (int i = 0; i < cameras.length; i++) {
            this.poseEstimators[i] = new PhotonPoseEstimator(
                AprilTagLayoutType.OFFICIAL.getLayout(),
                cameras[i].getProperties().robotToCamera());
        }
    }

    @Override
    public void periodic()
    {
        for (int c = 0; c < cameras.length; c++) {
            String cameraLogPrefix = LOG_PREFIX + cameras[c].getProperties().name() + "/";

            PhotonPipelineResult[] results = cameras[c].getUnreadResults().orElse(null);
            if (results == null) {
                continue;
            }

            ArrayList<PhotonPipelineResult> acceptedResults = new ArrayList<>();
            ArrayList<PhotonPipelineResult> rejectedResults = new ArrayList<>();
            ArrayList<Pose2d> acceptedPoses = new ArrayList<>();
            ArrayList<Pose2d> rejectedPoses = new ArrayList<>();

            for (var result : results) {
                if (!preFilter(result)) {
                    rejectedResults.add(result);
                    continue;
                }

                Optional<EstimatedRobotPose> estPose = Optional.empty();

                if (result.multitagResult.isPresent()) {
                    estPose = poseEstimators[c].estimateCoprocMultiTagPose(result);
                    if (estPose.isEmpty()) {
                        estPose = poseEstimators[c].estimateLowestAmbiguityPose(result);
                    }
                } else {
                    estPose = poseEstimators[c].estimateLowestAmbiguityPose(result);
                }

                if (estPose.isEmpty()) {
                    rejectedResults.add(result);
                    continue;
                }

                VisionPoseRecord poseRecord = new VisionPoseRecord(
                    estPose.get().estimatedPose,
                    getTagsUsed(estPose.get().targetsUsed),
                    getAvgDistanceMeters(estPose.get().targetsUsed));

                if (!postFilter(poseRecord.pose())) {
                    rejectedResults.add(result);
                    rejectedPoses.add(
                        poseRecord.pose().toPose2d());
                    continue;
                }

                // Equation from AK template project
                // https://github.com/Mechanical-Advantage/AdvantageKit/blob/5dbc08a680e8b105c75c18be7c3442029b08e32b/template_projects/sources/vision/src/main/java/frc/robot/subsystems/vision/Vision.java#L123
                double stdDevFactor =
                    (Math.pow(poseRecord.averageDistanceMeters(), 2.0) / result.getTargets().size())
                        * cameras[c].getProperties().stdDevFactor();
                double linearStdDev = LINEAR_STDDEV_BASELINE * stdDevFactor;
                double angularStdDev = ANGULAR_STDDEV_BASELINE * stdDevFactor;

                robotState.addVisionObservation(
                    new VisionPoseObservation(
                        result.getTimestampSeconds(),
                        poseRecord.pose().toPose2d(),
                        linearStdDev,
                        angularStdDev));

                acceptedResults.add(result);
                acceptedPoses.add(
                    poseRecord.pose().toPose2d());
            }

            Logger.recordOutput(
                cameraLogPrefix + "/Results/AcceptedLength",
                acceptedResults.size());
            for (int i = 0; i < acceptedResults.size(); i++) {
                Logger.recordOutput(
                    cameraLogPrefix + "/Results/Accepted/" + i,
                    acceptedResults.get(i));
            }

            Logger.recordOutput(
                cameraLogPrefix + "/Results/RejectedLength",
                rejectedResults.size());
            for (int i = 0; i < rejectedResults.size(); i++) {
                Logger.recordOutput(
                    cameraLogPrefix + "/Results/Rejected/" + i,
                    rejectedResults.get(i));
            }

            Logger.recordOutput(
                cameraLogPrefix + "/Poses/AcceptedLength",
                acceptedPoses.size());
            for (int i = 0; i < acceptedPoses.size(); i++) {
                Logger.recordOutput(
                    cameraLogPrefix + "/Poses/Accepted/" + i,
                    acceptedPoses.get(i));
            }

            Logger.recordOutput(
                cameraLogPrefix + "/Poses/RejectedLength",
                rejectedPoses.size());
            for (int i = 0; i < rejectedPoses.size(); i++) {
                Logger.recordOutput(
                    cameraLogPrefix + "/Poses/Rejected/" + i,
                    rejectedPoses.get(i));
            }
        }
    }

    private double getAvgDistanceMeters(List<PhotonTrackedTarget> targets)
    {
        return targets.stream()
            .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
            .average().orElse(0.0);
    }

    private List<Integer> getTagsUsed(List<PhotonTrackedTarget> targets)
    {
        List<Integer> tagsUsed = new ArrayList<>(targets.size());
        for (var target : targets) {
            tagsUsed.add(target.getFiducialId());
        }
        return tagsUsed;
    }
}
