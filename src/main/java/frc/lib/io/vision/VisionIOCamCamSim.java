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

package frc.lib.io.vision;

import static edu.wpi.first.units.Units.Radians;

import com.google.flatbuffers.FlatBufferBuilder;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;

import frc.lib.devices.CamCamCoproc.CameraProperties;
import frc.robot.generated.flatbuffers.CamCamConfiguration;
import frc.robot.generated.flatbuffers.CamCamData;
import frc.robot.generated.flatbuffers.ConfiguredData;
import frc.robot.generated.flatbuffers.Data;
import frc.robot.generated.flatbuffers.PoseObservation;
import frc.robot.generated.flatbuffers.PoseObservations;
import frc.robot.generated.flatbuffers.YOLOObservation;
import frc.robot.generated.flatbuffers.YOLOObservations;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Simulated CamCam VisionIO that emits AprilTag pose observations and optional object-detection
 * observations from one shared coprocessor queue.
 */
public class VisionIOCamCamSim implements VisionIO {
    private record SimPoseObservation(
            double timestampSeconds,
            edu.wpi.first.math.geometry.Pose3d pose,
            double averageDistanceMeters,
            double ambiguity,
            int numTags) {}

    private final String targetName;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<VisionTargetSim[]> visionTargetSupplier;

    private final VisionSystemSim apriltagVisionSim;
    private final PhotonCamera[] apriltagCameras;
    private final PhotonPoseEstimator[] apriltagPoseEstimators;

    private final VisionSystemSim objectDetectionVisionSim;
    private final PhotonCamera objectDetectionCamera;

    private boolean apriltagConfigured = false;
    private boolean objectDetectionConfigured = false;

    /**
     * Creates a simulated CamCam transport backed by PhotonVision simulation objects.
     *
     * @param apriltagCameraProperties AprilTag camera definitions hosted by the simulated
     *     coprocessor
     * @param objectDetectionCameraProperties optional object-detection camera definition
     * @param apriltagLayout AprilTag field layout to expose to the simulator
     * @param robotPoseSupplier supplier for the current robot pose used to step simulation
     * @param targetName PhotonVision target namespace for simulated object detections
     * @param visionTargetSupplier supplier for simulated object-detection targets
     */
    public VisionIOCamCamSim(
            CameraProperties[] apriltagCameraProperties,
            CameraProperties objectDetectionCameraProperties,
            AprilTagFieldLayout apriltagLayout,
            Supplier<Pose2d> robotPoseSupplier,
            String targetName,
            Supplier<VisionTargetSim[]> visionTargetSupplier) {
        this.targetName = targetName;
        this.robotPoseSupplier = robotPoseSupplier;
        this.visionTargetSupplier = visionTargetSupplier;

        apriltagVisionSim = new VisionSystemSim("camcam-apriltag");
        apriltagVisionSim.addAprilTags(apriltagLayout);
        apriltagCameras = new PhotonCamera[apriltagCameraProperties.length];
        apriltagPoseEstimators = new PhotonPoseEstimator[apriltagCameraProperties.length];
        for (int i = 0; i < apriltagCameraProperties.length; i++) {
            CameraProperties cameraProperties = apriltagCameraProperties[i];
            PhotonCamera camera = new PhotonCamera(cameraProperties.name());
            PhotonCameraSim cameraSim =
                    new PhotonCameraSim(
                            camera, createSimCameraProperties(cameraProperties), apriltagLayout);
            cameraSim.enableDrawWireframe(true);
            apriltagVisionSim.addCamera(cameraSim, cameraProperties.robotToCamera());
            apriltagCameras[i] = camera;
            apriltagPoseEstimators[i] =
                    new PhotonPoseEstimator(apriltagLayout, cameraProperties.robotToCamera());
        }

        if (objectDetectionCameraProperties != null) {
            objectDetectionVisionSim = new VisionSystemSim("camcam-object-detection");
            objectDetectionCamera = new PhotonCamera(objectDetectionCameraProperties.name());
            PhotonCameraSim cameraSim =
                    new PhotonCameraSim(
                            objectDetectionCamera,
                            createSimCameraProperties(objectDetectionCameraProperties));
            cameraSim.enableDrawWireframe(true);
            objectDetectionVisionSim.addCamera(
                    cameraSim, objectDetectionCameraProperties.robotToCamera());
        } else {
            objectDetectionVisionSim = null;
            objectDetectionCamera = null;
        }
    }

    @Override
    public void configure(byte[] configuration) {
        apriltagConfigured = hasApriltagConfiguration(configuration);
        objectDetectionConfigured = hasObjectDetectionConfiguration(configuration);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        Pose2d robotPose = robotPoseSupplier.get();
        ArrayList<SimPoseObservation> poseObservations = new ArrayList<>();
        ArrayList<PhotonPipelineResult> objectDetectionResults = new ArrayList<>();
        boolean configured = apriltagConfigured || objectDetectionConfigured;

        if (apriltagConfigured) {
            apriltagVisionSim.update(robotPose);
            for (int i = 0; i < apriltagCameras.length; i++) {
                for (PhotonPipelineResult result : apriltagCameras[i].getAllUnreadResults()) {
                    estimatePose(apriltagPoseEstimators[i], result)
                            .ifPresent(poseObservations::add);
                }
            }
        }

        if (objectDetectionConfigured
                && objectDetectionVisionSim != null
                && objectDetectionCamera != null) {
            updateTargetPoses();
            objectDetectionVisionSim.update(robotPose);
            objectDetectionResults.addAll(objectDetectionCamera.getAllUnreadResults());
        }

        if (!configured) {
            inputs.unreadData = new byte[][] {};
            return;
        }

        inputs.unreadData =
                new byte[][] {serializeConfiguredData(poseObservations, objectDetectionResults)};
    }

    private SimCameraProperties createSimCameraProperties(CameraProperties cameraProperties) {
        var simCameraProperties = new SimCameraProperties();
        if (cameraProperties.cameraMatrix() == null || cameraProperties.distCoeffs() == null) {
            if (cameraProperties.fov() != null) {
                simCameraProperties.setCalibration(
                        cameraProperties.resolutionWidth(),
                        cameraProperties.resolutionHeight(),
                        Rotation2d.fromRadians(cameraProperties.fov().in(Radians)));
            }
        } else {
            simCameraProperties.setCalibration(
                    cameraProperties.resolutionWidth(),
                    cameraProperties.resolutionHeight(),
                    cameraProperties.cameraMatrix(),
                    cameraProperties.distCoeffs());
        }

        simCameraProperties.setFPS(cameraProperties.fps());
        if (cameraProperties.latency() != null) {
            simCameraProperties.setAvgLatencyMs(cameraProperties.latency().in(Units.Milliseconds));
        }
        if (cameraProperties.latencyStdDev() != null) {
            simCameraProperties.setLatencyStdDevMs(
                    cameraProperties.latencyStdDev().in(Units.Milliseconds));
        }
        return simCameraProperties;
    }

    private Optional<SimPoseObservation> estimatePose(
            PhotonPoseEstimator estimator, PhotonPipelineResult result) {
        Optional<EstimatedRobotPose> estimatedPose;
        if (result.getMultiTagResult().isPresent()) {
            estimatedPose = estimator.estimateCoprocMultiTagPose(result);
            if (estimatedPose.isEmpty()) {
                estimatedPose = estimator.estimateLowestAmbiguityPose(result);
            }
        } else {
            estimatedPose = estimator.estimateLowestAmbiguityPose(result);
        }

        return estimatedPose.map(
                pose ->
                        new SimPoseObservation(
                                pose.timestampSeconds,
                                pose.estimatedPose,
                                averageDistanceMeters(pose.targetsUsed),
                                ambiguity(result, pose.targetsUsed),
                                Math.max(1, pose.targetsUsed.size())));
    }

    private void updateTargetPoses() {
        objectDetectionVisionSim.clearVisionTargets();
        VisionTargetSim[] targets = visionTargetSupplier.get();
        if (targets.length > 0) {
            objectDetectionVisionSim.addVisionTargets(targetName, targets);
        }
    }

    private boolean hasApriltagConfiguration(byte[] configuration) {
        return parseConfiguration(configuration)
                .map(config -> config.apriltagCamerasLength() > 0)
                .orElse(false);
    }

    private boolean hasObjectDetectionConfiguration(byte[] configuration) {
        return parseConfiguration(configuration)
                .map(config -> config.objectDetectionCamera() != null)
                .orElse(false);
    }

    private Optional<CamCamConfiguration> parseConfiguration(byte[] configuration) {
        if (configuration == null || configuration.length < 8) {
            return Optional.empty();
        }

        ByteBuffer buffer = ByteBuffer.wrap(configuration);
        if (!CamCamConfiguration.CamCamConfigurationBufferHasIdentifier(buffer)) {
            return Optional.empty();
        }

        buffer.rewind();
        return Optional.of(CamCamConfiguration.getRootAsCamCamConfiguration(buffer));
    }

    private byte[] serializeConfiguredData(
            List<SimPoseObservation> poseObservations,
            List<PhotonPipelineResult> objectDetectionResults) {
        FlatBufferBuilder builder = new FlatBufferBuilder(0);
        int poseObservationsOffset =
                poseObservations.isEmpty()
                        ? 0
                        : serializePoseObservations(builder, poseObservations);
        int objectDetectionObservationsOffset =
                objectDetectionResults.isEmpty()
                        ? 0
                        : serializeObjectDetectionObservations(builder, objectDetectionResults);
        int configuredData =
                ConfiguredData.createConfiguredData(
                        builder, poseObservationsOffset, objectDetectionObservationsOffset, 0);
        int camCamData = CamCamData.createCamCamData(builder, Data.ConfiguredData, configuredData);
        CamCamData.finishCamCamDataBuffer(builder, camCamData);
        return builder.sizedByteArray();
    }

    private int serializePoseObservations(
            FlatBufferBuilder builder, List<SimPoseObservation> poseObservations) {
        PoseObservations.startObservationsVector(builder, poseObservations.size());
        for (int i = poseObservations.size() - 1; i >= 0; i--) {
            SimPoseObservation observation = poseObservations.get(i);
            var quaternion = observation.pose().getRotation().getQuaternion();
            PoseObservation.createPoseObservation(
                    builder,
                    observation.timestampSeconds(),
                    observation.pose().getX(),
                    observation.pose().getY(),
                    observation.pose().getZ(),
                    quaternion.getW(),
                    quaternion.getX(),
                    quaternion.getY(),
                    quaternion.getZ(),
                    observation.averageDistanceMeters(),
                    observation.ambiguity(),
                    observation.numTags());
        }
        int observationsVector = builder.endVector();
        return PoseObservations.createPoseObservations(builder, observationsVector);
    }

    private double averageDistanceMeters(List<PhotonTrackedTarget> targets) {
        return targets.stream()
                .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
                .average()
                .orElse(0.0);
    }

    private double ambiguity(PhotonPipelineResult result, List<PhotonTrackedTarget> targetsUsed) {
        if (result.getMultiTagResult().isPresent() || targetsUsed.size() != 1) {
            return -1.0;
        }

        double ambiguity = targetsUsed.get(0).getPoseAmbiguity();
        return ambiguity >= 0.0 ? ambiguity : -1.0;
    }

    private int serializeObjectDetectionObservations(
            FlatBufferBuilder builder, List<PhotonPipelineResult> objectDetectionResults) {
        int[] observationOffsets = new int[objectDetectionResults.size()];
        for (int i = 0; i < objectDetectionResults.size(); i++) {
            observationOffsets[i] =
                    serializeObjectDetectionObservation(builder, objectDetectionResults.get(i));
        }
        int observationsVector =
                YOLOObservations.createObservationsVector(builder, observationOffsets);
        return YOLOObservations.createYOLOObservations(builder, observationsVector);
    }

    private int serializeObjectDetectionObservation(
            FlatBufferBuilder builder, PhotonPipelineResult result) {
        List<PhotonTrackedTarget> targets = result.getTargets();
        YOLOObservation.startObjectsVector(builder, targets.size());
        for (int i = targets.size() - 1; i >= 0; i--) {
            PhotonTrackedTarget target = targets.get(i);
            frc.robot.generated.flatbuffers.ObjectDetection.createObjectDetection(
                    builder,
                    (short) target.getDetectedObjectClassID(),
                    target.getPitch(),
                    target.getYaw(),
                    target.getArea());
        }
        int objectsVector = builder.endVector();
        return YOLOObservation.createYOLOObservation(
                builder, result.getTimestampSeconds(), objectsVector);
    }
}
