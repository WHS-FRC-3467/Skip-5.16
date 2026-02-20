// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.io.objectdetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;

import frc.lib.devices.AprilTagCamera.CameraProperties;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import java.util.function.Supplier;

/** An object detection sim class that utilizes the PhotonVision implementation for tests. */
public class ObjectDetectionIOSim extends ObjectDetectionIOPhotonVision {
    private final String target_name;
    private final PhotonCameraSim camSim;
    private final VisionSystemSim visionSim;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<VisionTargetSim[]> visionTargetSupplier;

    private VisionTargetSim[] visionTargets;

    public ObjectDetectionIOSim(
            CameraProperties cameraProperties,
            Supplier<Pose2d> robotPoseSupplier,
            String target_name,
            Supplier<VisionTargetSim[]> visionTargetSupplier) {
        super(cameraProperties.name());
        this.target_name = target_name;

        var simCameraProperties = new SimCameraProperties();
        simCameraProperties.setCalibration(
                cameraProperties.resolutionWidth(),
                cameraProperties.resolutionHeight(),
                cameraProperties.cameraMatrix(),
                cameraProperties.distCoeffs());
        simCameraProperties.setFPS(cameraProperties.fps());
        simCameraProperties.setAvgLatencyMs(cameraProperties.latency().in(Units.Milliseconds));
        simCameraProperties.setLatencyStdDevMs(
                cameraProperties.latencyStdDev().in(Units.Milliseconds));
        // Generate a sim camera associated with the super's real PhotonVision camera
        camSim = new PhotonCameraSim(super.camera, simCameraProperties);

        // Wireframe visualizer for objects
        camSim.enableDrawWireframe(true);
        // Create a vision system sim and add the sim camera to it. Currently factored for only one
        // ML camera.
        visionSim = new VisionSystemSim("objectDetection");
        visionSim.addCamera(camSim, cameraProperties.robotToCamera());
        // Suppliers for dynamic sim object position updates
        this.robotPoseSupplier = robotPoseSupplier;
        this.visionTargetSupplier = visionTargetSupplier;
    }

    // Update the robot's pose in the sim and use the super's implementation to update inputs
    @Override
    public void updateInputs(ObjectDetectionIOInputs inputs) {
        // Update target & robot poses + camera observations
        updateTargetPoses();
        visionSim.update(robotPoseSupplier.get());
        super.updateInputs(inputs);
    }

    // Private helper for simulating moving game pieces.
    private void updateTargetPoses() {
        visionSim.clearVisionTargets();
        visionTargets = visionTargetSupplier.get();
        if (visionTargets.length > 0) {
            visionSim.addVisionTargets(target_name, visionTargets);
        }
    }
}
