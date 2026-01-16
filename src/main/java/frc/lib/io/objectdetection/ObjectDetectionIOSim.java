// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.io.objectdetection;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import frc.lib.devices.AprilTagCamera.CameraProperties;

/** An object detection sim class that utilizes the PhotonVision implementation for tests. */
public class ObjectDetectionIOSim extends ObjectDetectionIOPhotonVision {
    private final String target_name;
    private final PhotonCameraSim camSim;
    private final VisionSystemSim system;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<VisionTargetSim[]> visionTargetSupplier;

    private VisionTargetSim[] visionTargets;
    private Set<VisionTargetSim> targetSet;
    private List<VisionTargetSim> targetList;

    public ObjectDetectionIOSim(
        CameraProperties cameraProperties, 
        VisionSystemSim system,
        Supplier<Pose2d> robotPoseSupplier,
        String target_name, 
        Supplier<VisionTargetSim[]> visionTargetSupplier)
    {
        super(cameraProperties);
        this.target_name = target_name;
        // Suppliers for dynamic sim object position updates
        this.robotPoseSupplier = robotPoseSupplier;
        this.visionTargetSupplier = visionTargetSupplier;
        this.system = system;

        var simCameraProperties = new SimCameraProperties();
        simCameraProperties.setCalibration(
            cameraProperties.resolutionWidth(),
            cameraProperties.resolutionHeight(),
            cameraProperties.cameraMatrix(),
            cameraProperties.distCoeffs());
        simCameraProperties.setFPS(cameraProperties.fps());
        simCameraProperties.setAvgLatencyMs(cameraProperties.latency().in(Units.Milliseconds));  
        simCameraProperties.setLatencyStdDevMs(cameraProperties.latencyStdDev().in(Units.Milliseconds));
        // Generate a sim camera associated with the super's real PhotonVision camera
        camSim = new PhotonCameraSim(super.camera, simCameraProperties);

        // Wireframe visualizer for objects
        camSim.enableDrawWireframe(true);

        // Add the sim camera to the VisionSystemSim. Currently factored for only one ML camera.
        system.addCamera(camSim, cameraProperties.robotToCamera());
        
        // Initialize sim vision targets on field
        // Current vision targets
        visionTargets = visionTargetSupplier.get();
        // Add current vision targets to the sim field
        system.addVisionTargets(target_name, visionTargets);
        // Retrieve the vision targets on the sim field in a set and then convert it to a list for
        // easy indexing
        targetSet = system.getVisionTargets();
        targetList = new ArrayList<>(targetSet);
        // Initialize sim target pose logging; update in periodic below for AScope
        for (VisionTargetSim target : targetList) {
            Logger.recordOutput("TARGET POSE" + targetList.indexOf(target), target.getPose());
        }
    }

    // Update the robot's pose in the sim and use the super's implementation to update inputs
    @Override
    public void updateInputs(ObjectDetectionIOInputs inputs)
    {
        // Update robot & target poses
        system.update(robotPoseSupplier.get());
        system.clearVisionTargets();
        visionTargets = visionTargetSupplier.get();
        system.addVisionTargets(target_name, visionTargets);
        // Log updated target poses for AScope
        targetSet = system.getVisionTargets();
        targetList = new ArrayList<>(targetSet);
        for (VisionTargetSim target : targetList) {
            Logger.recordOutput("TARGET POSE" + targetList.indexOf(target), target.getPose());
        }
        super.updateInputs(inputs);
    }

    @Override
    public String getCamera()
    {
        return cameraName;
    }
}
