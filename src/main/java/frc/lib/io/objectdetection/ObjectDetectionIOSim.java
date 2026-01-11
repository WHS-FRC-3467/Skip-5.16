// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.io.objectdetection;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

/** An object detection sim class that utilizes the PhotonVision implementation for tests. */
public class ObjectDetectionIOSim extends ObjectDetectionIOPhotonVision {
    private final VisionSystemSim visionSim;
    private final PhotonCamera cam;
    private final PhotonCameraSim camSim;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<VisionTargetSim[]> visionTargetSupplier;
    private VisionTargetSim[] visionTargets;
    private Set<VisionTargetSim> targetSet;
    private List<VisionTargetSim> targetList;
    private final String target_name;

    public ObjectDetectionIOSim(String cameraName, Transform3d cameraTransform,
        Supplier<Pose2d> robotPoseSupplier,
        String target_name, Supplier<VisionTargetSim[]> visionTargetSupplier)
    {
        super(cameraName);
        this.target_name = target_name;
        // Initialize simulated object detection camera
        cam = new PhotonCamera(cameraName);
        camSim = new PhotonCameraSim(cam, new SimCameraProperties());
        // Wireframe visualizer for objects
        camSim.enableDrawWireframe(true);
        // Create a vision system sim and add the sim camera to it
        visionSim = new VisionSystemSim("objectDetection");
        visionSim.addCamera(camSim, cameraTransform);
        // Suppliers for dynamic sim object position updates
        this.robotPoseSupplier = robotPoseSupplier;
        this.visionTargetSupplier = visionTargetSupplier;
        // Initialize sim vision targets on field
        // Current vision targets
        visionTargets = visionTargetSupplier.get();
        // Add current vision targets to the sim field
        visionSim.addVisionTargets(target_name, visionTargets);
        // Retrieve the vision targets on the sim field in a set and then convert it to a list for
        // easy indexing
        targetSet = visionSim.getVisionTargets();
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
        visionSim.update(robotPoseSupplier.get());
        visionSim.clearVisionTargets();
        visionTargets = visionTargetSupplier.get();
        visionSim.addVisionTargets(target_name, visionTargets);
        // Log updated target poses for AScope
        targetSet = visionSim.getVisionTargets();
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
