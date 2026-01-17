// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.io.objectdetection;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import frc.lib.devices.AprilTagCamera.CameraProperties;

/** An object detection sim class that utilizes the PhotonVision implementation for tests. */
public class ObjectDetectionIOSim extends ObjectDetectionIOPhotonVision {
    private final String target_name;
    private final PhotonCameraSim camSim;
    private final VisionSystemSim system;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Transform3d robotToCamera;
    private final Supplier<VisionTargetSim[]> visionTargetSupplier;

    private VisionTargetSim[] visionTargets;
    private Set<VisionTargetSim> targetSet;
    private List<VisionTargetSim> targetList;
    // Vision targets in the FOV of the camera
    List<VisionTargetSim> seenTargets = new ArrayList<>();
    // Storing the seen targets inside array of strings
    VisionTargetSim[] seenTargetsArray;
    VisionTargetSim[] seenTargetsArrayBuffer;
    int counter = 0;

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
        this.robotToCamera = cameraProperties.robotToCamera();
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
        seenTargetsArrayBuffer = new VisionTargetSim[visionTargets.length];
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
        // Get all latest vision targets from supplier
        visionTargets = visionTargetSupplier.get();
        Pose3d cameraPose = new Pose3d(robotPoseSupplier.get()).plus(robotToCamera);
        Map<VisionTargetSim,Integer> targetIndexMap = new IdentityHashMap<>();
        for (int i = 0; i < visionTargets.length; ++i) {
            targetIndexMap.put(visionTargets[i], i);
        }

        // Log updated target poses for AScope before filtering
        for (VisionTargetSim target : visionTargets) {
            if (counter % 5 == 0) {
                Logger.recordOutput("Object Sim/TARGET POSE/" + targetIndexMap.get(target), target.getPose());
            }
            counter++;
            // Filter out targets that are not visible to the camera based on its FOV for display on dashboard
            // Also log whether each one is "seen"
            if (camSim.canSeeTargetPose(cameraPose, target)) {
                seenTargets.add(target);
                Logger.recordOutput("Object Sim/SeesTarget/" + targetList.indexOf(target), true);
            } else {
                Logger.recordOutput("Object Sim/SeesTarget/" + targetList.indexOf(target), false);
            }
        }

        seenTargetsArrayBuffer = seenTargets.toArray(seenTargetsArrayBuffer);

        // Add seen vision targets to the sim field
        system.addVisionTargets(target_name, Arrays.copyOf(seenTargetsArrayBuffer, seenTargets.size()));
        // seenTargetsArray = null;
        seenTargets.clear(); // Clear list for next update
        
        super.updateInputs(inputs);
    }

    @Override
    public String getCamera()
    {
        return cameraName;
    }
}
