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

package frc.robot.subsystems.objectdetector;

import frc.lib.devices.ObjectDetection;
import frc.lib.devices.ObjectDetection.ObjectDetectionObservation;
import frc.lib.devices.ObjectDetection.ContourSelectionMode;
import frc.lib.io.objectdetection.ObjectDetectionIO;
import frc.robot.RobotState;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Meters;
import java.util.ArrayList;
import java.util.Optional;

/**
 * Generates the Object Detection subsystem which consists of some number of Object Detection
 * Devices (each representing a unique camera) each with their own inputs data structure that
 * periodically updates via a call from the device layer to the IO implementation layer.
 */
public class ObjectDetector extends SubsystemBase {
    private final RobotState robotState = RobotState.getInstance();
    private final ObjectDetection objectDetection;

    @Getter
    private Optional<ObjectDetectionObservation> latestObjectObservation;
    @Getter
    private Optional<ObjectDetectionObservation> latestBigContourObservation;
    @Getter
    private Optional<ObjectDetectionObservation> latestLowContourObservation;
    @Getter
    private ArrayList<Translation2d> objectPoseBuffer = new ArrayList<>(10);

    // Pass in any Object Detection IO implementation (e.g. ObjectDetectionIOPhotonVision) which can
    // be real or sim. Generates an Object Detection device with a unique inputs field that can
    // update periodically. Currently factored for only PhotonVision & only one camera.
    public ObjectDetector(ObjectDetectionIO io)
    {
        objectDetection = new ObjectDetection(io);
    }

    // Private helper for generating latest ML Object Observation and updating internal buffer of
    // detected objects.
    private Optional<ObjectDetectionObservation> latestObjectObservation()
    {
        Optional<ObjectDetectionObservation> observation =
            objectDetection.getObjectObservation(objectDetection.getTargets()[0],
                ObjectDetectorConstants.CAMERA0_TRANSFORM,
                ObjectDetectorConstants.OBJECT0_HEIGHT_METERS, 1, 0, 1, 0,
                robotState.getEstimatedPose());
        if (observation.isEmpty() || observation.get().objectPose().isEmpty()) {
            return Optional.empty();
        }
        // Update object pose buffer of most-recently detected objects
        objectDetection.updateObservationPoseBuffer(10, objectPoseBuffer, 0.4572,
            observation.get().objectPose().get().getTranslation());
        // Object pose determined and buffer updated -- log results
        logObjectObservation(observation.get().distance().get().in(Meters),
            observation.get().objectPose().get().getTranslation());
        // Return packaged latest ML detection
        return observation;
    }

    // Private helper for generating latest Color Contour Object observation
    private Optional<ObjectDetectionObservation> latestContourObservation(
        ContourSelectionMode selection)
    {
        // Return packaged latest Color Contour observation
        return objectDetection.getContourObservation(objectDetection.getTargets(), selection);
    }

    // Private helper for logging Object Detection values
    private void logObjectObservation(double distance,
        Translation2d targetLocation)
    {
        // Logged calculations
        Logger.recordOutput("Detection/" + "Calculated Distance", distance);

        Logger.recordOutput("Detection/" + "Latest Detection's Calculated Coordinates",
            targetLocation);

        Logger.recordOutput("Detection/" + "Newest Detection",
            objectPoseBuffer.get(objectPoseBuffer.size() - 1));

        Logger.recordOutput("Detection/" + "Oldest Detection", objectPoseBuffer.get(0));

        Logger.recordOutput("Detection/" + "Detection List Size", objectPoseBuffer.size());

        Logger.recordOutput("Detection/" + "Sim Target #0 True Range",
            ObjectDetectorConstants.SIM_TARGETS[0].getPose().toPose2d().getTranslation()
                .minus(robotState.getEstimatedPose().getTranslation()).getX());

        Logger.recordOutput("Detection/" + "Sim Target #0 True Heading",
            ObjectDetectorConstants.SIM_TARGETS[0].getPose().toPose2d().getTranslation()
                .minus(robotState.getEstimatedPose().getTranslation()).getY());

        Logger.recordOutput("Detection/" + "Sim Target #0 True Distance",
            ObjectDetectorConstants.SIM_TARGETS[0].getPose().toPose2d().getTranslation()
                .getDistance(robotState.getEstimatedPose().getTranslation()));
    }

    @Override
    public void periodic()
    {
        // Update the inputs data structure associated with this object detection camera with latest
        // readings.
        objectDetection.periodic();
        // Inputs from camera now updated, re-populate observations with new data.
        if (objectDetection.getTargets().length > 0) {
            // Generate latest ML Object observation
            latestObjectObservation = latestObjectObservation();
            // Generate latest Color Contour observations
            latestBigContourObservation = latestContourObservation(ContourSelectionMode.LARGEST);
            latestLowContourObservation = latestContourObservation(ContourSelectionMode.LOWEST);
        } else {
            return;
        }
    }
}

