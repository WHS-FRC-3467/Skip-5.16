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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Degrees;
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

    // Pass in any Object Detection IO implementation (e.g. ObjectDetectionIOPhotonVision) -- can be
    // real or sim. Generates an Object Detection device with a unique inputs field that can update
    // periodically. Currently factored for only PhotonVision & only one camera.
    public ObjectDetector(ObjectDetectionIO io)
    {
        objectDetection = new ObjectDetection(io);
    }

    // Private helper for generating latest ML Object Observation and updating internal buffer of
    // detected objects
    private Optional<ObjectDetectionObservation> latestObjectObservation()
    {
        if (objectDetection.getTargets().length > 0) {
            // Robot-local range to target
            double range =
                objectDetection.rangeToTarget_Pitch(objectDetection.getTargets()[0],
                    ObjectDetectorConstants.CAMERA0_TRANSFORM,
                    ObjectDetectorConstants.OBJECT0_HEIGHT_METERS / 2,
                    1, 0);
            // Robot-local heading to target
            double heading =
                objectDetection.headingToTarget_Yaw(objectDetection.getTargets()[0],
                    ObjectDetectorConstants.CAMERA0_TRANSFORM,
                    range, 1, 0);
            // 2D distance from robot center to target
            double distance = objectDetection.distanceToTarget2d(range, heading);
            // Field-relative Translation2D of target
            Translation2d targetLocation =
                objectDetection.estimateTargetToField(
                    range,
                    heading,
                    robotState.getEstimatedPose());
            // Update object pose buffer of most-recently detected objects
            objectDetection.updateObservationPoseBuffer(10, objectPoseBuffer, 0.4572,
                targetLocation);
            // Object pose determined and buffer updated: now, log results
            logObjectObservation(range, heading, distance, targetLocation);
            // Return packaged latest ML detection
            return Optional.ofNullable(new ObjectDetectionObservation(
                objectDetection.getTargets()[0].getDetectedObjectClassID(),
                objectDetection.getTargets()[0].getDetectedObjectConfidence(),
                Degrees.of(objectDetection.getTargets()[0].getPitch()),
                Degrees.of(objectDetection.getTargets()[0].getYaw()),
                objectDetection.getTargets()[0].getArea(),
                Optional.of(Meters.of(distance)),
                Optional.of(new Pose2d(targetLocation, new Rotation2d()))));
        } else {
            return Optional.empty();
        }
    }

    // Private helper for generating latest contour object observation
    private Optional<ObjectDetectionObservation> latestContourObservation(
        ContourSelectionMode selection)
    {
        return objectDetection.getContourObservation(objectDetection.getTargets(), selection);
    }

    // Private helper for logging Object Detection values
    private void logObjectObservation(double range, double heading, double distance,
        Translation2d targetLocation)
    {
        // Logged calculations
        Logger.recordOutput("Detection/" + "Calculated Range", range);

        Logger.recordOutput("Detection/" + "Calculated Heading", heading);

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
        // Generate latest ML object observation
        latestObjectObservation = latestObjectObservation();
        // Generate latest blob object observations
        latestBigContourObservation = latestContourObservation(ContourSelectionMode.LARGEST);
        latestLowContourObservation = latestContourObservation(ContourSelectionMode.LOWEST);
    }
}

