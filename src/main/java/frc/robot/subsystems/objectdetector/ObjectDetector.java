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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.devices.ObjectDetection;
import frc.lib.devices.ObjectDetection.ContourSelectionMode;
import frc.lib.devices.ObjectDetection.ObjectDetectionObservation;
import frc.lib.io.objectdetection.ObjectDetectionIO;
import frc.lib.util.FieldUtil;
import frc.robot.FieldConstants;
import frc.robot.RobotState;

import lombok.Getter;
import lombok.Setter;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

/**
 * Generates the Object Detection subsystem which consists of some number of Object Detection
 * Devices (each representing a unique camera) each with their own inputs data structure that
 * periodically updates via a call from the device layer to the IO implementation layer.
 */
public class ObjectDetector extends SubsystemBase {
    private final RobotState robotState = RobotState.getInstance();
    private final ObjectDetection objectDetection;
    private int maxDetectionsSize = 0;
    private static final Translation2d INVALID_TRANSLATION = new Translation2d(-1, -1);

    @Getter private List<Optional<ObjectDetectionObservation>> latestObjectObservation;
    @Getter private Optional<ObjectDetectionObservation> latestBigContourObservation;
    @Getter private Optional<ObjectDetectionObservation> latestLowContourObservation;

    /**
     * First coordinate of rectangular bounding box for object detection ROI in BLUE-RIGHT alliance
     * frame, automatically mirrored to current alliance frame and starting location.
     */
    @Getter @Setter
    private static Translation2d bottomRightROI =
            new Translation2d(Meters.of(5.22), Meters.of(1.58));

    private static Translation2d mirroredBottomRight = new Translation2d();

    /**
     * Second coordinate of rectangular bounding box for object detection ROI in BLUE-RIGHT alliance
     * frame, automatically mirrored to current alliance frame and starting location.
     */
    @Getter @Setter
    private static Translation2d topLeftROI = new Translation2d(Meters.of(8.28), Meters.of(6.49));

    private static Translation2d mirroredTopLeft = new Translation2d();

    /**
     * Constructs a new ObjectDetector subsystem with the specified IO implementation. Creates an
     * Object Detection device that can periodically update its inputs field.
     *
     * @param cameraName the name of the camera for logging purposes
     * @param io the IO implementation for object detection (real, sim, or replay)
     */
    public ObjectDetector(String cameraName, ObjectDetectionIO io) {
        objectDetection = new ObjectDetection(cameraName, io);
    }

    // Private helper for generating latest ML Object Observation
    private Optional<ObjectDetectionObservation> generateObjectObservation(
            PhotonTrackedTarget target) {
        // Attempt to generate full Object record using ML model
        Optional<ObjectDetectionObservation> observation =
                objectDetection.getObjectObservation(
                        target,
                        ObjectDetectorConstants.CAMERA0_TRANSFORM,
                        ObjectDetectorConstants.OBJECT0_HEIGHT_METERS,
                        1,
                        0,
                        1,
                        0,
                        robotState.getEstimatedPose());
        // Return latest Object observation (full, partial, or empty record)
        return observation;
    }

    // Private helper for generating latest Contour observation
    private Optional<ObjectDetectionObservation> generateContourObservation(
            ContourSelectionMode selection) {
        // Attempt to generate partial Object record using Blob model
        Optional<ObjectDetectionObservation> observation =
                objectDetection.getContourObservation(objectDetection.getTargets(), selection);
        // Return latest Blob observation (partial or empty record)
        return observation;
    }

    // Private helper for logging Object Detection values. -1 for stale values.
    private void logObjectObservation(List<Optional<ObjectDetectionObservation>> observation) {
        String prefix = "Detection/ML:";
        int detectionSize = observation.size();
        maxDetectionsSize = Math.max(detectionSize, maxDetectionsSize);

        Logger.recordOutput(prefix + "Active Detections", detectionSize);
        for (int i = 0; i < maxDetectionsSize; i++) {
            Translation2d translation = INVALID_TRANSLATION;
            if (i < detectionSize) {
                Optional<ObjectDetectionObservation> detection = observation.get(i);
                if (detection.isPresent() && detection.get().objectPose().isPresent()) {
                    translation = detection.get().objectPose().get().getTranslation();
                }
            }
            Logger.recordOutput(prefix + i, translation);
        }
    }

    // Private helper for logging Contour values. Object ID = -9999 for stale values.
    private void logContourObservation(
            Optional<ObjectDetectionObservation> observation, ContourSelectionMode mode) {
        String prefix = "Detection/" + "Contour: " + mode + ": ";
        if (observation.isPresent()) {
            ObjectDetectionObservation obs = observation.get();
            Logger.recordOutput(prefix + "Object ID", obs.objID());
            Logger.recordOutput(prefix + "Confidence", obs.confidence());
            Logger.recordOutput(prefix + "Pitch", obs.pitch().in(Degrees));
            Logger.recordOutput(prefix + "Yaw", obs.yaw().in(Degrees));
            Logger.recordOutput(prefix + "Area", obs.area());
        } else {
            Logger.recordOutput(prefix + "Object ID", -9999);
        }
    }

    /**
     * Ascertain whether a detected object's pose is within the ROI defined above, mirrored to be in
     * current alliance frame.
     */
    private boolean isObjectWithinROI(Optional<ObjectDetectionObservation> observation) {
        if (observation.isEmpty() || observation.get().objectPose().isEmpty()) {
            return false;
        }
        Pose2d objectPose = observation.get().objectPose().get();
        updateROI();
        return false; // TODO
    }

    /**
     * Updates the ROI based on current robot position, mirroring the blue-right canonical ROI if
     * red and/or left.
     */
    private void updateROI() {
        // Mirror x coordinates of ROI if necessary (red vs. blue)
        if (FieldUtil.shouldFlip()) flipX();
        // Mirror y coordinates of ROI if necessary (left vs. right)
        if (FieldUtil.isBlueLeft(robotState.getEstimatedPose())) flipY();
    }

    /** Flips the x-coordinate of the blue canonical ROI if on red alliance. */
    private void flipX() {
        Distance deltaBottomRight =
                Meters.of(FieldConstants.FIELD_LENGTH / 2.0).minus(bottomRightROI.getMeasureX());
        Distance deltaTopLeft =
                Meters.of(FieldConstants.FIELD_LENGTH / 2.0).minus(topLeftROI.getMeasureX());
        mirroredBottomRight =
                new Translation2d(
                        bottomRightROI.getMeasureX().plus(deltaBottomRight.times(2.0)),
                        bottomRightROI.getMeasureY());
        mirroredTopLeft =
                new Translation2d(
                        topLeftROI.getMeasureX().plus(deltaTopLeft.times(2.0)),
                        topLeftROI.getMeasureY());
    }

    /** Flips the y-coordinate of the right canonical ROI if starting left. */
    private void flipY() {
        Distance deltaBottomRight =
                Meters.of(FieldConstants.FIELD_WIDTH / 2.0).minus(bottomRightROI.getMeasureY());
        Distance deltaTopLeft =
                Meters.of(FieldConstants.FIELD_WIDTH / 2.0).minus(topLeftROI.getMeasureY());
        mirroredBottomRight =
                new Translation2d(
                        bottomRightROI.getMeasureX(),
                        bottomRightROI.getMeasureY().plus(deltaBottomRight.times(2.0)));
        mirroredTopLeft =
                new Translation2d(
                        topLeftROI.getMeasureX(),
                        topLeftROI.getMeasureY().plus(deltaTopLeft.times(2.0)));
    }

    @Override
    public void periodic() {
        // Update the inputs data structure associated with this Object Detection camera with latest
        // readings
        objectDetection.periodic();
        // Now that inputs are updated, re-populate observations with new data
        if (objectDetection.getTargets().length > 0) {
            // Generate latest ML Object observations
            latestObjectObservation =
                    Arrays.stream(objectDetection.getTargets())
                            .map(this::generateObjectObservation)
                            .toList();
            // Generate latest Contour observations
            latestBigContourObservation = generateContourObservation(ContourSelectionMode.LARGEST);
            latestLowContourObservation = generateContourObservation(ContourSelectionMode.LOWEST);
            // Filter out observations that are outside of the defined ROI TODO

        } else {
            // Prevent stale data from persisting
            latestObjectObservation = List.of();
            latestBigContourObservation = Optional.empty();
            latestLowContourObservation = Optional.empty();
        }
        // Log Object & Blob observations for sim
        logObjectObservation(latestObjectObservation);
        logContourObservation(latestBigContourObservation, ContourSelectionMode.LARGEST);
        logContourObservation(latestLowContourObservation, ContourSelectionMode.LOWEST);
    }
}
