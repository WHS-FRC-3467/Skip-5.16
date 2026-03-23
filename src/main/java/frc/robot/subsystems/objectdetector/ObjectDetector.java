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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
     * Bottom left coordinate of a rectangular bounding box for the Object Detection ROI in the blue
     * (left) alliance frame. Automaticaly alliance corrected (i.e. the ROI is reflected such that
     * red 1 and blue 1 have the same driver station relative detection zone). Additionally, an optional
     * left/right mirror is provided by {@link #shouldMirror} for use in autos.
     */
    @Getter @Setter
    private static Translation2d bottomLeftCorner =
            new Translation2d(Meters.of(4.22), Meters.of(3.49)); // x = 5.22, y = 6.49

    /**
     * Bottom left coordinate of a rectangular bounding box for the Object Detection ROI in the blue
     * (left) alliance frame. Automaticaly alliance corrected (i.e. the ROI is reflected such that
     * red 1 and blue 1 have the same driver station relative detection zone). Additionally, an optional
     * left/right mirror is provided by {@link #shouldMirror} for use in autos.
     */
    @Getter @Setter
    private static Translation2d topRightCorner =
            new Translation2d(Meters.of(6.28), Meters.of(1.58)); // x = 8.28, y = 1.58

    /**
     * Parameter informing the Object Detector whether it should reflect its ROI to the other side
     * of the current alliance. Primarily for use in autos.
     */
    @Getter @Setter private static boolean shouldMirror = false;

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

    /**
     * Corrects the canonical blue-left ROI for the current alliance frame and applies optional
     * left/right mirroring based on {@link #shouldMirror}.
     */
    private List<Translation2d> getCorrectedROI() {
        Translation2d swappedBottomLeft = FieldUtil.apply(bottomLeftCorner);
        Translation2d swappedTopRight = FieldUtil.apply(topRightCorner);
        return List.of(
                getMirroredPoint(swappedBottomLeft, shouldMirror),
                getMirroredPoint(swappedTopRight, shouldMirror));
    }

    /** Mirrors an ROI point about the horizontal midline of the field (Y = FIELD_WIDTH / 2). */
    private Translation2d getMirroredPoint(Translation2d point, boolean flipY) {
        double y = point.getMeasureY().in(Meters);
        if (flipY) y = FieldConstants.FIELD_WIDTH - point.getMeasureY().in(Meters);
        return new Translation2d(point.getMeasureX(), Meters.of(y));
    }

    /** Ascertain whether a detected object's pose is within the corrected ROI. */
    private boolean isObjectWithinROI(
            Optional<ObjectDetectionObservation> observation,
            Translation2d correctedCornerOne,
            Translation2d correctedCornerTwo) {
        if (observation.isEmpty() || observation.get().objectPose().isEmpty()) {
            return false;
        }
        Pose2d objectPose = observation.get().objectPose().get();

        // Order coordinates for ROI boundary check
        double minX =
                Math.min(
                        correctedCornerOne.getMeasureX().in(Meters),
                        correctedCornerTwo.getMeasureX().in(Meters));
        double maxX =
                Math.max(
                        correctedCornerOne.getMeasureX().in(Meters),
                        correctedCornerTwo.getMeasureX().in(Meters));
        double minY =
                Math.min(
                        correctedCornerOne.getMeasureY().in(Meters),
                        correctedCornerTwo.getMeasureY().in(Meters));
        double maxY =
                Math.max(
                        correctedCornerOne.getMeasureY().in(Meters),
                        correctedCornerTwo.getMeasureY().in(Meters));

        return objectPose.getX() >= minX
                && objectPose.getX() <= maxX
                && objectPose.getY() >= minY
                && objectPose.getY() <= maxY;
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

    // Private helper for visualizing ROI corners.
    private void logROI(List<Translation2d> mirroredCorners) {
        double x1 = mirroredCorners.get(0).getX();
        double x2 = mirroredCorners.get(1).getX();
        double y1 = mirroredCorners.get(0).getY();
        double y2 = mirroredCorners.get(1).getY();

        Translation2d a = new Translation2d(Math.max(x1, x2), Math.max(y1, y2));
        Translation2d b = new Translation2d(Math.min(x1, x2), Math.max(y1, y2));
        Translation2d c = new Translation2d(Math.min(x1, x2), Math.min(y1, y2));
        Translation2d d = new Translation2d(Math.max(x1, x2), Math.min(y1, y2));

        Logger.recordOutput(
                "Detection/ROI",
                new Pose2d[] {
                    new Pose2d(a, Rotation2d.kZero),
                    new Pose2d(b, Rotation2d.kZero),
                    new Pose2d(c, Rotation2d.kZero),
                    new Pose2d(d, Rotation2d.kZero),
                    new Pose2d(a, Rotation2d.kZero)
                });
    }

    @Override
    public void periodic() {
        // Update the inputs data structure associated with this Object Detection camera with latest
        // readings
        objectDetection.periodic();
        // Calculate corrected ROI corners based on current alliance and starting position
        List<Translation2d> correctedCorners = getCorrectedROI();
        // Now that inputs are updated, re-populate observations with new data
        if (objectDetection.getTargets().length > 0) {
            // Generate latest ML Object observations within the corrected ROI
            latestObjectObservation =
                    Arrays.stream(objectDetection.getTargets())
                            .map(this::generateObjectObservation)
                            .filter(
                                    obs ->
                                            isObjectWithinROI(
                                                    obs,
                                                    correctedCorners.get(0),
                                                    correctedCorners.get(1)))
                            .toList();
            // Generate latest Contour observations
            latestBigContourObservation = generateContourObservation(ContourSelectionMode.LARGEST);
            latestLowContourObservation = generateContourObservation(ContourSelectionMode.LOWEST);
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
        logROI(correctedCorners);
    }
}
