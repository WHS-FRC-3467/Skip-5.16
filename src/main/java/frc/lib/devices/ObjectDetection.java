/*
 * Copyright (C) 2025 Windham Windup
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

package frc.lib.devices;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.lib.io.objectdetection.ObjectDetectionIO;
import frc.lib.io.objectdetection.ObjectDetectionIO.ObjectDetectionIOInputs;
import static edu.wpi.first.units.Units.Degrees;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * Represents a single Object Detection camera on the robot.
 * 
 * <p>
 * Handles interfacing with the {@link ObjectDetectionIO} hardware layer. While the IO (hardware)
 * layer is responsible for defining the variables of interest coming from our camera, this device
 * layer is responsible for periodically polling that IO and performing relevant calculations on the
 * return results to generate data for the robot to make decisions. Contains methods useful for both
 * ML object detection as well as HSV Color detection.
 */
public class ObjectDetection {
    // Inputs data structure
    private final ObjectDetectionIOInputs inputs =
        new ObjectDetectionIOInputs();
    // IO implementation of ObjectDetectionIO (how inputs data structure is populated)
    private final ObjectDetectionIO io;

    /**
     * Represents an Object Detection observation.
     * 
     * <p>
     * These values are a combination of baseline return values from the camera and device-level
     * calculations using those basic values. This structure can represent essential observations
     * from an ML or HSV Color Detection pipeline.
     * 
     * @param objID Object ID (from ML pipeline only).
     * @param confidence Object ID confidence (from ML pipeline only).
     * @param pitch Pitch of the object relative to the centerline of the camera.
     * @param yaw Yaw of the object relative to the centerline of the camera.
     * @param area Area of the object in the image.
     * @param distance Approximate 2d robot-relative distance to the detected object.
     * @param objectPose Estimated field-relative pose of the detected object.
     */
    public record ObjectDetectionObservation(int objID, double confidence, Angle pitch, Angle yaw,
        double area, Optional<Distance> distance, Optional<Pose2d> objectPose) {
    }

    /*
     * Interface as a data type allows ObjectDetection to accept various implementations of
     * ObjectDetectionIO (e.g. ObjectDetectionIOPhotonVision or ObjectDetectionIOLimelight).
     * Currently factored for PhotonVision only.
     */
    public ObjectDetection(ObjectDetectionIO io)
    {
        this.io = io;
    }

    /*
     * Periodically retrive most recent ObjectDetection pipeline results and populate into inputs.
     */
    public void periodic()
    {
        io.updateInputs(inputs);
        Logger.processInputs(io.getCamera(), inputs);
    }

    /*
     * Returns the latestPhotonTrackedTarget field (array of PhotonTrackedTargets) of inputs. Result
     * is an array of object information (e.g. skew, yaw, objID, etc.) from latest pipeline result.
     * Each index contains information for a single detected object.
     */
    public PhotonTrackedTarget[] getTargets()
    {
        return inputs.latestTargets;
    }

    /**
     * Uses an empirical curve fit of area to estimate distance (in.). deltaS = a*area^3 + b*area^2
     * + c*area + d. Cubic fit required to better match governing physics (tan(x) based). Determine
     * fit coefficients from calibration procedure.
     * 
     * @param target A data type containing vision pipeline results for a single object.
     * @param a Coefficient for cubic term in curve fit equation.
     * @param b Coefficient for quadratic term in curve fit equation.
     * @param c Coefficient for linear term in curve fit equation.
     * @param d Coefficient for constant term in curve fit equation.
     * @return The estimated range to the object in meters.
     */
    public double rangeToTarget_SingleFactorArea(PhotonTrackedTarget target,
        float a, float b, float c, float d)
    {
        return (a * Math.pow(target.getArea(), 3)
            + b * Math.pow(target.getArea(), 2) + c * target.getArea() + d);
    }

    /**
     * Uses the camera's focal length & trig to estimate range from target; requires no measurement
     * of pitch. Utilizes pinhole model of a camera. Note that camera focal length in pixels = (P *
     * D) / H, where P = perceived width of known object (px), D = known distance from camera (in.),
     * H = known height of object (in.). For unreliable corner detection or object digital height
     * calculation, use rangeToTarget_SingleFactorArea.
     * 
     * @param target A data type containing vision pipeline results for a single object.
     * @param objectPhysicalHeightMeters The physical height of the object being targeted in meters.
     * @param cameraFocalLengthPixels The camera focal length in pixels as determined by a
     *        calibration procedure.
     * @param cameraCalFactor An empirical calibration factor to account for real lens effects (e.g.
     *        blur, distortion, focus).
     * @return The estimated range to the object in meters.
     */
    public double rangeToTarget_FocalLength(PhotonTrackedTarget target,
        double objectPhysicalHeightMeters, double cameraFocalLengthPixels, double cameraCalFactor)
    {
        // Return & sort corners to estimate detected object's digital height in pixels.
        double[] objectDigitalCorners_px =
            {target.getDetectedCorners().get(0).y, target.getDetectedCorners().get(1).y,
                    target.getDetectedCorners().get(2).y, target.getDetectedCorners().get(3).y};
        Arrays.sort(objectDigitalCorners_px);
        // Calculate object's digital height in pixels.
        double objectPhysicalHeightPixels =
            objectDigitalCorners_px[objectDigitalCorners_px.length - 1]
                - objectDigitalCorners_px[0];
        // Return estimated range to object in meters.
        return (cameraCalFactor
            * ((objectPhysicalHeightMeters * cameraFocalLengthPixels)
                / objectPhysicalHeightPixels));
    }

    /**
     * Estimates robot's range to a target using the target's known height. Algorithm similar to
     * {@link org.photonvision.PhotonUtils} but also allows for camera installation yaw (at the
     * expense of accuracy, particularly when operating around the angular limits of the camera's
     * FOV). This method can produce more stable results than SolvePNP when well tuned, if the full
     * 6d robot pose is not required. Note that this method requires the camera to have 0 roll (not
     * be skewed clockwise or CCW relative to the floor), and for there to exist a height
     * differential between goal and camera. The larger this differential, the more accurate the
     * distance estimate will be. For very small differentials, use rangeToTarget_FocalLength.
     *
     * @param target A data type containing vision pipeline results for a single target. Used to
     *        determine the pitch & yaw of the target from the centerline of the camera's lens in
     *        degrees; centerline assumed through geometric center of conical FOV. Target pitch is
     *        positive above centerline and target yaw is positive right of centerline.
     * @param cameraTransform Transform3d of the camera relative to the robot. Used to determine the
     *        camera's height off the ground, the range offset, installation pitch, and installation
     *        yaw.
     * @param targetHeightMeters The physical height of the target off the floor as measured by the
     *        location of the detection reticle in meters. For example, if your detection reticle is
     *        set to the center of the detected object's bounding box, this height should be the
     *        elevation off the ground to the center of the target.
     * @param cameraCalFactor An empirical calibration factor to account for real lens effects (e.g.
     *        blur, distortion, focus).
     * @param cameraOffset An empirical calibration factor to account for bias in the range estimate
     *        as a result of either camera hardware or installation.
     * @return The estimated robot range to the target in meters.
     */
    public double rangeToTarget_Pitch(PhotonTrackedTarget target,
        Transform3d cameraTransform, double targetHeightMeters, double cameraCalFactor,
        double cameraOffset)
    {
        // Empirically-determined tolerance (m)
        // Below which, height differential is too small for algorithm to be reliable.
        double tolerance = 0.175;
        // Salient camera transform parameters
        // X offset of camera from robot center (apply as offset to range calculation).
        double cameraRangeDelta = cameraTransform.getX();
        // Z offset of camera from robot center (used within range calculation).
        double cameraHeightMeters = cameraTransform.getZ();
        // Camera installation pitch math uses positive up but .getY() is positive down.
        double cameraPitchRadians = -cameraTransform.getRotation().getY();
        // Camera installation yaw math uses positive left and .getZ() uses the same.
        double cameraYawRadians = cameraTransform.getRotation().getZ();
        // When camera yaw is applied, the pitch triangle becomes non-orthogonal, requiring a
        // correction.
        double yawCorrection;
        if (Math.abs(targetHeightMeters - cameraHeightMeters) > tolerance) {
            if (cameraYawRadians == 0) {
                // Mathematically verified for camera pitched up or down with target above or below
                // lens centerline. This is the most robust configuration.
                // Mathematical approach: object projection onto lens centerline plane.
                yawCorrection = 1;
            } else {
                // Empirical, algebraic correction; it's workable but accuracy is relatively limited
                // and further degrades at extreme angles (relative to the camera's FOV). Pose
                // estimation or vector transforms are more appropriate but not implemented.
                // Mathematical approach: lens centerline projection onto object/lens-center plane.
                yawCorrection =
                    Math.cos(Math.abs(cameraYawRadians - Math.toRadians(target.getYaw())));
            }
            return (((targetHeightMeters - cameraHeightMeters)
                / Math.tan(cameraPitchRadians + Math.toRadians(target.getPitch())))
                * yawCorrection * cameraCalFactor + cameraOffset + cameraRangeDelta);

        } else {
            // Use rangeToTarget_FocalLength.
            return -1.0d;
        }
    }

    /**
     * Estimates robot's heading to a target using the target's robot-relative calculated range.
     * Allows for camera installation yaw (at the expense of accuracy, particularly when operating
     * around the angular limits of the camera's FOV). Note that this method requires the camera to
     * have 0 roll (not be skewed clockwise or CCW relative to the floor), and for there to exist a
     * finite range between goal and camera. The larger this differential, the more accurate the
     * distance estimate will be.
     *
     * @param target A data type containing vision pipeline results for a single target. Used to
     *        determine the yaw of the target from the centerline of the camera's lens in degrees;
     *        centerline assumed through geometric center of conical FOV. Target pitch is positive
     *        above centerline and target yaw is positive right of centerline.
     * @param cameraTransform Transform3d of the camera relative to the robot. Used to determine the
     *        camera's heading offset and installation yaw.
     * @param targetRangeMeters Robot's range to the target in meters.
     * @param cameraCalFactor An empirical calibration factor to account for real lens effects (e.g.
     *        blur, distortion, focus).
     * @param cameraOffset An empirical calibration factor to account for bias in heading estimate
     *        as a result of either camera or installation.
     * @return The estimated robot heading to the target in meters. Positive heading = robot local X
     *         axis right of target; negative heading = robot local X axis left of target.
     */
    public double headingToTarget_Yaw(PhotonTrackedTarget target,
        Transform3d cameraTransform, double targetRangeMeters, double cameraCalFactor,
        double cameraOffset)
    {
        // Salient camera transform parameters
        // Camera's range to target (math utilizes camera's range, not robot's).
        double cameraRangeMeters = targetRangeMeters - cameraTransform.getX();
        // Y offset of camera from robot center (apply as offset to heading calculation).
        double cameraHeadingDelta = cameraTransform.getY();
        // Camera installation yaw math uses positive left and .getZ() uses the same.
        double cameraYawRadians = cameraTransform.getRotation().getZ();
        // Mathematically verified for target left or right of centerline & camera yawed left or
        // right; no sign correction required.
        return (Math.tan(cameraYawRadians - Math.toRadians(target.getYaw()))
            * cameraRangeMeters * cameraCalFactor + cameraOffset + cameraHeadingDelta);
    }

    /**
     * Estimates the target's 2d distance from the robot using target's robot-relative range and
     * heading.
     *
     * @param targetRangeMeters Robot's range to the target in meters.
     * @param targetHeadingMeters Robot's heading to the target in meters.
     * @return The estimated 2d distance from the robot to the target in meters.
     */
    public double distanceToTarget2d(double targetRangeMeters, double targetHeadingMeters)
    {
        // Distance from robot to target
        return Math.sqrt((Math.pow(targetRangeMeters, 2) + Math.pow(targetHeadingMeters, 2)));
    }

    /**
     * Estimates a {@link Translation2d} that estimates the detected target's position in field
     * coordinates. This function effectively transforms the robot's pose by its local dX (range) &
     * dY (heading) to the target; the robot local coodinate system is a frame attached to the robot
     * "center" (the point upon which the camera transform is applied to). Note that the gyro angle
     * provided *must* line up with the field coordinate system -- that is, it should read zero
     * degrees when pointed towards the opposing alliance station, and increase as the robot rotates
     * CCW.
     *
     * @param targetRangeMeters Robot's range to the target is meters (dX in robot local).
     * @param targetHeadingMeters Robot's heading to the target in meters (dY in robot local).
     * @param robotPose The 2D pose of the robot on the field.
     * @return A Translation2d of the detected object in field coordinates.
     */
    public Translation2d estimateTargetToField(double targetRangeMeters,
        double targetHeadingMeters, Pose2d robotPose)
    {
        Translation2d fieldToTargetTranslation = robotPose
            .transformBy(new Transform2d(targetRangeMeters, targetHeadingMeters, new Rotation2d()))
            .getTranslation();
        return fieldToTargetTranslation;
    }

    /**
     * Generates an N-element FIFO list of the last N object poses detected by the camera. 0th index
     * represents the oldest detection (i.e. start of the list), N-1th index represents the most
     * recent detection (i.e. end of the list). If a detection is deemed a repeat (according to the
     * passed Translation2D tolerance), it is removed from its current location in robot memory and
     * re-added to the end of the list.
     * 
     * @param N The number of last detections to store in memory.
     * @param lastNDetections The list of Translation2d objects representing the camera's memory of
     *        last N detections.
     * @param toleranceMeters The tolerance in meters for determining whether a detection is new or
     *        old.
     * @param targetTranslation The Translation2d of the current target detection to be evaluated.
     */
    public void updateObservationPoseBuffer(int N,
        List<Translation2d> lastNDetections, double toleranceMeters,
        Translation2d targetTranslation)
    {
        Translation2d currentTranslation;
        boolean isNewDetection = true;
        double repeatIndex = 0;
        // Loop through robot detection memory to determine if the current detection is new.
        for (int i = 0; i < lastNDetections.size(); i++) {
            currentTranslation = lastNDetections.get(i);
            if ((Math.abs(targetTranslation.getX() - currentTranslation.getX()) <= toleranceMeters)
                && (Math
                    .abs(
                        targetTranslation.getY() - currentTranslation.getY()) <= toleranceMeters)) {
                isNewDetection = false;
                repeatIndex = i;
            }
        }
        // If the detection is new and the list is full, remove the oldest (index 0). Removal of 0
        // shifts list to the left. Add new detection to the end of the list (index N-1).
        if (isNewDetection && lastNDetections.size() >= N) {
            lastNDetections.remove(0);
            lastNDetections.add(N - 1, targetTranslation);
        }
        // If the detection is new and the list isn't full, add it to the end of the list.
        else if (isNewDetection && lastNDetections.size() < N) {
            lastNDetections.add(targetTranslation);
        }
        // If the detection is old, remove it from where it is in the list, shift everything after
        // it left, and re-add it to the end of the list.
        else {
            lastNDetections.remove((int) repeatIndex);
            lastNDetections.add(targetTranslation);
        }
    }

    /**
     * Returns contour (blob) heading by specifiying LARGEST (largest area) or LOWEST (smallest
     * pitch) target and retrieving the basic camera observation best meeting that criterion.
     * 
     * @param targets An array of PhotonTrackedTargets, likely from ObjectDetection.getTargets().
     * @param selection An enum representing the two selection modes: LARGEST or LOWEST.
     * @return An optional ObjectDetectionObservation representing the image observation best
     *         meeting the selection criterion.
     */
    public Optional<ObjectDetectionObservation> getContourObservation(PhotonTrackedTarget[] targets,
        ContourSelectionMode selection)
    {
        if (targets == null || targets.length == 0) {
            return Optional.empty();
        }
        PhotonTrackedTarget selectedTarget;
        switch (selection) {
            case LARGEST:
                selectedTarget = getLargestContour(targets);
                if (selectedTarget == null) {
                    return Optional.empty();
                } else {
                    return Optional.ofNullable(
                        new ObjectDetectionObservation(selectedTarget.getDetectedObjectClassID(),
                            selectedTarget.getDetectedObjectConfidence(),
                            Degrees.of(selectedTarget.getPitch()),
                            Degrees.of(selectedTarget.getYaw()), selectedTarget.getArea(),
                            Optional.empty(), Optional.empty()));
                }
            case LOWEST:
                selectedTarget = getLowestContour(targets);
                if (selectedTarget == null) {
                    return Optional.empty();
                } else {
                    return Optional.ofNullable(
                        new ObjectDetectionObservation(selectedTarget.getDetectedObjectClassID(),
                            selectedTarget.getDetectedObjectConfidence(),
                            Degrees.of(selectedTarget.getPitch()),
                            Degrees.of(selectedTarget.getYaw()), selectedTarget.getArea(),
                            Optional.empty(), Optional.empty()));
                }
            default:
                return Optional.empty();
        }
    }

    // Singleton selector for returnContour().
    public enum ContourSelectionMode {
        LARGEST,
        LOWEST
    }

    // Private helper for returnContour(). Finds blob with largest area.
    private PhotonTrackedTarget getLargestContour(PhotonTrackedTarget[] result)
    {
        PhotonTrackedTarget largestTarget = null;
        double maxArea = 0.0;

        for (PhotonTrackedTarget target : result) {
            if (target.getArea() > maxArea) {
                maxArea = target.getArea();
                largestTarget = target;
            }
        }
        return largestTarget;
    }

    // Private helper for returnContour(). Finds blob with smallest pitch.
    private PhotonTrackedTarget getLowestContour(PhotonTrackedTarget[] result)
    {
        PhotonTrackedTarget lowestTarget = null;
        double smallestPitch = 90.0;

        for (PhotonTrackedTarget target : result) {
            if (target.getPitch() < smallestPitch) {
                smallestPitch = target.getPitch();
                lowestTarget = target;
            }
        }
        return lowestTarget;
    }

    /*
     * Return whether the camera is connected.
     */
    public boolean isConnected()
    {
        return inputs.connected;
    }
}

