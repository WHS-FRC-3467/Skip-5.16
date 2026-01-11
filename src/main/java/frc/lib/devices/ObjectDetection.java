// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.devices;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.lib.io.objectdetection.ObjectDetectionIO;
import frc.lib.io.objectdetection.ObjectDetectionIO.ObjectDetectionIOInputs;
import java.util.Arrays;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Device level implementation of an object detection camera. While the IO layer is responsible for
 * defining the variables of interest coming from our camera, the device layer is responsible for
 * periodically polling that IO and performing relevant calculations on the return results to
 * generate data for the robot to make decisions.
 */
public class ObjectDetection {
    // Placeholder for concrete implementation of ObjectDetectionIO.
    private final ObjectDetectionIO io;
    // DetectionMLIOInputs (e.g. skew, yaw, objID, etc.) from the AutoLog file.
    private final ObjectDetectionIOInputs inputs =
        new ObjectDetectionIOInputs();

    /*
     * Interface as a data type allows ObjectDetection to accept various implementations of
     * ObjectDetectionIO (e.g. ObjectDetectionIOPhotonVision or ObjectDetectionIOLimelight).
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
     * Generates an N-element FIFO list of the last N objects detected by the robot. 0th index
     * represents the oldest detection (i.e. start of the list), N-1th index represents the most
     * recent detection (i.e. end of the list). If a detection is deemed a repeat (according to the
     * passed Translation2D tolerance), it is removed from its current location in robot memory and
     * re-added to the end of the list.
     * 
     * @param N The number of last detections to store in memory.
     * @param lastNDetections The List of Translation2d objects representing the robot's memory of
     *        last N detections.
     * @param toleranceMeters The tolerance in meters for determining whether a detection is new or
     *        old.
     * @param targetTranslation The Translation2d of the current target detection to be evaluated.
     */
    public void getLastNDetections(int N,
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

    /*
     * Return whether the camera is connected.
     */
    public boolean isConnected()
    {
        return inputs.connected;
    }
}

