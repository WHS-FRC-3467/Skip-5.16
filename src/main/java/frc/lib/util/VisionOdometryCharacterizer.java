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

package frc.lib.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.posestimator.PoseEstimator.VisionPoseObservation;
import frc.robot.Robot;
import frc.robot.RobotState;
import java.util.Random;
import org.littletonrobotics.junction.Logger;

/**
 * Attempts to characterize vision and odometry covariance diagonals for use in WPILib Kalman
 * filter. 
 */
public class VisionOdometryCharacterizer {
    private static final RobotState robotState = RobotState.getInstance();

    /** Thresholds */
    // Static limit for tuning 
    private static final int MINIMUM_VISION_SAMPLE_SIZE = 2000;

    // Vision standard error for sampling 
    private static final double VISION_STANDARD_ERROR_THRESHOLD = 0.03;

    /** Whether to collect observations */
    private static boolean enabled = false;

    /** Vision statistics tracking */
    private static int nVis = 0;

    private static double meanX = 0.0;
    private static double meanY = 0.0;
    private static double meanTheta = 0.0;
    private static double m2X = 0.0;
    private static double m2Y = 0.0;
    private static double m2Theta = 0.0;

    // Used to filter results and generate/verify variance scaling equation
    private static double meanDistance = 0.0;
    private static double meanNumTags = 0.0;

    /** For simulation testing with configurable noise */
    private static final Random NOISE_DISTRIBUTION = new Random();

    /** Odometry statistics tracking */
    // TODO

    public static void enable() {
        enabled = true;
    }

    public static void disable() {
        enabled = false;
    }

    public static void reset() {
        nVis = 0;
        meanX = meanY = meanTheta = 0.0;
        meanDistance = meanNumTags = 0.0;
        m2X = m2Y = m2Theta = 0.0;
    }

    public static boolean isEnabled() {
        return enabled;
    }

    /** Generate noise in sim to verify functionality before deployment. */
    private static double generateSimNoise() {
        if (Robot.isSimulation()) {
            return NOISE_DISTRIBUTION.nextGaussian(0.0, 0.02);
        } else {
            return 0.0;
        }
    }

    // ----------------------
    // Vision Noise (R) Tracking
    // ----------------------

    /**
     * Record a state estimator (model) pose prediction (x_k|k-1) and vision measurement (z_k) delta
     * to calculate innovation (y_k) and track variance such that y_​k ​= z_k ​− x_k|k-1​
     * (innovation ~ measurement - model prediction at time of measurement).
     *
     * <p>This method computes the covariance of the innovation S_k = P_k|k-1 + R where P_k|k-1 is
     * the prediction covariance and R is the measurement covariance. S_k represents the total
     * uncertainty in the measurement update step of the Kalman filter, combining both the
     * uncertainty from the state prediction and the measurement noise.
     *
     * <p>Under specific conditions, tracking the variance (diagonals) of S_k allows us to
     * empirically estimate the measurement noise covariance R, which is crucial for tuning the
     * Kalman filter's performance. Specifically, this approximation is only valid when the robot is
     * approximately stationary, such that the prediction covariance P_k|k-1 is minimal and
     * innovation variance is dominated by measurement noise -- in other words, as the prediction
     * covariance P_k|k-1 is minimized, S_k becomes more reflective of R.
     *
     * @param predictedPose pose predicted by state estimator (model) at time t (x_k|k-1).
     * @param observation vision observation containing pure vision pose at time t (z_k).
     */
    public static void recordVisionCorrection(
            Pose2d predictedPose, VisionPoseObservation observation) {
        if (!validVisionMeasurement(predictedPose, observation)) return;

        Transform2d innovation = observation.robotPose().minus(predictedPose);
        double errX = innovation.getX() + generateSimNoise();
        double errY = innovation.getY() + generateSimNoise();
        double errTheta =
                MathUtil.angleModulus(innovation.getRotation().getRadians())
                        + generateSimNoise(); // Prevent wrapping issues

        // Welford update
        nVis++;
        // X
        double dx = errX - meanX;
        meanX += dx / nVis;
        m2X += dx * (errX - meanX);

        // Y
        double dy = errY - meanY;
        meanY += dy / nVis;
        m2Y += dy * (errY - meanY);

        // Theta
        double dtheta = errTheta - meanTheta;
        meanTheta += dtheta / nVis;
        m2Theta += dtheta * (errTheta - meanTheta);

        // Distance & tags
        meanDistance += meanDistance / nVis;
        meanNumTags += meanNumTags / nVis;

        if (hasSufficientVisionSamples()) disable();
    }

    private static double getVisionStdDevX() {
        return nVis > 1 ? Math.sqrt(m2X / (nVis - 1)) : 0.0;
    }

    private static double getVisionStdDevY() {
        return nVis > 1 ? Math.sqrt(m2Y / (nVis - 1)) : 0.0;
    }

    private static double getVisionStdDevTheta() {
        return nVis > 1 ? Math.sqrt(m2Theta / (nVis - 1)) : 0.0;
    }

    /**
     * Ensure the vision measurement is valid. This check should ensure the vision measurement is
     * being taken at standard conditions as defined by the assumptions of our measurement
     * covariance scaling equation. The result is an estimate of R (baseline) such that this
     * characterizer produces LINEAR_STDDEV_BASELINE & ANGULAR_STDDEV_BASELINE.
     */
    private static boolean validVisionMeasurement(
            Pose2d predictedPose, VisionPoseObservation observation) {
        // Verify enabled & valid poses
        if (!enabled || predictedPose == null || observation.robotPose() == null) {
            return false;
        }
        // Verify robot is approximately stationary to ensure innovation covariance approximates
        // measurement covariance
        ChassisSpeeds vel = robotState.getFieldRelativeVelocity();
        if (Math.hypot(vel.vxMetersPerSecond, vel.vyMetersPerSecond) > 0.1
                || Math.abs(vel.omegaRadiansPerSecond) > 0.1) {
            return false;
        }
        /** avgTagDistance is used as a proxy for measurement variance since variance scales with distance, 
         * and numTagsUsed is used as a proxy for number of independent measurements contributing to the 
         * correction since variance scales with 1/N such that sigma^2 ~ d^2 / N.
         */
        if (observation.avgTagDistance() > 4 || observation.avgTagDistance() < 1.5 || 
        observation.numTagsUsed() < 2 || observation.numTagsUsed() > 4) {
            return false;
        }
        return true;
    }

    public static int getVisionSampleSize() {
        return nVis;
    }

    public static boolean hasSufficientVisionSamples() {
        if (nVis < MINIMUM_VISION_SAMPLE_SIZE) return false;

        double visStdError = 1.0 / Math.sqrt(2.0 * nVis);
        return visStdError < VISION_STANDARD_ERROR_THRESHOLD;
    }

    // ----------------------
    // Process Noise (Q) Tracking // TODO
    // ----------------------

    // ----------------------
    // Utility / Reporting
    // ----------------------

    public static void printResults() {
        String prefix = "Pose Stats/";
        Logger.recordOutput(prefix + "VisionSigmaX", getVisionStdDevX());
        Logger.recordOutput(prefix + "VisionSigmaY", getVisionStdDevY());
        Logger.recordOutput(prefix + "VisionSigmaTheta", getVisionStdDevTheta());
        Logger.recordOutput(prefix + "VisionSigmaDistance", meanDistance);
        Logger.recordOutput(prefix + "VisionSigmaNumTags", meanNumTags);
        Logger.recordOutput(prefix + "BiasX", meanX);
        Logger.recordOutput(prefix + "BiasY", meanY);
        Logger.recordOutput(prefix + "BiasTheta", meanTheta);
    }
}
