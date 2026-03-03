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
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.posestimator.PoseEstimator.VisionPoseObservation;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

/**
 * Attempts to characterize vision and odometry covariance diagonals for use in WPILib Kalman
 * filter. Recommended for use with controlled motion sequences.
 */
public class VisionOdometryCharacterizer {
    private static final RobotState robotState = RobotState.getInstance();

    /** Static threshold for tuning */
    private static final int MINIMUM_VISION_SAMPLE_SIZE = 2000;

    /** Vision standard error threshold for sampling */
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

    /** Odometry process noise tracking */ // TODO

    public static void enable() {
        enabled = true;
    }

    public static void disable() {
        enabled = false;
    }

    public static void reset() {
        nVis = 0;
        meanX = meanY = meanTheta = 0.0;
        m2X = m2Y = m2Theta = 0.0;
    }

    public static boolean isEnabled() {
        return enabled;
    }

    // ----------------------
    // Vision Noise (R) Tracking
    // ----------------------

    /**
     * Record a state estimator prediction and vision measurement delta to calculate innovation and
     * track variance.
     *
     * @param predictedPose pose predicted by state estimator (model) at time t.
     * @param observation vision observation containing pure vision pose at time t.
     */
    public static void recordVisionCorrection(
            Pose2d predictedPose, VisionPoseObservation observation) {
        if (!validVisionMeasurement(predictedPose, observation)) return;

        Transform2d innovation = observation.robotPose().minus(predictedPose);
        double errX = innovation.getX();
        double errY = innovation.getY();
        double errTheta =
                MathUtil.angleModulus(
                        innovation.getRotation().getRadians()); // Prevent wrapping issues

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

    private static boolean validVisionMeasurement(
            Pose2d predictedPose, VisionPoseObservation observation) {
        if (!enabled || predictedPose == null || observation.robotPose() == null) {
            return false;
        }

        ChassisSpeeds vel = robotState.getFieldRelativeVelocity();
        if (Math.hypot(vel.vxMetersPerSecond, vel.vyMetersPerSecond) > 0.1
                || Math.abs(vel.omegaRadiansPerSecond) > 0.1) {
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
        if (odoTimestep < 0.0) return;

        String prefix = "Pose Stats/";
        Logger.recordOutput(prefix + "VisionSigmaX", getVisionStdDevX());
        Logger.recordOutput(prefix + "VisionSigmaY", getVisionStdDevY());
        Logger.recordOutput(prefix + "VisionSigmaTheta", getVisionStdDevTheta());
    }
}
