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
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

/**
 * Attempts to characterize vision and odometry covariance diagonals for use in WPILib Kalman
 * filter. Recommended for use with controlled motion sequences.
 */
public class VisionOdometryCharacterizer {
    /** Static threshold for tuning */
    private static final int MINIMUM_SAMPLE_SIZE = 1000;

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

    /** Odometry process noise tracking */
    private static int nOdo = 0;

    private static double meanVx = 0.0;
    private static double meanVy = 0.0;
    private static double meanOmega = 0.0;
    private static double m2Vx = 0.0;
    private static double m2Vy = 0.0;
    private static double m2Omega = 0.0;

    private static double lastOdoTimestamp = -1.0;
    private static double odoTimestep = -1.0;

    public static void enable() {
        enabled = true;
    }

    public static void reset() {
        nVis = nOdo = 0;
        meanX = meanY = meanTheta = 0.0;
        meanVx = meanVy = meanOmega = 0.0;
        m2X = m2Y = m2Theta = 0.0;
        m2Vx = m2Vy = m2Omega = 0.0;
        lastOdoTimestamp = odoTimestep = -1.0;
    }

    public static boolean isEnabled() {
        return enabled;
    }

    // ----------------------
    // Vision Noise (R) Tracking
    // ----------------------

    /** Record a state estiamtor prediction and measurement delta to calculate innovation */
    public static void recordVisionCorrection(
            double timestamp, Pose2d predictedPose, Pose2d visionPose) {
        if (!enabled || predictedPose == null || visionPose == null) return;

        Transform2d innovation = visionPose.minus(predictedPose);
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

    // ----------------------
    // Process Noise (Q) Tracking
    // ----------------------

    /** Records latest odometry twist required for process noise estimate */
    public static void recordOdometryTwist(double timestamp, Twist2d odomTwist) {
        if (!enabled || odomTwist == null) return;
        calculateTwistVariance(timestamp, odomTwist.dx, odomTwist.dy, odomTwist.dtheta);
    }

    /** Update running variance of odometry twists (velocities) */
    private static void calculateTwistVariance(
            double timestamp, double dx, double dy, double dtheta) {
        if (lastOdoTimestamp < 0.0) {
            lastOdoTimestamp = timestamp;
            return;
        }

        odoTimestep = timestamp - lastOdoTimestamp;
        lastOdoTimestamp = timestamp;

        if (odoTimestep <= 1e-6) return;

        double vx = dx / odoTimestep;
        double vy = dy / odoTimestep;
        double omega = dtheta / odoTimestep;

        // Welford update
        nOdo++;
        // Vx
        double dvx = vx - meanVx;
        meanVx += dvx / nOdo;
        m2Vx += dvx * (vx - meanVx);

        // Vy
        double dvy = vy - meanVy;
        meanVy += dvy / nOdo;
        m2Vy += dvy * (vy - meanVy);

        // omega
        double domega = omega - meanOmega;
        meanOmega += domega / nOdo;
        m2Omega += domega * (omega - meanOmega);
    }

    public static double getOdoStdDevX(double dt) {
        return nOdo > 1 ? Math.sqrt(m2Vx / (nOdo - 1)) * dt : 0.0;
    }

    public static double getOdoStdDevY(double dt) {
        return nOdo > 1 ? Math.sqrt(m2Vy / (nOdo - 1)) * dt : 0.0;
    }

    public static double getOdoStdDevTheta(double dt) {
        return nOdo > 1 ? Math.sqrt(m2Omega / (nOdo - 1)) * dt : 0.0;
    }

    // ----------------------
    // Utility / Reporting
    // ----------------------

    public static int getVisionSampleSize() {
        return nVis;
    }

    public static int getOdometrySampleSize() {
        return nOdo;
    }

    public static boolean hasSufficientSamples() {
        if (nVis < MINIMUM_SAMPLE_SIZE || nOdo < MINIMUM_SAMPLE_SIZE) return false;

        double visStdError = 1.0 / Math.sqrt(2.0 * nVis);
        double odoStdError = 1.0 / Math.sqrt(2.0 * nOdo);
        return visStdError < 0.03 && odoStdError < 0.03;
    }

    public static void printResults() {
        if (odoTimestep < 0.0) return;

        DriverStation.reportWarning(
                "Vision R Estimates: "
                        + "Sufficient Samples: "
                        + hasSufficientSamples()
                        + "\nVision sigmaX: "
                        + getVisionStdDevX()
                        + "\nVision sigmaY: "
                        + getVisionStdDevY()
                        + "\nVision sigmaTheta: "
                        + getVisionStdDevTheta()
                        + "\nOdometry sigmaX: "
                        + getOdoStdDevX(odoTimestep)
                        + "\nOdometry sigmaY: "
                        + getOdoStdDevY(odoTimestep)
                        + "\nOdometry sigmaTheta: "
                        + getOdoStdDevTheta(odoTimestep),
                false);

        String prefix = "VISION CHARACTERIZER/";
        Logger.recordOutput(prefix + "VISION_SIGMA_X", getVisionStdDevX());
        Logger.recordOutput(prefix + "VISION_SIGMA_Y", getVisionStdDevY());
        Logger.recordOutput(prefix + "VISION_SIGMA_THETA", getVisionStdDevTheta());
        Logger.recordOutput(prefix + "ODOMETRY_SIGMA_X", getOdoStdDevX(odoTimestep));
        Logger.recordOutput(prefix + "ODOMETRY_SIGMA_Y", getOdoStdDevY(odoTimestep));
        Logger.recordOutput(prefix + "ODOMETRY_SIGMA_THETA", getOdoStdDevTheta(odoTimestep));
    }
}
