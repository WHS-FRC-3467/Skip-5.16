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
 * Attempts to semi-autonomously characterize both vision and odometry covariance diagonals for use
 * in WPILib Kalman filter gain calculation. This utility produces baseline estimates that can be
 * further tuned empirically as necessary.
 *
 * <p>TODO: complete docstring. move multiple values and thresholds into constants. test on robot.
 */
public class VisionOdometryCharacterizer {
    private static final RobotState robotState = RobotState.getInstance();

    /** Static limits for tuning */
    private static final int MINIMUM_VISION_SAMPLE_SIZE = 2000;

    private static final int MINIMUM_ODOMETRY_SAMPLE_SIZE = 3500;

    // Vision standard error for sampling
    private static final double VISION_STANDARD_ERROR_THRESHOLD = 0.03;
    private static final double ODOMETRY_STANDARD_ERROR_THRESHOLD = 0.03;

    /** Whether to collect observations */
    private static boolean enabled = false;

    /** Vision statistics tracking */
    private static int nVis = 0;

    private static double meanVisX = 0.0;
    private static double meanVisY = 0.0;
    private static double meanVisTheta = 0.0;
    private static double m2VisX = 0.0;
    private static double m2VisY = 0.0;
    private static double m2VisTheta = 0.0;

    // Used to filter results and generate/verify variance scaling equation
    private static double meanVisDistance = 0.0;
    private static double meanNumVisTags = 0.0;

    /** Odometry statistics tracking */
    private static int nOdo = 0;

    private static double meanOdoX = 0.0;
    private static double meanOdoY = 0.0;
    private static double meanOdoTheta = 0.0;
    private static double m2OdoX = 0.0;
    private static double m2OdoY = 0.0;
    private static double m2OdoTheta = 0.0;

    private static Pose2d lastOdoPoseForP = null;
    private static Pose2d lastVisPoseForP = null;

    /** For simulation testing with configurable noise */
    private static final Random NOISE_DISTRIBUTION = new Random();

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
        double dx = errX - meanVisX;
        meanVisX += dx / nVis;
        m2VisX += dx * (errX - meanVisX);

        // Y
        double dy = errY - meanVisY;
        meanVisY += dy / nVis;
        m2VisY += dy * (errY - meanVisY);

        // Theta
        double dtheta = errTheta - meanVisTheta;
        meanVisTheta += dtheta / nVis;
        m2VisTheta += dtheta * (errTheta - meanVisTheta);

        // Distance & tags
        meanVisDistance += meanVisDistance / nVis;
        meanNumVisTags += meanNumVisTags / nVis;

        if (hasSufficientVisionSamples() && hasSufficientOdoSamples()) disable();
    }

    /**
     * Ensure the vision measurement is valid. This check should ensure the vision measurement is
     * being taken at representative conditions. The result is an estimate of R (baseline) such that
     * this characterizer produces LINEAR_STDDEV_BASELINE & ANGULAR_STDDEV_BASELINE.
     *
     * <p>Note that avgTagDistance is used as a proxy for measurement variance since variance scales
     * with distance, and numTagsUsed is used as a proxy for number of independent measurements
     * contributing to the correction since variance scales with 1/N such that sigma^2 ~ d^2 / N.
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
                || Math.abs(vel.omegaRadiansPerSecond) > 0.2) {
            return false;
        }
        // Verify measurement is within typical bounds to ensure we are characterizing expected
        // vision noise and not outliers
        if (observation.avgTagDistance() > 4
                || observation.avgTagDistance() < 1.5
                || observation.numTagsUsed() < 2
                || observation.numTagsUsed() > 4) {
            return false;
        }
        return true;
    }

    private static double getVisionStdDevX() {
        return nVis > 1 ? Math.sqrt(m2VisX / (nVis - 1)) : 0.0;
    }

    private static double getVisionStdDevY() {
        return nVis > 1 ? Math.sqrt(m2VisY / (nVis - 1)) : 0.0;
    }

    private static double getVisionStdDevTheta() {
        return nVis > 1 ? Math.sqrt(m2VisTheta / (nVis - 1)) : 0.0;
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
    // Odometry Noise (P) Tracking
    // ----------------------

    /**
     * Assume high confidence vision measurement is "ground truth" and represents the actual delta
     * traveled across time. Compare motion model (odometry) predicted delta to the "ground truth"
     * vision delta. The noise in the motion model predicted delta can be abstracted as P (process
     * noise covariance) in the Kalman filter, which is crucial for predicting Kalman filter gain.
     */
    public static void recordOdometryCorrection(
            Pose2d odometryPoseAtTimestamp, VisionPoseObservation observation) {
        if (!validOdometryMeasurement(odometryPoseAtTimestamp, observation)) return;

        // Initialization
        if (lastOdoPoseForP == null || lastVisPoseForP == null) {
            lastOdoPoseForP = odometryPoseAtTimestamp;
            lastVisPoseForP = observation.robotPose();
            return;
        }

        // Compute deltas
        Transform2d odoDelta = odometryPoseAtTimestamp.minus(lastOdoPoseForP);
        Transform2d visDelta = observation.robotPose().minus(lastVisPoseForP);

        // Estimate error in motion model
        double errX = odoDelta.getX() - visDelta.getX() + generateSimNoise();
        double errY = odoDelta.getY() - visDelta.getY() + generateSimNoise();
        double errTheta =
                MathUtil.angleModulus(
                                odoDelta.getRotation().getRadians()
                                        - visDelta.getRotation().getRadians())
                        + generateSimNoise(); // Prevent wrapping issues

        // Welford update
        nOdo++;
        // X
        double dx = errX - meanOdoX;
        meanOdoX += dx / nOdo;
        m2OdoX += dx * (errX - meanOdoX);
        // Y
        double dy = errY - meanOdoY;
        meanOdoY += dy / nOdo;
        m2OdoY += dy * (errY - meanOdoY);
        // Theta
        double dtheta = errTheta - meanOdoTheta;
        meanOdoTheta += dtheta / nOdo;
        m2OdoTheta += dtheta * (errTheta - meanOdoTheta);

        // Update history
        lastOdoPoseForP = odometryPoseAtTimestamp;
        lastVisPoseForP = observation.robotPose();

        if (hasSufficientVisionSamples() && hasSufficientOdoSamples()) disable();
    }

    /**
     * Ensure the odometry measurement is valid. This is only true if we're confident our vision
     * measurement grants us an estimate of "ground truth" at the time of the odometry measurement,
     * such that the delta between the odometry pose and vision pose is representative of odometry
     * noise.
     *
     * <p>TODO: complete docstring. udpate model to include camera FPS normalization. verify math.
     */
    private static boolean validOdometryMeasurement(
            Pose2d odometryPoseAtTimestamp, VisionPoseObservation observation) {
        // Verify enabled & valid poses
        if (!enabled || odometryPoseAtTimestamp == null || observation.robotPose() == null) {
            return false;
        }
        // Verify robot is in motion to ensure we are capturing odometry covariance (wheel slip,
        // encoder quantization, kinematic linearization, gyro noise, etc)
        ChassisSpeeds vel = robotState.getFieldRelativeVelocity();
        if (Math.hypot(vel.vxMetersPerSecond, vel.vyMetersPerSecond) < 0.3
                && Math.abs(vel.omegaRadiansPerSecond) < 1.0) {
            return false;
        }
        // Assume vision is "ground truth" only if it is a high confidence measurement (multiple
        // tags, close range)
        if (observation.avgTagDistance() > 2.75 || observation.numTagsUsed() < 3) {
            return false;
        }
        return true;
    }

    private static double getOdoStdDevX() {
        return nOdo > 1 ? Math.sqrt(m2OdoX / (nOdo - 1)) : 0.0;
    }

    private static double getOdoStdDevY() {
        return nOdo > 1 ? Math.sqrt(m2OdoY / (nOdo - 1)) : 0.0;
    }

    private static double getOdoStdDevTheta() {
        return nOdo > 1 ? Math.sqrt(m2OdoTheta / (nOdo - 1)) : 0.0;
    }

    public static int getOdoSampleSize() {
        return nOdo;
    }

    public static boolean hasSufficientOdoSamples() {
        if (nOdo < MINIMUM_ODOMETRY_SAMPLE_SIZE) return false;

        double odoStdError = 1.0 / Math.sqrt(2.0 * nOdo);
        return odoStdError < ODOMETRY_STANDARD_ERROR_THRESHOLD;
    }

    // ----------------------
    // Utility / Reporting
    // ----------------------

    public static void enable() {
        enabled = true;
    }

    public static void disable() {
        enabled = false;
    }

    public static void reset() {
        nVis = nOdo = 0;
        meanVisX = meanVisY = meanVisTheta = 0.0;
        meanVisDistance = meanNumVisTags = 0.0;
        m2VisX = m2VisY = m2VisTheta = 0.0;
        meanOdoX = meanOdoY = meanOdoTheta = 0.0;
        m2OdoX = m2OdoY = m2OdoTheta = 0.0;
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

    public static void printResults() {
        String prefix = "Pose Stats/";
        Logger.recordOutput(prefix + "VisionSigmaX", getVisionStdDevX());
        Logger.recordOutput(prefix + "VisionSigmaY", getVisionStdDevY());
        Logger.recordOutput(prefix + "VisionSigmaTheta", getVisionStdDevTheta());
        Logger.recordOutput(prefix + "VisionAvgDistance", meanVisDistance);
        Logger.recordOutput(prefix + "VisionAvgNumTags", meanNumVisTags);
        Logger.recordOutput(prefix + "VisionBiasX", meanVisX);
        Logger.recordOutput(prefix + "VisionBiasY", meanVisY);
        Logger.recordOutput(prefix + "VisionBiasTheta", meanVisTheta);

        Logger.recordOutput(prefix + "OdoSigmaX", getOdoStdDevX());
        Logger.recordOutput(prefix + "OdoSigmaY", getOdoStdDevY());
        Logger.recordOutput(prefix + "OdoSigmaTheta", getOdoStdDevTheta());
    }
}
