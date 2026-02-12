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

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import java.util.Optional;
import java.util.OptionalDouble;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.objectdetector.ObjectDetector;
import lombok.Getter;
import frc.lib.devices.ObjectDetection.ContourSelectionMode;
import frc.lib.devices.ObjectDetection.ObjectDetectionObservation;
import frc.lib.util.LoggedTuneableProfiledPID;

/**
 * Strategy layer that determines robot heading required to align robot with centroid of detected
 * contour. Communal for both teleop &amp; auto. Logs outputs for sim.
 */
public abstract class AlignToObjectBase {
    @Getter
    private final LoggedTuneableProfiledPID angularController;
    private final ObjectDetector objectDetector;
    private final ContourSelectionMode mode;
    private final double maxAngularSpeed; // rad/s
    private static final double K_P = 8.3;
    private static final double K_I = 0.0;
    private static final double K_D = 1.0;
    private double contourYaw;
    private boolean hasTarget;

    /**
     * Constructs an AlignToObjectBase with the given parameters.
     *
     * @param objectDetector the object detector subsystem
     * @param mode the contour selection mode (LARGEST or LOWEST)
     * @param maxAngularSpeed the maximum angular speed in rad/s
     * @param maxAngularAcceleration the maximum angular acceleration in rad/s^2
     */
    public AlignToObjectBase(ObjectDetector objectDetector, ContourSelectionMode mode,
        double maxAngularSpeed, double maxAngularAcceleration) {
        this.objectDetector = objectDetector;
        this.mode = mode;
        this.maxAngularSpeed = maxAngularSpeed;
        this.angularController =
            new LoggedTuneableProfiledPID("AlignToObject", K_P, K_I, K_D, maxAngularSpeed,
                maxAngularAcceleration);
        hasTarget = false;
    }

    /**
     * Generate angular velocity required to match the robot's heading to the centroid of the
     * detected object.
     *
     * @return Error-reduced angular velocity (rad/s).
     */
    protected OptionalDouble getVisionOmega() {
        Optional<ObjectDetectionObservation> contourObservation;

        switch (mode) {
            case LARGEST:
                contourObservation = objectDetector.getLatestBigContourObservation();
                break;
            case LOWEST:
                contourObservation = objectDetector.getLatestLowContourObservation();
                break;
            default:
                contourObservation = objectDetector.getLatestBigContourObservation();
                break;
        }
        // If Object Detection camera sees no targets or record fails to generate
        if (contourObservation.isEmpty()) {
            if (hasTarget) {
                // If camera loses sight of target temporarily, assume heading is approximately
                // equal to prevent derivative kick
                angularController.reset(contourYaw, 0.0);
            } else {
                angularController.reset(0.0, 0.0);
            }
            hasTarget = false;
            logObjectAlign(contourObservation, -9999.0);
            return OptionalDouble.empty();
        }
        // Object Detection camera sees target(s) and record successfully generates
        hasTarget = true;
        contourYaw = contourObservation.get().yaw().in(Radians);

        // Alignment PID: contourYaw is the heading measurement (feedback), 0.0 is the setpoint;
        // the controller output omega is the angular velocity command (CV)
        double omega = angularController.calculate(contourYaw, 0.0);
        omega = MathUtil.clamp(omega, -maxAngularSpeed, maxAngularSpeed);
        logObjectAlign(contourObservation, omega);
        return OptionalDouble.of(omega);
    }

    // Private helper for sim logging
    private void logObjectAlign(Optional<ObjectDetectionObservation> observation,
        double speedDegPerSec) {
        // Log for sim
        if (observation.isPresent()) {
            Logger.recordOutput("VisionAlign/" + "ContourYawDeg",
                observation.get().yaw().in(Degrees));
        } else {
            Logger.recordOutput("VisionAlign/" + "ContourYawDeg",
                -9999.0);
        }
        Logger.recordOutput("VisionAlign/" + "OmegaCmdDegPerSec", speedDegPerSec);
        Logger.recordOutput("VisionAlign/" + "HasTarget", hasTarget);
    }

    /**
     * Checks if the robot is aligned within the specified tolerance.
     *
     * @param tolRad the tolerance in radians
     * @return true if aligned within tolerance, false otherwise
     */
    protected Boolean isAligned(double tolRad) {
        return (hasTarget && Math.abs(contourYaw) < tolRad);
    }
}
