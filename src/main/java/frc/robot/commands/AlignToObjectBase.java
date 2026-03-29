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

import edu.wpi.first.math.MathUtil;

import frc.lib.devices.ObjectDetection.ObjectDetectionObservation;
import frc.lib.util.LoggedTunableProfiledPID;
import frc.robot.subsystems.objectdetector.ObjectDetector;

import lombok.Getter;

import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.OptionalDouble;

/**
 * Strategy layer that determines robot heading required to align robot with centroid of detected
 * object. Communal for both teleop &amp; auto. Logs outputs for sim.
 */
public abstract class AlignToObjectBase {
    @Getter private final LoggedTunableProfiledPID angularController;
    private final ObjectDetector objectDetector;
    private final double maxAngularSpeed; // rad/s
    private static final double K_P = 8.3;
    private static final double K_I = 0.0;
    private static final double K_D = 1.0;
    private double objectYaw;
    private boolean hasTarget;

    /**
     * Constructs an AlignToObjectBase with the given parameters.
     *
     * @param objectDetector the object detector subsystem
     * @param maxAngularSpeed the maximum angular speed in rad/s
     * @param maxAngularAcceleration the maximum angular acceleration in rad/s^2
     */
    public AlignToObjectBase(
            ObjectDetector objectDetector, double maxAngularSpeed, double maxAngularAcceleration) {
        this.objectDetector = objectDetector;
        this.maxAngularSpeed = maxAngularSpeed;
        this.angularController =
                new LoggedTunableProfiledPID(
                        "AlignToObject", K_P, K_I, K_D, maxAngularSpeed, maxAngularAcceleration);
        hasTarget = false;
    }

    /**
     * Generate angular velocity required to match the robot's heading to the centroid of the
     * detected object.
     *
     * @return Error-reduced angular velocity (rad/s).
     */
    protected OptionalDouble getVisionOmega() {
        Optional<ObjectDetectionObservation> targetObservation =
                objectDetector.getLatestTargetObservation();

        if (targetObservation.isEmpty()) {
            if (hasTarget) {
                angularController.reset(objectYaw, 0.0);
            } else {
                angularController.reset(0.0, 0.0);
            }
            hasTarget = false;
            logObjectAlign(targetObservation, -9999.0);
            return OptionalDouble.empty();
        }

        hasTarget = true;
        objectYaw = targetObservation.get().yaw().in(Radians);

        // Alignment PID: objectYaw is the heading measurement (feedback), 0.0 is the setpoint;
        // the controller output omega is the angular velocity command (CV)
        double omega = angularController.calculate(objectYaw, 0.0);
        omega = MathUtil.clamp(omega, -maxAngularSpeed, maxAngularSpeed);
        logObjectAlign(targetObservation, omega);
        return OptionalDouble.of(omega);
    }

    // Private helper for sim logging
    private void logObjectAlign(
            Optional<ObjectDetectionObservation> observation, double speedRadPerSec) {
        // Log for sim
        if (observation.isPresent()) {
            Logger.recordOutput(
                    "VisionAlign/" + "ObjectYawDeg", observation.get().yaw().in(Degrees));
        } else {
            Logger.recordOutput("VisionAlign/" + "ObjectYawDeg", -9999.0);
        }
        Logger.recordOutput("VisionAlign/" + "OmegaCmdRadPerSec", speedRadPerSec);
        Logger.recordOutput("VisionAlign/" + "HasTarget", hasTarget);
    }

    /**
     * Checks if the robot is aligned within the specified tolerance.
     *
     * @param tolRad the tolerance in radians
     * @return true if aligned within tolerance, false otherwise
     */
    protected Boolean isAligned(double tolRad) {
        return (hasTarget && Math.abs(objectYaw) < tolRad);
    }
}
