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

import static edu.wpi.first.units.Units.Radians;
import java.util.Optional;
import java.util.OptionalDouble;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.objectdetector.ObjectDetector;
import frc.lib.devices.ObjectDetection.ContourSelectionMode;
import frc.lib.devices.ObjectDetection.ObjectDetectionObservation;
import frc.lib.util.LoggedTuneableProfiledPID;

/**
 * Strategy layer that determines robot heading required to align robot with centroid of detected
 * contour. Communal layer for both teleop & auto.
 */
public abstract class AlignToObjectBase {
    private final ObjectDetector objectDetector;
    private final ContourSelectionMode mode;
    private final LoggedTuneableProfiledPID angularController;
    private final double maxAngularVelocityRadPerSec;
    private double contourYaw;
    private boolean hadTarget = false;

    public AlignToObjectBase(ObjectDetector objectDetector, ContourSelectionMode mode,
        LoggedTuneableProfiledPID angularController, double maxAngularVelocityRadPerSec)
    {
        this.objectDetector = objectDetector;
        this.mode = mode;
        this.angularController = angularController;
        this.maxAngularVelocityRadPerSec = maxAngularVelocityRadPerSec;

        angularController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Generate CV required to match the robot's heading to the centroid of the detected object.
     * 
     * @return Error-reduced angular velocity (rad/s).
     */
    protected OptionalDouble getVisionOmega()
    {
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

        if (contourObservation.isEmpty()) {
            if (hadTarget) {
                angularController.reset(contourYaw, 0.0);
            }
            hadTarget = false;
            return OptionalDouble.empty();
        }
        hadTarget = true;
        contourYaw = contourObservation.get().yaw().in(Radians);

        // PhotonVision positive right, WPILib positive left
        double omega = angularController.calculate(-contourYaw, 0.0);
        omega = MathUtil.clamp(omega, -maxAngularVelocityRadPerSec, maxAngularVelocityRadPerSec);

        return OptionalDouble.of(omega);
    }

    protected Boolean isAligned(double tolRad)
    {
        return (hadTarget && Math.abs(contourYaw) < tolRad);
    }
}
