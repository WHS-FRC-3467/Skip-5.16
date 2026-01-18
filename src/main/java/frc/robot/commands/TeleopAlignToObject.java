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

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.LoggedTuneableProfiledPID;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.objectdetector.ObjectDetector;
import frc.lib.devices.ObjectDetection.ContourSelectionMode;

public class TeleopAlignToObject extends Command {
    private final Drive drive;
    private final DoubleSupplier vxSupplier;
    private final DoubleSupplier vySupplier;
    private final DoubleSupplier rotSupplier;
    private final AlignToObjectBase strategy;

    public TeleopAlignToObject(Drive drive, ObjectDetector objectDetector,
        ContourSelectionMode mode, DoubleSupplier vxSupplier,
        DoubleSupplier vySupplier, DoubleSupplier rotSupplier,
        LoggedTuneableProfiledPID angularController)
    {
        this.drive = drive;
        this.vxSupplier = vxSupplier;
        this.vySupplier = vySupplier;
        this.rotSupplier = rotSupplier;

        this.strategy = new AlignToObjectBase(objectDetector, mode, angularController,
            drive.getMaxAngularSpeedRadPerSec()) {};

        addRequirements(drive);
    }

    @Override
    public void execute()
    {
        // Take translation inputs from joystick
        double vx = vxSupplier.getAsDouble();
        double vy = vySupplier.getAsDouble();
        // Implement heading strategy; if fails, fallback to joystick heading
        double omega = strategy.getVisionOmega().orElse(rotSupplier.getAsDouble());
        // Command vx, vy, omega
        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega,
            drive.robotState.getEstimatedPose().getRotation()));
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}


