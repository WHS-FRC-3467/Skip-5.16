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

package frc.robot.commands.autos;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.devices.ObjectDetection;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.objectdetector.ObjectDetector;
import frc.lib.devices.ObjectDetection.ContourSelectionMode;
import frc.lib.devices.ObjectDetection.ObjectDetectionObservation;
import frc.lib.util.LoggedTuneableProfiledPID;

public class AlignToObject extends Command {
    private final Drive drive;
    private final ObjectDetector objectDetector;
    private final ContourSelectionMode mode;
    private final LoggedTuneableProfiledPID angularController;
    private final Velocity maxAngularVelocity;

    public AlignToObject(Drive drive, ObjectDetector objectDetector, ContourSelectionMode mode,
        LoggedTuneableProfiledPID angularController, Velocity maxAngularVelocity)
    {
        this.drive = drive;
        this.objectDetector = objectDetector;
        this.mode = mode;
        this.angularController = angularController;
        this.maxAngularVelocity = maxAngularVelocity;

        angularController.enableContinuousInput(-Math.PI, Math.PI);


        ObjectDetectionObservation obj;
    }

    @Override
    public void initialize()
    {}

    @Override
    public void execute()
    {
        drive.setOverride(pidOutput)
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
