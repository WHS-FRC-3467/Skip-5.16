// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import frc.lib.util.AutoRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;

public class WheelCharacterizationAuto extends AutoRoutine {
    public WheelCharacterizationAuto(Drive drive)
    {
        loadCommands(DriveCommands.wheelRadiusCharacterization(drive));
    }
}
