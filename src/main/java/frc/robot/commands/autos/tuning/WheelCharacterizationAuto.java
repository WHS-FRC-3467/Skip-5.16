// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.tuning;

import frc.lib.util.AutoRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;

/** Autonomous routine for characterizing wheel radius. Used for tuning drivetrain odometry. */
public class WheelCharacterizationAuto extends AutoRoutine {
    /**
     * Constructs a WheelCharacterizationAuto that runs wheel radius characterization routine.
     *
     * @param drive the drive subsystem to characterize
     */
    public WheelCharacterizationAuto(Drive drive) {
        loadCommands(DriveCommands.wheelRadiusCharacterization(drive));
    }
}
