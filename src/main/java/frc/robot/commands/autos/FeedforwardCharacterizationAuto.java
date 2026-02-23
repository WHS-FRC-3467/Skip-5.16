// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import frc.lib.util.AutoRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;

/** Autonomous routine for characterizing drive ff. */
public class FeedforwardCharacterizationAuto extends AutoRoutine {
    /**
     * Constructs a FeedforwardCharacterizationAuto that runs feedforward characterization routine.
     *
     * @param drive the drive subsystem to characterize
     */
    public FeedforwardCharacterizationAuto(Drive drive) {
        loadCommands(DriveCommands.feedforwardCharacterization(drive));
    }
}
