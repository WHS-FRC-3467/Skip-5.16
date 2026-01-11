// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.Collections;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.util.AutoCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;

public class WheelCharacterizationAuto extends AutoCommand {
    public WheelCharacterizationAuto(Drive drive)
    {
        addCommands(DriveCommands.wheelRadiusCharacterization(drive));
    }

    @Override
    public List<Pose2d> getAllPathPoses()
    {
        return Collections.emptyList();
    }

    @Override
    public Pose2d getStartingPose()
    {
        return new Pose2d();
    }
}
