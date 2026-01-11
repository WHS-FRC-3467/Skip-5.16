// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.FieldUtil;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;

public class AutoCommands extends SequentialCommandGroup {

    public static Command resetOdom(Drive drive, PathPlannerPath path)
    {
        final RobotState robotState = RobotState.getInstance();
        return drive.runOnce(
            () -> {
                Pose2d pose =
                    path.getStartingHolonomicPose().get();
                if (FieldUtil.shouldFlip()) {
                    pose = FieldUtil.handleAllianceFlip(pose);
                }

                robotState.resetPose(pose);
            });
    }
}
