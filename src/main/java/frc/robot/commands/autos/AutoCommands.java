// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.lib.util.FieldUtil;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.ShooterSuperstructure;

public class AutoCommands {

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

    public static Command followPath(String pathName) {
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            DriverStation.reportError(
                "Failed to load " + pathName + " Path: " + e.getMessage(),
                e.getStackTrace());
            return Commands.none();
        }
        return AutoBuilder.followPath(path);
    }

    public static Command shootFuel(Indexer indexer, ShooterSuperstructure shooter, double duration)
    {
        return Commands.sequence(
            new ParallelDeadlineGroup(
                Commands.waitSeconds(duration),
                shooter.spinUpShooter(),
                indexer.holdStateUntilInterrupted(Indexer.State.PULL)
                    .onlyWhile(shooter.readyToShoot())),
            indexer.holdStateUntilInterrupted(Indexer.State.STOP));
    }

    public static Command runIntake(Intake intake) {
        return intake.runIntake(Intake.State.INTAKE).finallyDo(() -> intake.stop());
    }
}
