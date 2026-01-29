// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
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

    /**
     * Resets the robot's odometry to the starting pose of the specified path.
     * Handles alliance flipping if necessary.
     *
     * @param drive the drive subsystem
     * @param path the PathPlanner path containing the starting pose
     * @return a command that resets the robot's pose to the path's starting position
     */
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

    /**
     * Creates a command sequence to shoot fuel from the robot.
     * Spins up the shooter, pulls fuel through the indexer when ready, and stops after the duration.
     *
     * @param indexer the indexer subsystem
     * @param shooter the shooter superstructure
     * @param duration the maximum duration in seconds to run the shooting sequence
     * @return a command that shoots fuel and then stops the indexer
     */
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

    /**
     * Creates a command to run the intake mechanism to collect game pieces.
     * The intake will stop automatically when the command ends.
     *
     * @param intake the intake subsystem
     * @return a command that runs the intake and stops it when finished
     */
    public static Command runIntake(Intake intake) {
        return intake.runIntake(Intake.State.INTAKE).finallyDo(() -> intake.stop());
    }
}
