// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.lib.util.FieldUtil;
import frc.robot.RobotState;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.tower.Tower.State;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intakeLinear.IntakeLinear;
import frc.robot.subsystems.intakeRoller.IntakeRoller;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.tower.Tower;

/**
 * Class containing useful individual commands or small-group command sequences that can be strung
 * together into larger command units (AutoSegments). Command logic layer.
 */
public class AutoCommands {

    /**
     * Resets the robot's odometry to the starting pose of the specified path. Handles alliance
     * flipping if necessary. ONLY RUNS IN SIMULATION.
     *
     * @param drive the drive subsystem
     * @param path the PathPlanner path containing the starting pose
     * @return a command that resets the robot's pose to the path's starting position
     */
    public static Command resetSimOdom(Drive drive, PathPlannerPath path)
    {
        final RobotState robotState = RobotState.getInstance();
        if (RobotBase.isSimulation()) {
            return drive.runOnce(
                () -> {
                    Pose2d pose =
                        path.getStartingHolonomicPose().get();
                    if (FieldUtil.shouldFlip()) {
                        pose = FieldUtil.handleAllianceFlip(pose);
                    }

                    robotState.resetPose(pose);
                });

        } else {
            return Commands.none();
        }
    }

    /**
     * Creates a command sequence to shoot fuel from the robot. Spins up the shooter, only pulls
     * fuel through the indexer when ready, and then stops tower & indexer afterwards (shooter
     * remains spun up).
     *
     * @param intakeLinear the intake linear subsystem
     * @param indexer the indexer subsystem
     * @param tower the tower subsystem
     * @param shooter the shooter superstructure
     * @param duration the maximum duration in seconds to run the shooting sequence
     * @return a command that shoots fuel and then stops the indexer
     */
    public static Command shootFuel(IntakeLinear intakeLinear, Indexer indexer, Tower tower,
        ShooterSuperstructure shooter, double duration)
    {
        return Commands.sequence(
            new ParallelDeadlineGroup(
                Commands.waitUntil(shooter.readyToShoot()), // Delay shooting until ready
                shooter.spinUpShooter()),
            new ParallelDeadlineGroup(
                Commands.waitSeconds(duration),
                shooter.spinUpShooter(), // Keep shooter scheduled
                tower.holdStateUntilInterrupted(Tower.State.SHOOT))
                    .onlyWhile(shooter.readyToShoot()),
            intakeLinear.retract(), // Return intake to stowed position after completing the cycle
            indexer.holdStateUntilInterrupted(Indexer.State.STOP));
    }

    /**
     * automatically when the command ends.
     *
     * @param intake the rotary intake subsystem
     * @return a command that runs the intake and stops it when finished
     */
    public static Command runIntake(IntakeRoller intake)
    {
        return intake.runIntake(IntakeRoller.State.INTAKE).finallyDo(() -> intake.stop());
    }

    /**
     * Creates a command to deploy the intake linearly such that the rotary intake can begin to
     * collect game pieces. This command is blocking (2s max) until the deployment is complete. The
     * intake will remain extended at the conclusion of this command.
     * 
     * @param intake the linear intake subsystem
     * @return a command that deploys the intake and keeps it deployed when finished
     */
    public static Command deployIntake(IntakeLinear intake)
    {
        return Commands.sequence(
            intake.extend(),
            Commands.waitUntil(intake.linearStopped).withTimeout(2.0)); // Wait until slam or max
    }

    /**
     * Creates a command to retract the intake linearly such that the robot can reduce its swept
     * volume. This command is blocking (2s max) until the retraction is complete. The intake will
     * remain retracted at the conclusion of this command.
     * 
     * @param intake the linear intake subsystem
     * @return a command that retracts the intake and keeps it retracted when finished
     */
    public Command retractIntake(IntakeLinear intake)
    {
        return Commands.sequence(
            intake.retract(),
            Commands.waitUntil(intake.linearStopped).withTimeout(2.0)); // Wait until slam or max
    }
}
