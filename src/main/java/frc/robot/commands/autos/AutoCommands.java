// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.function.BooleanSupplier;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.FieldUtil;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
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
    private static final LoggedTunableNumber SHOOT_TOLERANCE_DEGREES =
        new LoggedTunableNumber("Auto/ShootToleranceDegrees", 6.7);

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
     * Creates a command sequence to shoot fuel from the robot. Spins up the shooter, pulls fuel
     * through the indexer when ready, and stops after the duration.
     *
     * @param indexer the indexer subsystem
     * @param tower the tower subsystem
     * @param shooter the shooter superstructure
     * @param duration the maximum duration in seconds to run the shooting sequence
     * @return a command that shoots fuel and then stops the indexer
     */
    public static Command shootFuel(Indexer indexer, Tower tower, ShooterSuperstructure shooter,
        BooleanSupplier canShoot, double duration)
    {
        return Commands.sequence(
            Commands.parallel(
                // Continuously update flywheel speed and hood position
                shooter.spinUpShooter(),
                // Run the indexer while the shooter is at position
                Commands.parallel(
                    indexer.holdStateUntilInterrupted(Indexer.State.PULL),
                    tower.holdStateUntilInterrupted(Tower.State.SHOOT))
                    // Stop running if not at position
                    .onlyWhile(shooter.readyToShoot.and(canShoot))
                    .repeatedly()) // Try again
                .withTimeout(duration), // Stop after duration
            // Reset subsystems to usual states
            Commands.parallel(
                shooter.setFlywheelSpeed(RotationsPerSecond.zero()),
                indexer.setStateCommand(Indexer.State.STOP)));
    }

    /**
     * Creates a command that automatically aligns the robot to the target while executing the
     * shooting sequence. The drive will aim toward the target based on robot state, and shooting
     * will only occur when the robot is within a configured angular tolerance. The command ends
     * when the shooting sequence times out or is otherwise interrupted.
     *
     * @param drive the drive subsystem used to rotate and align the robot to the target
     * @param indexer the indexer subsystem used to feed game pieces into the shooter
     * @param tower the tower subsystem used to move game pieces toward the shooter
     * @param shooter the shooter superstructure responsible for spinning up and controlling
     *        the shooter
     * @param duration the maximum duration in seconds to run the align-and-shoot sequence
     * @return a command that aligns the robot to the target and shoots for up to the given
     *         duration
     */
    public static Command alignAndShoot(Drive drive, Indexer indexer, Tower tower,
        ShooterSuperstructure shooter,
        double duration)
    {
        final var robotState = RobotState.getInstance();
        return Commands.deadline(
            shootFuel(indexer, tower, shooter,
                () -> Math.abs(robotState.getAngleToTarget()
                    .minus(robotState.getEstimatedPose().getRotation())
                    .getDegrees()) < SHOOT_TOLERANCE_DEGREES.get(),
                duration),
            DriveCommands.joystickDriveAtAngle(drive, () -> 0.0, () -> 0.0,
                robotState::getAngleToTarget));
    }

    /**
     * Creates a command to deploy and run the intake mechanism to collect game pieces. The intake
     * will stop and retract automatically when the command ends.
     *
     * @param intake the intake subsystem
     * @param linear the intake linear subsystem
     * @return a command that runs the intake and stops it when finished
     */
    public static Command deployIntake(IntakeRoller intake, IntakeLinear linear)
    {
        return Commands.sequence(
            linear.extend(),
            intake.holdStateUntilInterrupted(IntakeRoller.State.INTAKE))
            .finallyDo(() -> CommandScheduler.getInstance().schedule(linear.retract()));
    }
}
