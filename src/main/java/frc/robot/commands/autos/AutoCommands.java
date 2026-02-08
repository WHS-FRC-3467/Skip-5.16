// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import java.util.function.BooleanSupplier;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.lib.util.FieldUtil;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intakeLinear.IntakeLinear;
import frc.robot.subsystems.intakeRoller.IntakeRoller;
import frc.robot.subsystems.shooter.ShooterSuperstructure;

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
        if (RobotBase.isSimulation()) {
            final RobotState robotState = RobotState.getInstance();
            return drive.runOnce(
                () -> {
                    Pose2d pose =
                        path.getStartingHolonomicPose().get();
                    if (FieldUtil.shouldFlip()) {
                        pose = FieldUtil.apply(pose);
                    }

                    robotState.resetPose(pose);
                });
        } else

        {
            return Commands.none();
        }
    }

    /**
     * Creates a command sequence that attempts to shoot fuel from the robot for duration. Spins up
     * the shooter, only pulls fuel through the feeder when ready, then stops indexer, tower, and
     * shooter after duration. If shooting is disrupted during duration because shooter readiness
     * drops, attempt a flywheel/hood adjustment and, if successful, re-commence shooting within the
     * remaining window. Unconditionally stops shot attempts after duration.
     *
     * @param indexer the indexer subsystem
     * @param tower the tower subsystem
     * @param shooter the shooter superstructure
     * @param canShoot secondary check on whether the robot is properly aligned to the target,
     *        independent of whether the the shooter is at the proper state
     * @param duration the approximate duration in seconds to run the shooting sequence
     * @return a command that shoots fuel and then stops the indexer
     */
    public static Command shootFuel(Indexer indexer, Tower tower,
        ShooterSuperstructure shooter, BooleanSupplier canShoot, double duration)
    {
        return Commands.sequence(
            // Defensively gate shooting until ready (5 scans max)
            new ParallelDeadlineGroup(
                Commands.waitUntil(shooter.readyToShoot.and(canShoot)),
                shooter.spinUpShooter()).withTimeout(0.10),
            // Shoot when ready. If readiness drops mid-cycle, adjust shooter, then resume
            new ParallelDeadlineGroup(
                Commands.waitSeconds(duration), // Unconditionally stop shooting after duration
                shooter.spinUpShooter(), // Keep shooter scheduled and updating
                Commands.repeatingSequence(
                    // Repeat shot attempts until duration timeout
                    Commands.waitUntil(shooter.readyToShoot.and(canShoot)),
                    Commands.parallel(
                        indexer.holdStateUntilInterrupted(Indexer.State.PULL),
                        tower.holdStateUntilInterrupted(Tower.State.SHOOT))
                        .until(shooter.readyToShoot.and(canShoot).negate()))))
            .finallyDo(() -> {
                // Spin shooter down, non-blocking
                CommandScheduler.getInstance()
                    .schedule(shooter.setFlywheelSpeed(RadiansPerSecond.zero()));
            });
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
     * @param shooter the shooter superstructure responsible for spinning up and controlling the
     *        shooter
     * @param duration the maximum duration in seconds to run the align-and-shoot sequence
     * @return a command that aligns the robot to the target and shoots for up to the given duration
     */
    public static Command alignAndShoot(Drive drive, Indexer indexer,
        Tower tower, ShooterSuperstructure shooter, double duration)
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

    /**
     * Creates a command to extend the intake linearly. This command is blocking (2s max) until the
     * extension is complete. The intake will remain extended and energized at the conclusion of
     * this command.
     * 
     * @param intake the linear intake subsystem
     * @return a command that retracts the intake and keeps it retracted when finished
     */
    public static Command extendIntake(IntakeLinear intake)
    {
        return Commands.sequence(
            intake.extend(),
            Commands.waitUntil(intake.isExtended).withTimeout(2.0)); // Wait until slam or max
    }

    /**
     * Creates a command to retract the intake linearly. This command is blocking (2s max) until the
     * retraction is complete. The intake will remain retracted and energized at the conclusion of
     * this command.
     * 
     * @param intake the linear intake subsystem
     * @return a command that retracts the intake and keeps it retracted when finished
     */
    public static Command retractIntake(IntakeLinear intake)
    {
        return Commands.sequence(
            intake.retract(),
            Commands.waitUntil(intake.isRetracted).withTimeout(2.0)); // Wait until slam or max
    }

    /**
     * Creates a blocking command to agitate the balls in the hopper to facilitate shooting. Should
     * be externally interrupted / run in parallel with other commands. Can be used between
     * shootFuel() commands within an AutoSegment to prepare the hopper for game piece transport.
     * Subsystems end up in unguaranteed state; be sure to decorate or sequence this call with a
     * known state control command.
     * 
     * @param intake the linear intake subsystem
     * @param tower the tower subsystem
     * @param indexer the indexer subsystem
     * @return a blocking command that agitates the balls in the hopper and stops when finished
     */
    public static Command agitateHopper(IntakeLinear intake, Tower tower, Indexer indexer,
        HopperAgitation state)
    {
        switch (state) {
            case INTAKE_CYCLE:
                return intake.cycle();
            case TOWER_PULSE:
                return Commands.repeatingSequence(
                    tower.holdStateUntilInterrupted(Tower.State.SHOOT).withTimeout(0.1),
                    tower.holdStateUntilInterrupted(Tower.State.STOP).withTimeout(0.05));
            case INDEXER_PULSE:
                return Commands.repeatingSequence(
                    indexer.holdStateUntilInterrupted(Indexer.State.PULL).withTimeout(0.1),
                    indexer.holdStateUntilInterrupted(Indexer.State.STOP).withTimeout(0.05));
            case NONE:
                return Commands.none();
            default:
                return Commands.none();
        }
    }
}
