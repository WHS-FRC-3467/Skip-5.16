// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import java.util.function.BooleanSupplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.lib.util.FieldUtil;
import frc.robot.RobotState;
import frc.robot.RobotState.Target;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.commands.FuelCommands;

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
    public static Command resetSimOdom(Drive drive, PathPlannerPath path) {
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
     * the shooter, only pulls fuel through the feeder when ready, then stops indexer and tower
     * after duration. If shooting is disrupted during duration because shooter readiness drops,
     * attempt a flywheel/hood adjustment and, if successful, re-commence shooting within the
     * remaining window. Unconditionally stops shot attempts after duration.
     *
     * @param indexer the indexer subsystem
     * @param tower the tower subsystem
     * @param shooter the shooter superstructure
     * @param canShoot secondary check on whether the robot is properly aligned to the target,
     *        independent of whether the the shooter is at the proper state
     * @param duration the approximate duration in seconds to run the shooting sequence
     * @return a command that shoots fuel and then stops the indexer / tower after the given
     *         duration
     */
    public static Command shootFuel(Indexer indexer, Tower tower,
        ShooterSuperstructure shooter, BooleanSupplier canShoot, double duration) {
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
                        indexer.shoot(),
                        tower.shoot())
                        .until(shooter.readyToShoot.and(canShoot).negate()))))
            .finallyDo(() -> {
                // Spin shooter down, non-blocking
                // CommandScheduler.getInstance()
                // .schedule(shooter.setFlywheelSpeed(RadiansPerSecond.zero()));
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
        Tower tower, ShooterSuperstructure shooter, double duration) {
        final var robotState = RobotState.getInstance();
        return Commands.deadline(
            FuelCommands.shootFuel(indexer, tower, shooter,
                robotState.facingTarget,
                duration),
            DriveCommands.joystickDriveAtAngle(drive, () -> 0.0, () -> 0.0,
                robotState::getAngleToTarget));
    }

    /**
     * Creates a command to deploy and run the intake mechanism to collect game pieces. The intake
     * will stop and retract automatically when the command ends.
     *
     * @param intake the intake subsystem
     * @return a command that runs the intake and stops it when finished
     */
    public static Command deployIntake(IntakeSuperstructure intake) {
        return Commands.startEnd(() -> intake.extendIntake(), () -> intake.retractIntake(), intake);
    }

    /**
     * A non-blocking command that initializes the intake by stopping the rollers and retracting the
     * linear stage
     *
     * @param intake the intake subsystem
     * @return a command that initializes the intake.
     */
    public static Command initializeIntake(IntakeSuperstructure intake) {
        return Commands.sequence(
            intake.stopRoller(),
            intake.retractLinear());
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
    public static Command agitateHopper(IntakeSuperstructure intake, Tower tower, Indexer indexer) {
        return intake.cycle();
    }

    /**
     * Prepares the shooter for shooting at THE HUB at the end of the provided path. Only valid to
     * prepare shots for THE HUB. Perpetual command -- never spins down. Therefore, to end, this
     * should be interrupted by a parent command group or timed-out. Primarily for use in autos.
     *
     * @param path the path to drive, the shooter will prepare to shoot at the end of this path.
     * @param shooter the shooter subsystem
     * @return a command that prepares the shooter to shoot THE HUB from the end of the provided
     *         path
     */
    public static Command prepareHubShot(PathPlannerPath path, ShooterSuperstructure shooter) {
        // All paths blue canonical, so flip end translation if red alliance
        return shooter.spinUpShooterToHubDistance(
            Meters.of(
                FieldUtil
                    .apply(path.getAllPathPoints().get(path.getAllPathPoints().size() - 1).position)
                    .getDistance(Target.HUB.getAllianceTranslation().toTranslation2d())));
    }

    /**
     * Drive to shooting location while spinning up shooter but not feeding game pieces. Once at
     * target position, with the shooter still spinning, verify subsystem process variables. Upon
     * confirmation of shooter-ready PVs, bring up the tower and indexer to begin shooting. Shoot
     * the PRELOADED FUEL for 1.5s. Bring down shooter, tower, and indexer to finish. If path
     * doesn't complete in 2.75s, attempt a shot anyway.
     *
     * @param drive The Drive subsystem
     * @param indexer The Indexer subsystem
     * @param tower The Tower subsystem
     * @param shooter The ShooterSuperstructure subsystem
     * @param path The path to drive to the shooting location, the robot will shoot from the path's
     *        end pose
     * @return a command that drives to the shooting location and attempts to shoot all the
     *         PRELOADED FUEL
     */
    public static Command makePreloadShot(Drive drive, Indexer indexer,
        Tower tower,
        ShooterSuperstructure shooter, PathPlannerPath path) {
        return Commands.sequence(
            new ParallelDeadlineGroup(
                AutoBuilder.followPath(path),
                AutoCommands.prepareHubShot(path, shooter)),
            FuelCommands.shootFuel(indexer, tower, shooter, () -> true, 1.5));
    }

    /**
     * Drive to shooting location while spinning up shooter but not feeding game pieces. Once at
     * target position, with the shooter still spinning, verify subsystem process variables. Upon
     * confirmation of shooter-ready PVs, bring up the tower and indexer to begin shooting. Shoot
     * all FUEL for 5s. Bring down shooter, tower, and indexer to finish. If path doesn't complete
     * in 3.5s, attempt a shot anyway.
     *
     * @param drive The Drive subsystem
     * @param intake The IntakeSuperStructure subsystem
     * @param indexer The Indexer subsystem
     * @param tower The Tower subsystem
     * @param shooter The ShooterSuperstructure subsystem
     * @param path The path to drive to the shooting location, the robot will shoot from the path's
     *        end pose
     * @return a command that drives to the shooting location and attempts to shoot all FUEL
     */
    public static Command makeFullShot(Drive drive, IntakeSuperstructure intake, Indexer indexer,
        Tower tower, ShooterSuperstructure shooter, PathPlannerPath path) {
        return Commands.sequence(
            new ParallelDeadlineGroup(
                AutoBuilder.followPath(path),
                AutoCommands.prepareHubShot(path, shooter)),
            new ParallelDeadlineGroup(
                FuelCommands.shootFuel(indexer, tower, shooter, () -> true, 5.0), // ~ 10 bps
                intake.cycle())); // TODO: more testing
    }

    /**
     * Drive to the end of the drive path, extend the intake, and drive into the FUEL with rollers
     * running. Once the intaking path is complete, stop the intake. This AutoSegment only linearly
     * actuates the intake while the robot is stationary. Non-blocking command.
     *
     * @param intake Intake subsystem
     * @param pathCommand The command that follows the desired path
     * @param afterPathWait The time to wait after the intaking path is complete before stopping the
     *        intake
     */
    public static Command driveAndIntake(IntakeSuperstructure intake, Command pathCommand,
        Time afterPathWait) {
        // Drive to near the intaking location, start up intake, and drive into the FUEL. Once the
        // intaking path is complete, stop the intake.
        return Commands.sequence(
            intake.extendIntake().withTimeout(1.25),
            pathCommand,
            Commands.waitSeconds(afterPathWait.in(Seconds)),
            intake.retractIntake().withTimeout(1.25));
    }
}
