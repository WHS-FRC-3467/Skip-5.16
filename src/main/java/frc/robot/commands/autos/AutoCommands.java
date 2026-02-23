// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

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
import frc.robot.commands.FuelCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.IndexerSuperstructure;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.tower.Tower;

/**
 * Class containing useful individual commands or small-group command sequences that can be strung
 * together into larger command units (AutoSegments). Command logic layer.
 */
public class AutoCommands {
    private static final RobotState robotState = RobotState.getInstance();

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
            return drive.runOnce(
                    () -> {
                        Pose2d pose = path.getStartingHolonomicPose().get();
                        if (FieldUtil.shouldFlip()) {
                            pose = FieldUtil.apply(pose);
                        }

                        robotState.resetPose(pose);
                    });
        } else {
            return Commands.none();
        }
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
     *     shooter
     * @param duration the maximum duration in seconds to run the align-and-shoot sequence
     * @return a command that aligns the robot to the target and shoots for up to the given duration
     */
    public static Command alignAndShoot(
            Drive drive,
            IndexerSuperstructure indexer,
            Tower tower,
            ShooterSuperstructure shooter,
            double duration) {
        return Commands.deadline(
                FuelCommands.shootFuel(indexer, tower, shooter, robotState.facingTarget, duration),
                DriveCommands.joystickDriveAtAngle(
                        drive, () -> 0.0, () -> 0.0, robotState::getAngleToTarget));
    }

    /**
     * Prepares the shooter for shooting at THE HUB at the end of the provided path. Only valid to
     * prepare shots for THE HUB. Perpetual command -- never spins down. Therefore, to end, this
     * should be interrupted by a parent command group or timed-out. Primarily for use in autos.
     *
     * @param path the path to drive, the shooter will prepare to shoot at the end of this path.
     * @param shooter the shooter subsystem
     * @return a command that prepares the shooter to shoot THE HUB from the end of the provided
     *     path
     */
    public static Command prepareHubShot(PathPlannerPath path, ShooterSuperstructure shooter) {
        // All paths blue canonical, so flip end translation if red alliance
        return shooter.spinUpShooterToHubDistance(
                Meters.of(
                        FieldUtil.apply(
                                        path.getAllPathPoints()
                                                .get(path.getAllPathPoints().size() - 1)
                                                .position)
                                .getDistance(
                                        Target.HUB.getAllianceTranslation().toTranslation2d())));
    }

    /**
     * Drive to shooting location while spinning up shooter but not feeding game pieces. Once at
     * target position, with the shooter still spinning, verify subsystem process variables. Upon
     * confirmation of shooter-ready PVs, bring up the tower and indexer to begin shooting. Shoot
     * the PRELOADED FUEL for 1.5s. Bring down shooter, tower, and indexer to finish. If path
     * doesn't complete in 2.75s, attempt a shot anyway.
     *
     * @param drive The Drive subsystem
     * @param indexer The IndexerSuperstructure subsystem
     * @param tower The Tower subsystem
     * @param shooter The ShooterSuperstructure subsystem
     * @param path The path to drive to the shooting location, the robot will shoot from the path's
     *     end pose
     * @return a command that drives to the shooting location and attempts to shoot all the
     *     PRELOADED FUEL
     */
    public static Command makePreloadShot(
            Drive drive,
            IndexerSuperstructure indexer,
            Tower tower,
            ShooterSuperstructure shooter,
            PathPlannerPath path) {
        return Commands.sequence(
                new ParallelDeadlineGroup(
                        AutoBuilder.followPath(path), AutoCommands.prepareHubShot(path, shooter)),
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
     * @param indexer The IndexerSuperstructure subsystem
     * @param tower The Tower subsystem
     * @param shooter The ShooterSuperstructure subsystem
     * @param path The path to drive to the shooting location, the robot will shoot from the path's
     *     end pose
     * @return a command that drives to the shooting location and attempts to shoot all FUEL
     */
    public static Command makeFullShot(
            Drive drive,
            IntakeSuperstructure intake,
            IndexerSuperstructure indexer,
            Tower tower,
            ShooterSuperstructure shooter,
            PathPlannerPath path) {
        return Commands.sequence(
                new ParallelDeadlineGroup(
                        AutoBuilder.followPath(path), AutoCommands.prepareHubShot(path, shooter)),
                new ParallelDeadlineGroup(
                        FuelCommands.shootFuel(
                                indexer, tower, shooter, () -> true, 5.0), // ~ 10 bps
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
     *     intake
     */
    public static Command driveAndIntake(
            IntakeSuperstructure intake, Command pathCommand, Time afterPathWait) {
        // Drive to near the intaking location, start up intake, and drive into the FUEL. Once the
        // intaking path is complete, stop the intake.
        return Commands.sequence(
                intake.intake().withTimeout(1.25),
                pathCommand,
                Commands.waitSeconds(afterPathWait.in(Seconds)));
    }
}
