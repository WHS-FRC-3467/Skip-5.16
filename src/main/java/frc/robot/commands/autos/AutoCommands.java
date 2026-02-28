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
import frc.robot.util.RobotSim;

/**
 * Class containing useful individual commands or small-group command sequences that can be strung
 * together into larger autos (that extend AutoRoutine). Command logic layer.
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
                FuelCommands.prepareShot(
                        indexer, tower, shooter, robotState.facingTarget, duration),
                DriveCommands.joystickDriveAtAngle(
                        drive, () -> 0.0, () -> 0.0, robotState::getAngleToTarget));
    }

    /**
     * Statically prepares the shooter for shooting at THE HUB at the end of the provided path. Only
     * valid to prepare shots for THE HUB. Perpetual command -- never spins down. Therefore, to end,
     * this should be interrupted by a parent command group or timed-out. Primarily for use in
     * autos.
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
     * Drive to shooting location while STATICALLY spinning up shooter to setpoints associated with
     * ANTICIPATED POSE at end of path but NOT feeding game pieces. Once at ANTICIPATED POSE, with
     * the shooter still spinning at ANTICIPATED SETPOINTS, call {@code FuelCommands.prepareShot()}.
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
    public static Command moveToShot(
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
                        FuelCommands.prepareShot(
                                indexer,
                                tower,
                                shooter,
                                () -> true,
                                4.5))); // TODO: hopper agitation
    }

    /**
     * Drive to the end of the drive path, extend the intake, and drive into the FUEL with rollers
     * running. Once the intaking path is complete, stop the intake. This AutoSegment only linearly
     * actuates the intake during approachPath. Non-blocking command.
     *
     * @param approachPathCommand Staging path to follow while simultaneously extending intake
     * @param intake Intake subsystem
     * @param pathCommand The feeding path with intake already extended
     * @param afterPathWait The time to wait after the intaking path is complete before stopping the
     *     intake
     */
    public static Command driveAndIntake(
            Command approachPathCommand,
            IntakeSuperstructure intake,
            Command pathCommand,
            Time afterPathWait) {
        // Drive to near the intaking location and start up intake, and drive into the FUEL. Once
        // the intaking path is complete, stop the intake.
        return Commands.sequence(
                // Roll out intake (times out at 1.25 seconds)
                // WHILE following the path that takes the robot near the fuel location.
                Commands.parallel(intake.extendIntake().withTimeout(1.25), approachPathCommand),
                // Collect FUEL
                pathCommand,
                Commands.waitSeconds(afterPathWait.in(Seconds)));
    }

    /**
     * Drive to the outpost (via pathCommand), and wait up to 3 seconds for FUEL to be dumped.
     *
     * @param pathCommand The command that follows the desired path to the outpost.
     * @return A command that follows a path (to the outpost), stops the robot, and waits 3 seconds.
     */
    public static Command driveAndCollectAtOutpost(Command pathCommand) {
        return Commands.sequence(
                // Go to the OUTPOST
                pathCommand,
                // Wait for FUEL to be dumped
                Commands.waitSeconds(3),
                Commands.either(
                        Commands.runOnce(
                                () -> RobotSim.getInstance().getFuelSim().fillHopperBy(20)),
                        Commands.none(),
                        RobotBase::isSimulation));
    }

    /**
     * Makes the robot smaller by retracting the intake and lowering the hood.
     *
     * @param intake the intake superstructure subsystem
     * @param shooter the shooter superstructure subsystem
     * @return returns a timed command that retracts the intake and lowers the hood
     */
    public static Command makeSmall(IntakeSuperstructure intake, ShooterSuperstructure shooter) {
        return Commands.parallel(intake.retractIntake(), shooter.retractHood()).withTimeout(1.25);
    }
}
