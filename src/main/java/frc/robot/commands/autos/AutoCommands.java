// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.lib.util.FieldUtil;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
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

    public static Command shootCommand(
            Drive drive,
            IntakeSuperstructure intake,
            IndexerSuperstructure indexer,
            Tower tower,
            ShooterSuperstructure shooter,
            double timeoutDuration) {
        return Commands.deadline(
                Commands.parallel(
                                shooter.spinUpShooter().asProxy(),
                                Commands.parallel(
                                                indexer.shoot()
                                                        .withInterruptBehavior(
                                                                InterruptionBehavior
                                                                        .kCancelIncoming),
                                                tower.shoot(),
                                                intake.shuffleStep().repeatedly().asProxy())
                                        .onlyWhile(
                                                shooter.readyToShoot.and(
                                                        RobotState.getInstance().facingTarget))
                                        .repeatedly())
                        .withTimeout(timeoutDuration)
                        .finallyDo(
                                () -> {
                                    CommandScheduler.getInstance()
                                            .schedule(
                                                    shooter.setFlywheelSpeed(
                                                            RotationsPerSecond.zero()));
                                    CommandScheduler.getInstance()
                                            .schedule(shooter.setHoodAngle(Rotations.zero()));
                                }),
                DriveCommands.autoAimTowardsTarget(drive));
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
     * @param shooter the shooter superstructure subsystem
     * @return returns a timed command that retracts the intake and lowers the hood
     */
    public static Command stowHood(ShooterSuperstructure shooter) {
        return Commands.parallel(shooter.retractHood()).withTimeout(1.25);
    }

    public static Command safeFollowPath(
            Drive drive, PathPlannerPath path, Distance pathErrorTol, PathPlannerPath tunnelPath) {
        Debouncer pathErrorDebouncer = new Debouncer(.5);
        Timer errorCheckDelayTimer = new Timer();
        Pose2d goalPose =
                new Pose2d(
                        tunnelPath
                                .getPathPoses()
                                .get(tunnelPath.getPathPoses().size() - 1)
                                .getTranslation(),
                        tunnelPath.getGoalEndState().rotation());
        Pose2d tunnelEnterancePose = tunnelPath.getPathPoses().get(0);

        return AutoBuilder.followPath(path)
                .beforeStarting(errorCheckDelayTimer::restart)
                .until(
                        () ->
                                errorCheckDelayTimer.hasElapsed(2.0)
                                        && (pathErrorDebouncer.calculate(
                                                        robotState
                                                                .getActiveTrajectoryError()
                                                                .gte(pathErrorTol))
                                                || robotState.forcePathFind.get()))
                .andThen(
                        Commands.either(
                                Commands.either(
                                        AutoBuilder.pathfindToPoseFlipped(
                                                goalPose, DriveConstants.PATH_CONSTRAINTS),
                                        AutoBuilder.pathfindThenFollowPath(
                                                tunnelPath, DriveConstants.PATH_CONSTRAINTS),
                                        () ->
                                                FieldUtil.apply(robotState.getEstimatedPose())
                                                        .getMeasureX()
                                                        .lt(tunnelEnterancePose.getMeasureX())),
                                Commands.none(),
                                () ->
                                        pathErrorDebouncer.calculate(
                                                        robotState
                                                                .getActiveTrajectoryError()
                                                                .gte(pathErrorTol))
                                                || robotState.forcePathFind.get()));
    }
}
