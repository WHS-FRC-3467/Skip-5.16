// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.utils;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import choreo.auto.AutoTrajectory;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.lib.util.AlwaysTunableNumber;
import frc.robot.RobotState;
import frc.robot.RobotState.FieldRegion;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.indexer.IndexerSuperstructure;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.tower.Tower;
import frc.robot.util.RobotSim;

import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

/**
 * Class containing useful individual commands or small-group command sequences that can be strung
 * together into larger autonomous routines. Command logic layer.
 */
public class AutoCommands {
    private static final RobotState robotState = RobotState.getInstance();

    // Delay before following paths in auto.
    private static AlwaysTunableNumber autoDelay = new AlwaysTunableNumber("Auto/Delay", 0.0);

    /**
     * Accesses the value in the autoDelay AlwaysTunableNumber
     *
     * @return the delay, in seconds, to wait at the start of auto
     */
    public static double getAutoDelay() {
        return autoDelay.get();
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
                        .until(shooter.hopperEmpty)
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
                DriveCommands.staticAimTowardsTarget(drive));
    }

    /**
     * Drive to the outpost (via pathCommand), and wait up to 3 seconds for FUEL to be dumped.
     *
     * @param pathCommand The command that follows the desired path to the outpost.
     * @return A command that follows a path (to the outpost), stops the robot, and waits 3 seconds.
     */
    public static Command driveAndCollectAtOutpost(Command pathCommand) {
        return Commands.sequence(
                pathCommand,
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

    /**
     * Follows the given Choreo trajectory and falls back to pathfinding if the robot diverges too
     * far from the active trajectory target. When retrying from the neutral zone, the robot first
     * pathfinds to the tunnel entrance and then follows the tunnel trajectory back to the shared
     * shot pose.
     *
     * @param drive the drive subsystem
     * @param trajectory the primary Choreo auto trajectory to follow
     * @param tunnelTrajectory the Choreo tunnel trajectory used when retrying from the neutral zone
     * @param onRetry a command to run whenever fallback pathfinding begins
     * @return a command that safely follows the path with automatic fallback
     */
    public static Command safeFollowTrajectory(
            Drive drive,
            AutoTrajectory trajectory,
            AutoTrajectory tunnelTrajectory,
            Command onRetry) {
        Distance pathErrorTol = DriveConstants.ALLOWABLE_PATH_ERROR;
        Timer errorCheckDelayTimer = new Timer();
        Debouncer pathErrorDebouncer = new Debouncer(0.5);
        Debouncer goalPoseDebouncer = new Debouncer(0.25);
        Pose2d goalPose = trajectory.getFinalPose().orElse(new Pose2d());

        BooleanSupplier atGoal =
                () ->
                        goalPoseDebouncer.calculate(
                                robotState
                                                .getEstimatedPose()
                                                .getTranslation()
                                                .getDistance(goalPose.getTranslation())
                                        < DriveConstants.ALLOWABLE_SHOT_POSE_ERROR.in(Meters));

        return trajectory
                .cmd()
                .beforeStarting(
                        () -> {
                            errorCheckDelayTimer.restart();
                            robotState.setActiveTrajPose(goalPose);
                            Logger.recordOutput("Auto/GoalPose", goalPose);
                        })
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
                                Commands.none(),
                                retryPathing(
                                        drive,
                                        goalPose,
                                        tunnelTrajectory,
                                        atGoal,
                                        onRetry.asProxy()),
                                atGoal))
                .finallyDo(() -> Logger.recordOutput("Auto/GoalPose", new Pose2d()));
    }

    private static Command pathFindThenFollow(AutoTrajectory tunnelTrajectory) {
        Pose2d startPose = tunnelTrajectory.getInitialPose().orElseThrow();
        return Commands.sequence(
                AutoBuilder.pathfindToPose(startPose, DriveConstants.PATH_CONSTRAINTS),
                tunnelTrajectory.cmd());
    }

    /**
     * Repeatedly attempts to reach the goal pose by choosing between a direct pathfind to the goal
     * or a pathfind-then-follow sequence through the tunnel trajectory, depending on whether the
     * robot is still on the neutral-zone side of the field.
     */
    private static Command retryPathing(
            Drive drive,
            Pose2d goalPose,
            AutoTrajectory tunnelTrajectory,
            BooleanSupplier successCondition,
            Command onRetry) {
        Distance pathErrorTol = Meters.of(0.4572); // 18 inches
        Debouncer pathErrorDebouncer = new Debouncer(0.5);

        BooleanSupplier pathErrorExceeded =
                () ->
                        pathErrorDebouncer.calculate(
                                robotState.getActiveTrajectoryError().gte(pathErrorTol));

        return Commands.repeatingSequence(
                        onRetry,
                        drive.runOnce(drive::stop),
                        Commands.runOnce(
                                () ->
                                        Logger.recordOutput(
                                                "AutoCommands/RetryPathingStatus", "RETRY")),
                        Commands.either(
                                        pathFindThenFollow(tunnelTrajectory),
                                        AutoBuilder.pathfindToPose(
                                                goalPose, DriveConstants.PATH_CONSTRAINTS),
                                        () ->
                                                robotState.getFieldRegion()
                                                        == FieldRegion.NEUTRAL_ZONE)
                                .until(pathErrorExceeded))
                .until(successCondition)
                .finallyDo(() -> Logger.recordOutput("AutoCommands/RetryPathingStatus", "DONE"));
    }
}
