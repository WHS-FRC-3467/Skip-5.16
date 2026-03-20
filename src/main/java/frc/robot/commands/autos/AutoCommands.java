// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
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
import frc.robot.util.AlwaysTunableNumber;
import frc.robot.util.RobotSim;

import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

/**
 * Class containing useful individual commands or small-group command sequences that can be strung
 * together into larger autos (that extend AutoRoutine). Command logic layer.
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

    /**
     * Resets the robot's odometry to the starting pose of the specified path. Handles alliance
     * flipping if necessary.
     *
     * @param drive the drive subsystem
     * @param path the PathPlanner path containing the starting pose
     * @return a command that resets the robot's pose to the path's starting position
     */
    public static Command resetOdom(Drive drive, PathPlannerPath path) {
        return drive.runOnce(
                () -> {
                    Pose2d pose = path.getStartingHolonomicPose().get();
                    if (FieldUtil.shouldFlip()) {
                        pose = FieldUtil.apply(pose);
                    }

                    robotState.resetPose(pose);
                });
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

    /**
     * Follows the given path, and if the trajectory error exceeds the tolerance after an initial
     * delay, falls back to {@link #retryPathing} which pathfinds (or pathfind-then-follows through
     * the tunnel) until the robot reaches the goal pose.
     *
     * @param drive the drive subsystem
     * @param path the primary path to follow
     * @param tunnelPath the tunnel path used for fallback pathing
     * @param onRetry a command to run when falling back to retry pathing
     * @return a command that safely follows the path with automatic fallback
     */
    public static Command safeFollowPath(
            Drive drive, PathPlannerPath path, PathPlannerPath tunnelPath, Command onRetry) {
        Distance pathErrorTol = DriveConstants.ALLOWABLE_PATH_ERROR;

        Timer errorCheckDelayTimer = new Timer();

        Pose2d goalPose =
                (new Pose2d(
                        path.getPathPoses().get(path.getPathPoses().size() - 1).getTranslation(),
                        path.getGoalEndState().rotation()));

        Debouncer pathErrorDebouncer = new Debouncer(.5);

        Distance goalErrorTol = Inches.of(4.0);
        Debouncer goalPoseDebouncer = new Debouncer(.25);

        // Supplier that checks whether the robot has reached the goal pose
        BooleanSupplier atGoal =
                () ->
                        goalPoseDebouncer.calculate(
                                robotState
                                                .getEstimatedPose()
                                                .getTranslation()
                                                .getDistance(
                                                        FieldUtil.apply(goalPose.getTranslation()))
                                        < goalErrorTol.in(Meters));

        return AutoBuilder.followPath(path)
                .beforeStarting(
                        () -> {
                            errorCheckDelayTimer.restart();
                            Logger.recordOutput("Auto/Goal Pose", goalPose);
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
                                retryPathing(drive, goalPose, tunnelPath, atGoal, onRetry),
                                Commands.none(),
                                () -> !atGoal.getAsBoolean()));
    }

    /**
     * Pathfinds to the starting pose of the given path and then follows it to completion. Uses the
     * path's ideal starting velocity as the handoff speed between the pathfinding and
     * path-following segments.
     *
     * @param path the path to pathfind to and then follow
     * @return a command that pathfinds to the path start and then follows the path
     */
    private static Command pathFindThenFollow(PathPlannerPath path) {
        return Commands.sequence(
                AutoBuilder.pathfindToPoseFlipped(
                        path.getStartingHolonomicPose()
                                .orElseThrow(
                                        () ->
                                                new IllegalStateException(
                                                        "Path has no starting holonomic pose")),
                        DriveConstants.PATH_CONSTRAINTS,
                        path.getIdealStartingState().velocity()),
                AutoBuilder.followPath(path));
    }

    /**
     * Repeatedly attempts to reach the goal pose by choosing between a direct pathfind to the goal
     * or a {@link #pathFindThenFollow} through the tunnel, based on the robot's X position relative
     * to the first pose of the tunnel path. If the trajectory error exceeds tolerance during an
     * attempt, that attempt is cancelled and a new plan is generated from the robot's current pose.
     * Continues looping until the success condition is met.
     *
     * @param goalPose the target pose to reach
     * @param tunnelPath the path through the tunnel used when the robot is on the neutral-zone side
     * @param successCondition returns true when the robot has reached the goal pose
     * @return a command that retries pathing until the success condition is satisfied
     */
    private static Command retryPathing(
            Drive drive,
            Pose2d goalPose,
            PathPlannerPath tunnelPath,
            BooleanSupplier successCondition,
            Command onRetry) {
        Distance pathErrorTol = Inches.of(18.0);
        Debouncer pathErrorDebouncer = new Debouncer(0.5);

        // Cancel the current pathfind when the trajectory error exceeds tolerance,
        // causing the loop to restart and replan from the robot's current pose.
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
                                                "AutoCommands/RetryPathingStatus",
                                                "RETRY")), // If robot X is behind (less than) the
                        // tunnel entrance X, it is
                        // already past the tunnel on the alliance side — pathfind directly.
                        // Otherwise, it still needs to travel through the tunnel.
                        Commands.either(
                                        pathFindThenFollow(tunnelPath),
                                        AutoBuilder.pathfindToPoseFlipped(
                                                goalPose, DriveConstants.PATH_CONSTRAINTS),
                                        robotState.isInNeutralZone)
                                .until(pathErrorExceeded))
                .until(successCondition)
                .finallyDo(() -> Logger.recordOutput("AutoCommands/RetryPathingStatus", "DONE"));
    }
}
