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
import frc.robot.util.RobotSim;

import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

/**
 * Class containing useful individual commands or small-group command sequences that can be strung
 * together into larger autos (that extend AutoRoutine). Command logic layer.
 */
public class AutoCommands {
    private static final RobotState robotState = RobotState.getInstance();

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
     * @return a command that safely follows the path with automatic fallback
     */
    public static Command safeFollowPath(
            Drive drive, PathPlannerPath path, PathPlannerPath tunnelPath) {
        Distance pathErrorTol = Inches.of(18.0);

        Timer errorCheckDelayTimer = new Timer();

        Pose2d goalPose =
                FieldUtil.apply(
                        new Pose2d(
                                tunnelPath
                                        .getPathPoses()
                                        .get(tunnelPath.getPathPoses().size() - 1)
                                        .getTranslation(),
                                tunnelPath.getGoalEndState().rotation()));

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
                                                .getDistance(goalPose.getTranslation())
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
                                retryPathing(goalPose, tunnelPath, atGoal),
                                Commands.none(),
                                () -> !atGoal.getAsBoolean()));
    }

    private static Command pathFindThenFollow(PathPlannerPath path) {
        return Commands.sequence(
                AutoBuilder.pathfindToPoseFlipped(
                        path.getStartingHolonomicPose().get(),
                        DriveConstants.PATH_CONSTRAINTS,
                        path.getIdealStartingState().velocity()),
                AutoBuilder.followPath(path));
    }

    /**
     * Repeatedly attempts to reach the goal pose by choosing between a direct pathfind or a
     * pathfind-then-follow through the tunnel, based on the robot's Y position relative to the
     * tunnel entrance. Runs until the success condition is met (robot is at the goal).
     *
     * @param goalPose the target pose to reach
     * @param tunnelPath the path through the tunnel
     * @param successCondition returns true when the robot has reached the goal pose
     * @return a command that retries pathing until success
     */
    private static Command retryPathing(
            Pose2d goalPose, PathPlannerPath tunnelPath, BooleanSupplier successCondition) {
        Distance pathErrorTol = Inches.of(18.0);
        Debouncer pathErrorDebouncer = new Debouncer(0.5);

        // Cancel the current pathfind when the trajectory error exceeds tolerance,
        // causing the loop to restart and replan from the robot's current pose.
        BooleanSupplier pathErrorExceeded =
                () ->
                        pathErrorDebouncer.calculate(
                                robotState.getActiveTrajectoryError().gte(pathErrorTol));

        return Commands.repeatingSequence(
                        Commands.runOnce(() -> System.out.println("STARTING RETRY LOOP")),
                        // If robot Y is behind (less than) the tunnel entrance Y, it is
                        // already past the tunnel on the alliance side — pathfind directly.
                        // Otherwise, it still needs to travel through the tunnel.
                        Commands.either(
                                        AutoBuilder.pathfindToPoseFlipped(
                                                goalPose, DriveConstants.PATH_CONSTRAINTS),
                                        pathFindThenFollow(tunnelPath),
                                        () ->
                                                FieldUtil.apply(robotState.getEstimatedPose())
                                                        .getMeasureX()
                                                        .lt(
                                                                tunnelPath
                                                                        .getPathPoses()
                                                                        .get(0)
                                                                        .getMeasureX()))
                                .until(pathErrorExceeded),
                        Commands.waitSeconds(0.5))
                .until(successCondition)
                .finallyDo(() -> System.out.println("RETRY PATHING DONE"));
    }
}
