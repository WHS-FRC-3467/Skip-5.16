// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.utils;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.lib.util.AlwaysTunableNumber;
import frc.lib.util.FieldUtil;
import frc.robot.RobotState;
import frc.robot.RobotState.FieldRegion;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.tower.Tower;
import frc.robot.util.RobotSim;

import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.Set;
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
            Indexer indexer,
            Tower tower,
            ShooterSuperstructure shooter,
            double timeoutDuration) {
        return Commands.deadline(
                Commands.parallel(
                                shooter.spinUpShooter().asProxy(),
                                Commands.sequence(
                                        Commands.waitUntil(
                                                shooter.profileComplete.and(
                                                        RobotState.getInstance().facingTarget)),
                                        Commands.parallel(
                                                indexer.shoot(),
                                                tower.shoot(),
                                                Commands.waitSeconds(0.25)
                                                        .andThen(
                                                                Commands.defer(
                                                                        intake::slowRetract,
                                                                        Set.of(intake))))))
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
     * Shoots the currently held note, stows the hood, retracts the intake, then starts the next
     * trajectory.
     */
    public static Command shootThenFollow(
            AutoContext ctx, double timeoutSeconds, AutoTrajectory next) {
        return Commands.sequence(
                shootOnly(ctx, timeoutSeconds),
                stowHood(ctx.shooter()),
                retractIntake(ctx),
                next.spawnCmd());
    }

    /**
     * Starts the given trajectory then shoots the currently held FUEL. Spins down the shooter and
     * retracts the intake afterwards.
     */
    public static Command followThenShoot(
            AutoContext ctx, double timeoutSeconds, AutoTrajectory current) {
        return Commands.sequence(
                current.spawnCmd(),
                Commands.waitUntil(current.done()),
                shootOnly(ctx, timeoutSeconds),
                retractIntake(ctx));
    }

    // /**
    //  * Return the int corresponding to the lane most populated with FUEL according to the object
    //  * detector, if the lane exists. Currently factored for just 3 ML lanes indexed 0-2.
    //  *
    //  * @param objectDetector the object detector subsystem
    //  * @return an Optional containing the lane number (0, 1, or 2) with the most FUEL, or an
    // empty
    //  *     Optional if no lane is matched
    //  */
    // public static Optional<Integer> getBestLane(ObjectDetector objectDetector) {
    //     Optional<LaneTarget> bestLaneTarget = objectDetector.getBestLaneTarget();
    //     if (bestLaneTarget.isEmpty()) {
    //         return Optional.empty();
    //     }
    //     double laneX = bestLaneTarget.get().x();

    //     // Three pre-defined ML lanes, so find the closest one to the optimal lane
    //     double[] lanes =
    //             new double[] {
    //                 ChoreoVars.Poses.LanePose1ML.getX(),
    //                 ChoreoVars.Poses.LanePose2ML.getX(),
    //                 ChoreoVars.Poses.LanePose3ML.getX()
    //             };

    //     int bestIndex = -1;
    //     double bestDistance = Double.MAX_VALUE;
    //     for (int i = 0; i < lanes.length; i++) {
    //         double distance = Math.abs(laneX - lanes[i]);
    //         if (distance < bestDistance) {
    //             bestDistance = distance;
    //             bestIndex = i;
    //         }
    //     }

    //     return bestIndex == -1 ? Optional.empty() : Optional.of(bestIndex);
    // }

    /**
     * Recovers to the failed trajectory's endpoint, then runs the normal shoot-and-continue flow.
     */
    public static Command recoverThenFollow(
            AutoContext ctx,
            AutoTrajectory failedTrajectory,
            Optional<AutoTrajectory> tunnel,
            double timeoutSeconds,
            AutoTrajectory next) {
        return Commands.sequence(
                ctx.drive().runOnce(ctx.drive()::stop),
                recoverTrajectory(ctx.drive(), failedTrajectory, tunnel, retractIntake(ctx)),
                shootThenFollow(ctx, timeoutSeconds, next));
    }

    /**
     * Creates a routine-bound trigger that rises once a running trajectory has exceeded the
     * allowable path error for long enough that the auto should fall back to retry pathfinding.
     */
    public static Trigger retryTrigger(AutoRoutine routine, AutoTrajectory trajectory) {
        Distance pathErrorTol = DriveConstants.ALLOWABLE_PATH_ERROR;
        return routine.observe(
                new BooleanSupplier() {
                    private final Timer errorCheckDelayTimer = new Timer();
                    private final Debouncer pathErrorDebouncer = new Debouncer(0.5);
                    private boolean wasActive = false;

                    @Override
                    public boolean getAsBoolean() {
                        boolean isActive = trajectory.active().getAsBoolean();

                        if (isActive && !wasActive) {
                            errorCheckDelayTimer.restart();
                        } else if (!isActive && wasActive) {
                            errorCheckDelayTimer.stop();
                            errorCheckDelayTimer.reset();
                            pathErrorDebouncer.calculate(false);
                        }

                        wasActive = isActive;

                        return isActive
                                && errorCheckDelayTimer.hasElapsed(2.0)
                                && (pathErrorDebouncer.calculate(
                                                robotState
                                                        .getActiveTrajectoryError()
                                                        .gte(pathErrorTol))
                                        || robotState.forcePathFind.get());
                    }
                });
    }

    /**
     * Reaches the supplied trajectory's final pose via the retry pathfinding flow used when an
     * active Choreo trajectory has to be abandoned.
     */
    public static Command recoverTrajectory(
            Drive drive,
            AutoTrajectory trajectory,
            Optional<AutoTrajectory> tunnelTrajectory,
            Command onRetry) {
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

        return Commands.runOnce(
                        () -> {
                            robotState.setActiveTrajPose(goalPose);
                            Logger.recordOutput("Auto/GoalPose", goalPose);
                        })
                .andThen(
                        Commands.either(
                                Commands.none(),
                                retryPathing(drive, goalPose, tunnelTrajectory, atGoal, onRetry),
                                atGoal))
                .finallyDo(() -> Logger.recordOutput("Auto/GoalPose", new Pose2d()));
    }

    private static Command pathFindThenFollow(
            Drive drive, Pose2d goalPose, AutoTrajectory tunnelTrajectory) {
        return Commands.defer(
                () ->
                        tunnelTrajectory
                                .getRawTrajectory()
                                .getInitialPose(false)
                                .map(FieldUtil::apply)
                                .map(
                                        startPose ->
                                                Commands.sequence(
                                                        AutoBuilder.pathfindToPose(
                                                                startPose,
                                                                DriveConstants.PATH_CONSTRAINTS,
                                                                2.0),
                                                        tunnelTrajectory.cmd()))
                                .orElseGet(
                                        () -> {
                                            DriverStation.reportWarning(
                                                    "Retry tunnel trajectory is missing an initial"
                                                            + " pose. Falling back to a direct"
                                                            + " pathfind.",
                                                    false);
                                            return AutoBuilder.pathfindToPose(
                                                    goalPose, DriveConstants.PATH_CONSTRAINTS);
                                        }),
                Set.of(drive));
    }

    /**
     * Repeatedly attempts to reach the goal pose by choosing between a direct pathfind to the goal
     * or a pathfind-then-follow sequence through the tunnel trajectory, depending on whether the
     * robot is still on the neutral-zone side of the field.
     */
    private static Command retryPathing(
            Drive drive,
            Pose2d goalPose,
            Optional<AutoTrajectory> tunnelTrajectory,
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
                                        tunnelTrajectory
                                                .map(
                                                        trajectory ->
                                                                pathFindThenFollow(
                                                                        drive,
                                                                        goalPose,
                                                                        trajectory))
                                                .orElseGet(
                                                        () ->
                                                                AutoBuilder.pathfindToPose(
                                                                        goalPose,
                                                                        DriveConstants
                                                                                .PATH_CONSTRAINTS)),
                                        AutoBuilder.pathfindToPose(
                                                goalPose, DriveConstants.PATH_CONSTRAINTS),
                                        () ->
                                                robotState.getFieldRegion()
                                                                == FieldRegion.NEUTRAL_ZONE
                                                        && tunnelTrajectory.isPresent())
                                .until(pathErrorExceeded))
                .until(successCondition)
                .finallyDo(() -> Logger.recordOutput("AutoCommands/RetryPathingStatus", "DONE"));
    }

    private static Command shootOnly(AutoContext ctx, double timeoutSeconds) {
        return shootCommand(
                ctx.drive(),
                ctx.intake(),
                ctx.indexer(),
                ctx.tower(),
                ctx.shooter(),
                timeoutSeconds);
    }

    private static Command retractIntake(AutoContext ctx) {
        return ctx.intake().retractIntake().withTimeout(0.5);
    }
}
