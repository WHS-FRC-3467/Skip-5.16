// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Meters;
import java.util.function.BooleanSupplier;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
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
     * the shooter, only pulls fuel through the feeder when ready (shooter state + alignment), then
     * stops indexer and tower after duration. If shooting is disrupted during duration because
     * shooting readiness drops, attempt a flywheel/hood adjustment and, if successful, re-commence
     * shooting within the remaining window. Unconditionally stops shot attempts after duration.
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
        ShooterSuperstructure shooter, BooleanSupplier canShoot, double duration)
    {
        Command feed = Commands.parallel(
            indexer.holdStateUntilInterrupted(Indexer.State.PULL),
            tower.holdStateUntilInterrupted(Tower.State.SHOOT))
            .until(() -> !canShoot.getAsBoolean());

        return shooter
            .prepareShot(Commands.waitUntil(canShoot).andThen(feed)).withTimeout(duration);
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
    public static Command deployIntake(IntakeSuperstructure intake)
    {
        return Commands.startEnd(() -> intake.extendIntake(), () -> intake.retractIntake(), intake);
    }

    /**
     * A non-blocking command that initializes the intake by stopping the rollers and retracting the
     * linear stage
     *
     * @param intake the intake subsystem
     * @return a command that initializes the intake.
     */
    public static Command initializeIntake(IntakeSuperstructure intake)
    {
        return Commands.sequence(
            intake.stopRoller(),
            intake.retractLinear());
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
    public static Command prepareHubShot(PathPlannerPath path, ShooterSuperstructure shooter)
    {
        // All paths blue canonical, so flip end translation if red alliance
        return shooter.spinUpShooterToHubDistance(
            Meters.of(
                FieldUtil
                    .apply(path.getAllPathPoints().get(path.getAllPathPoints().size() - 1).position)
                    .getDistance(Target.HUB.getAllianceTranslation().toTranslation2d())));
    }
}
