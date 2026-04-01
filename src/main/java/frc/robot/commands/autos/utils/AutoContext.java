package frc.robot.commands.autos.utils;

import choreo.auto.AutoFactory;

import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.objectdetector.ObjectDetector;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.tower.Tower;

import org.littletonrobotics.junction.Logger;

import java.util.Optional;

/**
 * Shared dependencies for autonomous routine builders.
 *
 * <p>This keeps each auto class focused on sequencing while centralizing the common Choreo factory
 * configuration and subsystem references they all need.
 */
public record AutoContext(
        Drive drive,
        IntakeSuperstructure intake,
        Indexer indexer,
        Tower tower,
        ShooterSuperstructure shooter,
        Optional<ObjectDetector> objectDetector,
        RobotState robotState,
        AutoFactory autoFactory) {
    /**
     * Creates the shared autonomous context and configures the Choreo factory used by all autos.
     */
    public static AutoContext create(
            Drive drive,
            IntakeSuperstructure intake,
            Indexer indexer,
            Tower tower,
            ShooterSuperstructure shooter,
            Optional<ObjectDetector> objectDetector) {
        RobotState robotState = RobotState.getInstance();
        AutoFactory autoFactory =
                new AutoFactory(
                        robotState::getEstimatedPose,
                        robotState::resetPose,
                        drive::followTrajectorySample,
                        true,
                        drive,
                        (trajectory, starting) ->
                                Logger.recordOutput(
                                        "Odometry/Trajectory",
                                        starting ? trajectory.getPoses() : new Pose2d[] {}));
        return new AutoContext(
                drive, intake, indexer, tower, shooter, objectDetector, robotState, autoFactory);
    }
}
