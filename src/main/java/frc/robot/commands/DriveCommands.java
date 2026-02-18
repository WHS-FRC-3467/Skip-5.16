/*
 * Copyright (C) 2026 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If
 * not, see <https://www.gnu.org/licenses/>.
 */

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;

/**
 * Factory class for creating drive-related commands.
 *
 * <p>Provides factory methods for common drive operations including:
 *
 * <ul>
 *   <li>Joystick-controlled driving (field-relative and robot-relative)
 *   <li>Angle locking for driver assistance
 *   <li>Pathfinding to specific field positions
 *   <li>System identification and characterization
 * </ul>
 *
 * <p>All commands are designed to work with the {@link Drive} subsystem and integrate with
 * PathPlanner for autonomous path following.
 */
public class DriveCommands {
    @Getter private static final double DEADBAND = 0.1;
    @Getter private static final double ANGLE_MAX_VELOCITY = 8.0;
    @Getter private static final double ANGLE_MAX_ACCELERATION = 20.0;
    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    private DriveCommands() {}

    /**
     * Converts joystick inputs to a linear velocity vector.
     *
     * @param x the x-axis joystick input
     * @param y the y-axis joystick input
     * @return the calculated linear velocity as a Translation2d
     */
    public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        double linearMagnitude = Math.pow(Math.hypot(x, y), 2);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     *
     * @param drive the drive subsystem
     * @param xSupplier supplier for x-axis joystick input
     * @param ySupplier supplier for y-axis joystick input
     * @param omegaSupplier supplier for rotation joystick input
     * @return the joystick drive command
     */
    public static Command joystickDrive(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        RobotState robotState = RobotState.getInstance();
        return Commands.run(
                        () -> {
                            // Get linear velocity
                            Translation2d linearVelocity =
                                    getLinearVelocityFromJoysticks(
                                            xSupplier.getAsDouble(), ySupplier.getAsDouble());

                            // Apply rotation deadband
                            double omega =
                                    MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                            // Square rotation value for more precise control
                            omega = Math.copySign(omega * omega, omega);

                            // Convert to field relative speeds & send command
                            ChassisSpeeds speeds =
                                    new ChassisSpeeds(
                                            linearVelocity.getX()
                                                    * drive.getMaxLinearSpeedMetersPerSec(),
                                            linearVelocity.getY()
                                                    * drive.getMaxLinearSpeedMetersPerSec(),
                                            omega * drive.getMaxAngularSpeedRadPerSec());
                            boolean isFlipped =
                                    DriverStation.getAlliance().isPresent()
                                            && DriverStation.getAlliance().get() == Alliance.Red;
                            drive.runVelocity(
                                    ChassisSpeeds.fromFieldRelativeSpeeds(
                                            speeds,
                                            isFlipped
                                                    ? robotState
                                                            .getEstimatedPose()
                                                            .getRotation()
                                                            .plus(new Rotation2d(Math.PI))
                                                    : robotState.getEstimatedPose().getRotation()));
                        },
                        drive)
                .withName("Joystick Drive");
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     *
     * @param drive the drive subsystem
     * @param xSupplier supplier for x-axis joystick input
     * @param ySupplier supplier for y-axis joystick input
     * @param rotationSupplier supplier for target rotation angle
     * @return the joystick drive at angle command
     */
    public static Command joystickDriveAtAngle(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Rotation2d> rotationSupplier) {
        RobotState robotState = RobotState.getInstance();

        // Create PID controller
        ProfiledPIDController angleController =
                new ProfiledPIDController(
                        ANGLE_KP,
                        0.0,
                        ANGLE_KD,
                        new TrapezoidProfile.Constraints(
                                ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.run(
                        () -> {
                            // Get linear velocity
                            Translation2d linearVelocity =
                                    getLinearVelocityFromJoysticks(
                                            xSupplier.getAsDouble(), ySupplier.getAsDouble());

                            // Calculate angular speed
                            double omega =
                                    angleController.calculate(
                                            robotState
                                                    .getEstimatedPose()
                                                    .getRotation()
                                                    .getRadians(),
                                            rotationSupplier.get().getRadians());

                            // Convert to field relative speeds & send command
                            ChassisSpeeds speeds =
                                    new ChassisSpeeds(
                                            linearVelocity.getX()
                                                    * drive.getMaxLinearSpeedMetersPerSec(),
                                            linearVelocity.getY()
                                                    * drive.getMaxLinearSpeedMetersPerSec(),
                                            omega);
                            boolean isFlipped =
                                    DriverStation.getAlliance().isPresent()
                                            && DriverStation.getAlliance().get() == Alliance.Red;
                            drive.runVelocity(
                                    ChassisSpeeds.fromFieldRelativeSpeeds(
                                            speeds,
                                            isFlipped
                                                    ? robotState
                                                            .getEstimatedPose()
                                                            .getRotation()
                                                            .plus(new Rotation2d(Math.PI))
                                                    : robotState.getEstimatedPose().getRotation()));
                        },
                        drive)

                // Reset PID controller when command starts
                .beforeStarting(
                        () ->
                                angleController.reset(
                                        robotState.getEstimatedPose().getRotation().getRadians()))
                .withName("Joystick Drive At Angle");
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Always faces the current target in RobotState
     *
     * @param drive the drive subsystem
     * @param xSupplier supplier for x-axis joystick input
     * @param ySupplier supplier for y-axis joystick input
     * @return the joystick drive facing target command
     */
    public static Command joystickDriveFacingTarget(
            Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        RobotState robotState = RobotState.getInstance();
        return joystickDriveAtAngle(drive, xSupplier, ySupplier, robotState::getAngleToTarget);
    }

    /**
     * Stationary control command that prohibits motion while it aims towards the target, then holds
     * the wheels in an X pattern once the robot is aligned with the target heading.
     *
     * @param drive the drive subsystem
     * @return the static aim towards target command
     */
    public static Command staticAimTowardsTarget(Drive drive) {
        RobotState robotState = RobotState.getInstance();
        return Commands.repeatingSequence(
                joystickDriveAtAngle(drive, () -> 0.0, () -> 0.0, robotState::getAngleToTarget)
                        .until(robotState.facingTarget),
                drive.runOnce(drive::stopWithX)
                        .andThen(drive.idle())
                        .onlyWhile(robotState.facingTarget));
    }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>This command should only be used in voltage control mode.
     *
     * @param drive the drive subsystem
     * @return the feedforward characterization command
     */
    public static Command feedforwardCharacterization(Drive drive) {
        List<Double> velocitySamples = new ArrayList<>();
        List<Double> voltageSamples = new ArrayList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                        // Reset data
                        Commands.runOnce(
                                () -> {
                                    velocitySamples.clear();
                                    voltageSamples.clear();
                                }),

                        // Allow modules to orient
                        Commands.run(
                                        () -> {
                                            drive.runCharacterization(0.0);
                                        },
                                        drive)
                                .withTimeout(FF_START_DELAY),

                        // Start timer
                        Commands.runOnce(timer::restart),

                        // Accelerate and gather data
                        Commands.run(
                                        () -> {
                                            double voltage = timer.get() * FF_RAMP_RATE;
                                            drive.runCharacterization(voltage);
                                            velocitySamples.add(
                                                    drive.getFFCharacterizationVelocity());
                                            voltageSamples.add(voltage);
                                        },
                                        drive)

                                // When cancelled, calculate and print results
                                .finallyDo(
                                        () -> {
                                            int n = velocitySamples.size();
                                            double sumX = 0.0;
                                            double sumY = 0.0;
                                            double sumXY = 0.0;
                                            double sumX2 = 0.0;
                                            for (int i = 0; i < n; i++) {
                                                sumX += velocitySamples.get(i);
                                                sumY += voltageSamples.get(i);
                                                sumXY +=
                                                        velocitySamples.get(i)
                                                                * voltageSamples.get(i);
                                                sumX2 +=
                                                        velocitySamples.get(i)
                                                                * velocitySamples.get(i);
                                            }
                                            double kS =
                                                    (sumY * sumX2 - sumX * sumXY)
                                                            / (n * sumX2 - sumX * sumX);
                                            double kV =
                                                    (n * sumXY - sumX * sumY)
                                                            / (n * sumX2 - sumX * sumX);

                                            NumberFormat formatter = new DecimalFormat("#0.00000");
                                            String results =
                                                    "Drive FF Characterization Results - "
                                                            + "kS: "
                                                            + formatter.format(kS)
                                                            + ", kV: "
                                                            + formatter.format(kV);
                                            DriverStation.reportWarning(results, false);
                                        }))
                .withName("Feedforward Characterization");
    }

    /**
     * Measures the robot's wheel radius by spinning in a circle.
     *
     * @param drive the drive subsystem
     * @return the wheel radius characterization command
     */
    public static Command wheelRadiusCharacterization(Drive drive) {
        RobotState robotState = RobotState.getInstance();

        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
                        // Drive control sequence
                        Commands.sequence(
                                // Reset acceleration limiter
                                Commands.runOnce(
                                        () -> {
                                            limiter.reset(0.0);
                                        }),

                                // Turn in place, accelerating up to full speed
                                Commands.run(
                                        () -> {
                                            double speed =
                                                    limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                                            drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                                        },
                                        drive)),

                        // Measurement sequence
                        Commands.sequence(
                                // Wait for modules to fully orient before starting measurement
                                Commands.waitSeconds(1.0),

                                // Record starting measurement
                                Commands.runOnce(
                                        () -> {
                                            state.positions =
                                                    drive.getWheelRadiusCharacterizationPositions();
                                            state.lastAngle =
                                                    robotState.getEstimatedPose().getRotation();
                                            state.gyroDelta = 0.0;
                                        }),

                                // Update gyro delta
                                Commands.run(
                                                () -> {
                                                    var rotation =
                                                            robotState
                                                                    .getEstimatedPose()
                                                                    .getRotation();
                                                    state.gyroDelta +=
                                                            Math.abs(
                                                                    rotation.minus(state.lastAngle)
                                                                            .getRadians());
                                                    state.lastAngle = rotation;
                                                })

                                        // When cancelled, calculate and print results
                                        .finallyDo(
                                                () -> {
                                                    double[] positions =
                                                            drive
                                                                    .getWheelRadiusCharacterizationPositions();
                                                    double wheelDelta = 0.0;
                                                    for (int i = 0; i < 4; i++) {
                                                        wheelDelta +=
                                                                Math.abs(
                                                                                positions[i]
                                                                                        - state.positions[
                                                                                                i])
                                                                        / 4.0;
                                                    }
                                                    double wheelRadius =
                                                            (state.gyroDelta
                                                                            * Drive
                                                                                    .DRIVE_BASE_RADIUS)
                                                                    / wheelDelta;

                                                    NumberFormat formatter =
                                                            new DecimalFormat("#0.000");
                                                    String results =
                                                            "Wheel Radius Characterization Results - "
                                                                    + "Wheel Delta: "
                                                                    + formatter.format(wheelDelta)
                                                                    + " radians, "
                                                                    + "Gyro Delta: "
                                                                    + formatter.format(
                                                                            state.gyroDelta)
                                                                    + " radians, "
                                                                    + "Wheel Radius: "
                                                                    + formatter.format(wheelRadius)
                                                                    + " meters ("
                                                                    + formatter.format(
                                                                            Units.metersToInches(
                                                                                    wheelRadius))
                                                                    + " inches)";
                                                    DriverStation.reportWarning(results, false);
                                                })))
                .withName("Wheel Radius Characterization");
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }

    /**
     * Pathfinding command that uses the AutoBuilder to generate a path to a target position.
     *
     * @param currentPose supplier for the robot's current pose
     * @param targetPose the target pose to pathfind to
     * @param constraints the path constraints to apply
     * @param goalEndVelocity the goal final velocity
     * @param tolerance the allowed position tolerance from the target pose
     * @return the pathfinding command
     */
    public static Command pathFindToPose(
            Supplier<Pose2d> currentPose,
            Pose2d targetPose,
            PathConstraints constraints,
            LinearVelocity goalEndVelocity,
            Distance tolerance) {

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
                        targetPose, constraints, goalEndVelocity // Goal end velocity in meters/sec
                        )
                .raceWith(
                        // Interrupt the pathfinding command once the robot gets within the
                        // tolerance of the
                        // target pose
                        Commands.waitUntil(
                                () ->
                                        currentPose
                                                        .get()
                                                        .minus(targetPose)
                                                        .getTranslation()
                                                        .getNorm()
                                                < tolerance.in(Meters)))
                .withName("Pathfind to " + targetPose.toString());
    }
}
