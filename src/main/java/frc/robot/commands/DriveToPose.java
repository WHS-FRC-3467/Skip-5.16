// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggedTuneableProfiledPID;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Command to autonomously drive the robot to a target pose on the field.
 *
 * <p>Uses profiled PID controllers for both linear (x, y) and angular (rotation) motion. The
 * controllers have tunable gains that can be adjusted through NetworkTables for optimization.
 * Maximum velocities are also tunable.
 */
public class DriveToPose extends Command {

    private static final LoggedTuneableProfiledPID linearController =
            new LoggedTuneableProfiledPID("DriveToPose/LinearController", 3.0, 0, 0.1, 3.0, 0.0);
    private static final LoggedTuneableProfiledPID angularController =
            new LoggedTuneableProfiledPID("DriveToPose/AngularController", 3.0, 0, 0, 0, 0);

    private static final LoggedTunableNumber maxLinearVel =
            new LoggedTunableNumber("DriveToPose/MaxLinearVelocity (m s)", 3.0);
    private static final LoggedTunableNumber maxAngularVel =
            new LoggedTunableNumber("DriveToPose/MaxAngularVelocity (rad s)", 9.0);

    private final RobotState robotState = RobotState.getInstance();

    private final Drive drive;
    private final Supplier<Pose2d> targetPose;

    private Optional<Double> distanceTolerance = Optional.empty();
    private Optional<Double> angleTolerance = Optional.empty();

    /**
     * Constructs a DriveToPoseBase command.
     *
     * @param drive The drive subsystem to control
     * @param targetPose Supplier providing the target pose to drive to
     */
    public DriveToPose(Drive drive, Supplier<Pose2d> targetPose) {
        this.drive = drive;
        this.targetPose = targetPose;

        angularController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drive);
    }

    /**
     * Sets the distance tolerance for the command to finish
     *
     * @param tolerance Allowable distance to target pose
     */
    public DriveToPose withDistanceTolerance(Distance tolerance) {
        distanceTolerance = Optional.of(tolerance.in(Meters));
        return this;
    }

    /**
     * Sets the angular tolerance for the command to finish
     *
     * @param tolerance Allowable angle to target pose
     */
    public DriveToPose withAngularTolerance(Angle tolerance) {
        angleTolerance = Optional.of(tolerance.in(Radians));
        return this;
    }

    /**
     * Sets both distance and angular tolerances for the command to finish
     *
     * @param distanceTolerance Allowable distance to target pose
     * @param angleTolerance Allowable angle to target pose
     */
    public DriveToPose withTolerance(Distance distanceTolerance, Angle angleTolerance) {
        this.distanceTolerance = Optional.of(distanceTolerance.in(Meters));
        this.angleTolerance = Optional.of(angleTolerance.in(Radians));
        return this;
    }

    @Override
    public void initialize() {
        ChassisSpeeds fieldVelocity =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        drive.getChassisSpeeds(), robotState.getEstimatedPose().getRotation());

        linearController.reset(0.0);

        angularController.reset(
                robotState.getEstimatedPose().getRotation().getRadians(),
                fieldVelocity.omegaRadiansPerSecond);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Checks if tunable values for PID have changed and updates them if so
        linearController.updatePID();
        angularController.updatePID();

        // Calculate translation and direction to target
        Translation2d translationToTarget =
                targetPose
                        .get()
                        .getTranslation()
                        .minus(robotState.getEstimatedPose().getTranslation());

        Rotation2d directionToTarget = translationToTarget.getAngle();

        // Calculate outputs from controllers
        double linearOutput = -linearController.calculate(translationToTarget.getNorm());

        linearOutput = MathUtil.clamp(linearOutput, -maxLinearVel.get(), maxLinearVel.get());

        double angularOutput =
                angularController.calculate(
                        robotState.getEstimatedPose().getRotation().getRadians(),
                        targetPose.get().getRotation().getRadians());

        angularOutput = MathUtil.clamp(angularOutput, -maxAngularVel.get(), maxAngularVel.get());

        // Convert to robot-relative speeds and set request velocities
        var fieldRelativeSpeed =
                new ChassisSpeeds(
                        linearOutput * Math.cos(directionToTarget.getRadians()),
                        linearOutput * Math.sin(directionToTarget.getRadians()),
                        angularOutput);

        drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        fieldRelativeSpeed, robotState.getEstimatedPose().getRotation()));

        Logger.recordOutput("DriveToPose/Target Pose", targetPose.get());
        Logger.recordOutput("DriveToPose/Distance To Target (m)", translationToTarget.getNorm());
        Logger.recordOutput("DriveToPose/Angle To Target (deg)", directionToTarget.getDegrees());
        Logger.recordOutput(
                "DriveToPose/LinearController/Error", linearController.getPositionError());
        Logger.recordOutput("DriveToPose/LinearController/Output", linearOutput);
        Logger.recordOutput(
                "DriveToPose/AngularController/Error", angularController.getPositionError());
        Logger.recordOutput("DriveToPose/AngularController/Output", angularOutput);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean withinDistanceTolerance =
                distanceTolerance
                        .map(tolerance -> Math.abs(linearController.getPositionError()) < tolerance)
                        .orElse(true);

        boolean withinAngularTolerance =
                angleTolerance
                        .map(
                                tolerance ->
                                        Math.abs(angularController.getPositionError()) < tolerance)
                        .orElse(true);

        Logger.recordOutput(
                "DriveToPose/Distance Tolerance Present", distanceTolerance.isPresent());
        Logger.recordOutput("DriveToPose/Within Distance Tolerance", withinDistanceTolerance);
        Logger.recordOutput("DriveToPose/Angular Tolerance Present", angleTolerance.isPresent());
        Logger.recordOutput("DriveToPose/Within Angular Tolerance", withinAngularTolerance);

        boolean bothTolerancesSupplied =
                distanceTolerance.isPresent() && angleTolerance.isPresent();

        return bothTolerancesSupplied
                ? (withinDistanceTolerance && withinAngularTolerance)
                : (withinDistanceTolerance || withinAngularTolerance);
    }

    /**
     * Gets the current distance error to the target pose.
     *
     * @return The distance error in meters
     */
    public Distance getDistanceError() {
        return Meters.of(linearController.getPositionError());
    }

    /**
     * Gets the current angular error to the target pose.
     *
     * @return The angular error in radians
     */
    public Angle getAngularError() {
        return Radians.of(angularController.getPositionError());
    }
}
