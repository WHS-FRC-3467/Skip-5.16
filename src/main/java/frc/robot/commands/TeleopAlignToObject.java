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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.devices.ObjectDetection.ContourSelectionMode;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.objectdetector.ObjectDetector;
import java.util.function.DoubleSupplier;

/**
 * Command layer that actuates robot heading align robot with centroid of detected contour. Utilizes
 * a communal teleop/auto strategy layer containing the PID calculation.
 */
public class TeleopAlignToObject extends Command {
    private final Drive drive;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotSupplier;
    private final AlignToObjectBase strategy;
    private final RobotState robotState = RobotState.getInstance();

    /**
     * Creates a command to align the robot to a detected object while allowing driver control.
     *
     * @param drive the Drive subsystem to control robot movement
     * @param objectDetector the ObjectDetector subsystem to detect and track objects
     * @param mode the contour selection mode for choosing which detected object to align to
     * @param xSupplier a DoubleSupplier providing driver input for forward/backward movement
     * @param ySupplier a DoubleSupplier providing driver input for left/right movement
     * @param rotSupplier a DoubleSupplier providing fallback driver input for rotation if no object
     *     is detected
     */
    public TeleopAlignToObject(
            Drive drive,
            ObjectDetector objectDetector,
            ContourSelectionMode mode,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotSupplier) {
        this.drive = drive;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotSupplier = rotSupplier;
        this.strategy =
                new AlignToObjectBase(
                        objectDetector,
                        mode,
                        DriveCommands.getANGLE_MAX_VELOCITY(),
                        DriveCommands.getANGLE_MAX_ACCELERATION()) {};
        // Reserve drive for this command
        addRequirements(drive);
    }

    /** Initializes the command by resetting the angular controller to ensure a clean start. */
    @Override
    public void initialize() {
        // Conservatively reset upon initialization
        strategy.getAngularController().reset(0.0);
    }

    /**
     * Executes the alignment strategy by combining driver translational input with automated
     * rotational control to align to the detected object. Falls back to driver rotation input if no
     * object is detected.
     */
    @Override
    public void execute() {
        // Take translation inputs from joystick
        // Apply linear velocity shaping
        Translation2d linearVelocity =
                DriveCommands.getLinearVelocityFromJoysticks(
                        xSupplier.getAsDouble(), ySupplier.getAsDouble());
        double vx = linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec();
        double vy = linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec();

        // Implement heading strategy; if fails, fallback to joystick heading
        // Apply deadband and shaping to fallback
        double omega =
                strategy.getVisionOmega()
                        .orElseGet(
                                () -> {
                                    double raw =
                                            MathUtil.applyDeadband(
                                                    rotSupplier.getAsDouble(),
                                                    DriveCommands.getDEADBAND());
                                    raw = Math.copySign(raw * raw, raw);
                                    return raw * drive.getMaxAngularSpeedRadPerSec();
                                });
        // Generate scaled field-relative chassis speeds
        ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, omega);

        // Facing away from blue alliance = 0 degrees
        boolean isFlipped =
                DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red;

        // Command vx, vy, omega
        drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        isFlipped
                                ? robotState
                                        .getEstimatedPose()
                                        .getRotation()
                                        .plus(Rotation2d.k180deg)
                                : robotState.getEstimatedPose().getRotation()));
    }

    /**
     * Checks if the command is finished.
     *
     * @return false; this is a continuous teleop command that runs until interrupted.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
