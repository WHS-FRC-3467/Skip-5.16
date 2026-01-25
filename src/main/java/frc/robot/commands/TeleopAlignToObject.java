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

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.objectdetector.ObjectDetector;
import frc.lib.devices.ObjectDetection.ContourSelectionMode;

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

    public TeleopAlignToObject(Drive drive, ObjectDetector objectDetector,
        ContourSelectionMode mode, DoubleSupplier xSupplier,
        DoubleSupplier ySupplier, DoubleSupplier rotSupplier)
    {
        this.drive = drive;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotSupplier = rotSupplier;
        this.strategy = new AlignToObjectBase(objectDetector, mode,
            DriveCommands.getANGLE_MAX_VELOCITY(), DriveCommands.getANGLE_MAX_ACCELERATION()) {};
        // Reserve drive for this command
        addRequirements(drive);
    }

    @Override
    public void initialize()
    {
        // Conservatively reset upon initialization
        strategy.getAngularController().reset(0.0);
    }

    @Override
    public void execute()
    {
        // Take translation inputs from joystick
        // Apply linear velocity shaping
        Translation2d linearVelocity = DriveCommands
            .getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
        double vx = linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec();
        double vy = linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec();

        // Implement heading strategy; if fails, fallback to joystick heading
        // Apply deadband and shaping to fallback
        double omega = strategy.getVisionOmega().orElseGet(() -> {
            double raw =
                MathUtil.applyDeadband(rotSupplier.getAsDouble(), DriveCommands.getDEADBAND());
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
                    ? drive.robotState.getEstimatedPose().getRotation()
                        .plus(Rotation2d.k180deg)
                    : drive.robotState.getEstimatedPose().getRotation()));
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
