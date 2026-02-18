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

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;
    private final SwerveModuleConstants<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            constants;

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private final Alert turnEncoderDisconnectedAlert;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    /**
     * Constructs a new Module instance.
     *
     * @param io IO interface for the module
     * @param index Module index (0-3: FL, FR, BL, BR)
     * @param constants Module-specific constants from DriveConstants
     */
    public Module(
            ModuleIO io,
            int index,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                    constants) {
        this.io = io;
        this.index = index;
        this.constants = constants;
        driveDisconnectedAlert =
                new Alert(
                        "Disconnected drive motor on module " + Integer.toString(index) + ".",
                        AlertType.kError);
        turnDisconnectedAlert =
                new Alert(
                        "Disconnected turn motor on module " + Integer.toString(index) + ".",
                        AlertType.kError);
        turnEncoderDisconnectedAlert =
                new Alert(
                        "Disconnected turn encoder on module " + Integer.toString(index) + ".",
                        AlertType.kError);
    }

    /** Updates inputs, processes odometry data, and updates connection alerts. */
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
            Rotation2d angle = inputs.odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        // Update alerts
        driveDisconnectedAlert.set(!inputs.driveConnected);
        turnDisconnectedAlert.set(!inputs.turnConnected);
        turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
    }

    /**
     * Runs the module with the specified setpoint state. Mutates the state to optimize it.
     *
     * @param state Desired module state (speed and angle), will be optimized
     */
    public void runSetpoint(SwerveModuleState state) {
        // Optimize velocity setpoint
        state.optimize(getAngle());
        state.cosineScale(inputs.turnPosition);

        // Apply setpoints
        io.setDriveVelocity(state.speedMetersPerSecond / constants.WheelRadius);
        io.setTurnPosition(state.angle);
    }

    /**
     * Runs the module with the specified output while controlling to zero degrees.
     *
     * @param output Drive motor output voltage
     */
    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setTurnPosition(new Rotation2d());
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.setDriveOpenLoop(0.0);
        io.setTurnOpenLoop(0.0);
    }

    /**
     * Returns the current turn angle of the module.
     *
     * @return Current module angle
     */
    public Rotation2d getAngle() {
        return inputs.turnPosition;
    }

    /**
     * Returns the current drive position of the module in meters.
     *
     * @return Drive position in meters
     */
    public double getPositionMeters() {
        return inputs.drivePositionRad * constants.WheelRadius;
    }

    /**
     * Returns the current drive velocity of the module in meters per second.
     *
     * @return Drive velocity in meters per second
     */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * constants.WheelRadius;
    }

    /**
     * Returns the module position (turn angle and drive position).
     *
     * @return Module position containing angle and distance
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /**
     * Returns the module state (turn angle and drive velocity).
     *
     * @return Module state containing angle and velocity
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /**
     * Returns the module positions received this cycle.
     *
     * @return Array of module positions from this cycle
     */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    /**
     * Returns the timestamps of the samples received this cycle.
     *
     * @return Array of timestamps in seconds
     */
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    /**
     * Returns the module position in radians.
     *
     * @return Drive position in radians
     */
    public double getWheelRadiusCharacterizationPosition() {
        return inputs.drivePositionRad;
    }

    /**
     * Returns the module velocity in rotations/sec (Phoenix native units).
     *
     * @return Drive velocity in rotations per second
     */
    public double getFFCharacterizationVelocity() {
        return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
    }
}
