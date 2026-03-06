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

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.PID;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public boolean driveConnected = false;
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public boolean turnConnected = false;
        public boolean turnEncoderConnected = false;
        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public Rotation2d turnPosition = new Rotation2d();
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ModuleIOInputs inputs) {}

    /**
     * Run the drive motor at the specified open loop value.
     *
     * @param output Open loop output value (-12 to 12 volts)
     */
    public default void setDriveOpenLoop(double output) {}

    /**
     * Run the turn motor at the specified open loop value.
     *
     * @param output Open loop output value (-12 to 12 volts)
     */
    public default void setTurnOpenLoop(double output) {}

    /**
     * Run the drive motor at the specified velocity.
     *
     * @param velocityRadPerSec Target velocity in radians per second
     */
    public default void setDriveVelocity(double velocityRadPerSec) {}

    /**
     * Run the turn motor to the specified rotation.
     *
     * @param rotation Target rotation angle
     */
    public default void setTurnPosition(Rotation2d rotation) {}

    /**
     * Updates the main PID slot on the drive motor
     *
     * @param pid The PID to set
     */
    public default void setDrivePID(PID pid) {}

    /**
     * Updates the main PID slot on the turn motor
     *
     * @param pid The PID to set
     */
    public default void setTurnPID(PID pid) {}
}
