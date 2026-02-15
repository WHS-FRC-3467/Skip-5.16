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

package frc.lib.devices;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Angle;
import frc.lib.io.absoluteencoder.AbsoluteEncoderIO;
import frc.lib.io.absoluteencoder.AbsoluteEncoderInputsAutoLogged;

/**
 * Class for simplified AbsoluteEncoderIO implementation
 */
public class AbsoluteEncoder {
    private final AbsoluteEncoderIO io;
    private final AbsoluteEncoderInputsAutoLogged inputs = new AbsoluteEncoderInputsAutoLogged();
    private final String name;

    /**
     * Constructs an Absolute Encoder.
     *
     * @param name the name to use for logging
     * @param io the IO to interact with.
     */
    public AbsoluteEncoder(String name, AbsoluteEncoderIO io) {
        this.name = name;
        this.io = io;
    }

    /** Call this method periodically */
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    /**
     * Whether the sensor is connected.
     *
     * @return Whether the sensor is connected
     */
    public boolean isConnected() {
        return inputs.connected;
    }

    /**
     * Getter for the distance read by the sensor
     *
     * @return The distance read by the sensor
     */
    public Angle getAngle() {
        return inputs.angle;
    }
}
