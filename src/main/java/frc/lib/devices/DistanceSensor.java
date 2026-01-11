/*
 * Copyright (C) 2025 Windham Windup
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

import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Distance;
import frc.lib.io.distancesensor.DistanceSensorIO;
import frc.lib.io.distancesensor.DistanceSensorInputsAutoLogged;

/**
 * Class for simplified DistanceSensorIO implementation
 */
public class DistanceSensor {
    private final DistanceSensorIO io;
    private final DistanceSensorInputsAutoLogged inputs = new DistanceSensorInputsAutoLogged();

    /**
     * Constructs a Distance Sensor.
     *
     * @param io the IO to interact with.
     */
    public DistanceSensor(DistanceSensorIO io)
    {
        this.io = io;
    }

    /** Call this method periodically */
    public void periodic()
    {
        io.updateInputs(inputs);
        Logger.processInputs(io.getName(), inputs);
    }

    /**
     * Whether the sensor is connected.
     * 
     * @return Whether the sensor is connected
     */
    public boolean isConnected()
    {
        return inputs.connected;
    }

    /**
     * Getter for the distance read by the sensor
     * 
     * @return The distance read by the sensor
     */
    public Optional<Distance> getDistance()
    {
        return Optional.ofNullable(inputs.distance);
    }

    /**
     * Getter for the ambient light read by the sensor
     * 
     * @return The ambient light read by the sensor
     */
    public double getAmbientSignal()
    {
        return inputs.ambientSignal;
    }

}
