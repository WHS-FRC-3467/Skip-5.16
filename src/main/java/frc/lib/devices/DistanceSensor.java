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

import edu.wpi.first.units.measure.Distance;

import frc.lib.io.distancesensor.DistanceSensorIO;
import frc.lib.io.distancesensor.DistanceSensorInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

import java.util.Optional;

/** Class for simplified DistanceSensorIO implementation */
public class DistanceSensor {
    private final String name;
    private final DistanceSensorIO io;
    private final DistanceSensorInputsAutoLogged inputs = new DistanceSensorInputsAutoLogged();

    /**
     * Constructs a Distance Sensor.
     *
     * @param name the name to use for logging
     * @param io the IO to interact with.
     */
    public DistanceSensor(String name, DistanceSensorIO io) {
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
    public Optional<Distance> getDistance() {
        return Optional.ofNullable(inputs.distance);
    }

    /**
     * Getter for the ambient light read by the sensor
     *
     * @return The ambient light read by the sensor
     */
    public double getAmbientSignal() {
        return inputs.ambientSignal;
    }

    /**
     * Whether the current measured reading is between a specified min and max distance
     *
     * @param min minimum distance to compare to
     * @param max maximum distance to compare to
     * @return boolean specifying whether current distance reading is contained [min, max]
     */
    public boolean betweenDistance(Distance min, Distance max) {
        Optional<Distance> distanceOpt = getDistance();
        if (distanceOpt.isEmpty()) {
            return false;
        }

        Distance distance = distanceOpt.get();
        return distance.gte(min) && distance.lte(max);
    }
}
