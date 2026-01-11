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

package frc.lib.io.motor;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public interface MotorIOSim extends MotorIO {
    /**
     * Setter for the simulated mechanism position, typically taken from a WPILib mechanism
     * simulation
     * 
     * @param position The new mechanism position
     */
    public default void setPosition(Angle position)
    {}

    /**
     * Setter for the simulated mechanism velocity, typically taken from a WPILib mechanism
     * simulation
     * 
     * @param velocity The new mechanism velocity
     */
    public default void setRotorVelocity(AngularVelocity velocity)
    {}

    /**
     * Setter for the simulated mechanism acceleration, typically taken from a WPILib mechanism
     * simulation
     * 
     * @param acceleration The new mechanism acceleration
     */
    public default void setRotorAcceleration(AngularAcceleration acceleration)
    {}

    /**
     * Getter for the gear ratio to the sensor
     * 
     * @return The gear ratio to the sensor
     */
    public default double getRotorToSensorRatio()
    {
        return 0.0;
    }

    /**
     * Getter for the gear ratio from the sensor to the mechanism
     * 
     * @return The gear ratio from the sensor to the mechanism
     */
    public default double getSensorToMechanismRatio()
    {
        return 0.0;
    }
}
