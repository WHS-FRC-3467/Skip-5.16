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

package frc.lib.io.servo;

import edu.wpi.first.units.measure.Angle;

public interface ServoIO {

    /**
     * Getter for the name of the servo
     * 
     * @return The name of the servo
     */
    public default String getName()
    {
        return "";
    }

    /**
     * Runs the servo to position using an {@link Angle} value. The value should not exceed the
     * lower and upper limits of the servo.
     */
    public default void setAngle(Angle position)
    {}

    /* Disables output until told to run to a position again. */
    public default void stop()
    {}

}
