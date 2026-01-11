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

import edu.wpi.first.units.measure.Angle;
import frc.lib.io.servo.ServoIO;

public class Servo {
    private final ServoIO io;

    /**
     * Constructs a Servo.
     */
    public Servo(ServoIO io)
    {
        this.io = io;
    }

    /**
     * Sets the servo position using a scaled value.
     *
     * @param angle position, where an angle with measure 0 corresponds to the leftmost position of
     *        the servo.
     */
    public void setAngle(Angle angle)
    {
        io.setAngle(angle);
    }

    /**
     * If servo is real, this method disables the PWM output until told to run to a position again.
     */
    public void stop()
    {
        io.stop();
    }
}
