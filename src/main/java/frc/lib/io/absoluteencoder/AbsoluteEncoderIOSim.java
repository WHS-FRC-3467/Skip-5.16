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

package frc.lib.io.absoluteencoder;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Simulation extension of AbsoluteEncoderIO for testing and simulation.
 *
 * <p>
 * Adds methods to programmatically set the encoder's simulated angle and velocity, allowing physics
 * simulation and test code to inject realistic encoder values without needing real hardware.
 */
public interface AbsoluteEncoderIOSim extends AbsoluteEncoderIO {
    /**
     * Sets the simulated encoder angle.
     *
     * @param angle The angle to simulate
     */
    public default void setAngle(Angle angle) {}

    /**
     * Sets the simulated encoder angular velocity.
     *
     * @param velocity The angular velocity to simulate
     */
    public default void setAngularVelocity(AngularVelocity velocity) {}
}
