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

package frc.lib.io.beambreak;

import org.littletonrobotics.junction.AutoLog;

/** Standardized interface for beam breaks used in FRC. */
public interface BeamBreakIO {

    @AutoLog
    abstract class BeamBreakInputs {
        /** Whether the beam is broken */
        public boolean isBroken = false;
    }

    /**
     * Getter for the name of the sensor
     * 
     * @return The name of the sensor
     */
    public default String getName()
    {
        return "";
    }

    /**
     * Updates the provided {@link BeamBreakInputs} instance with the latest sensor readings. If the
     * sensor is not connected, it populates the fields with default values.
     *
     * @param inputs The structure to populate with updated sensor values.
     */
    public default void updateInputs(BeamBreakInputs inputs)
    {}
}
