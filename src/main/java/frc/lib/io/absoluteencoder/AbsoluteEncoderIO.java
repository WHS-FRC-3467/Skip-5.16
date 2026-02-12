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
import org.littletonrobotics.junction.AutoLog;

/** Standardized interface for absolute encoders used in FRC. */
public interface AbsoluteEncoderIO extends AutoCloseable {

    @AutoLog
    abstract class AbsoluteEncoderInputs {
        /** Whether the sensor is connected. */
        public boolean connected = false;
        /** Angle the encoder reads. 0 &lt;= r &lt; 1 where r = angle in rotations */
        public Angle angle = null;
    }

    /**
     * Updates the provided {@link AbsoluteEncoderInputs} instance with the latest sensor readings.
     * If the sensor is not connected, it populates the fields with default values.
     *
     * @param inputs The structure to populate with updated sensor values.
     */
    public default void updateInputs(AbsoluteEncoderInputs inputs)
    {}

    @Override
    public default void close()
    {}
}
