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

package frc.lib.io.vision;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware interface for CamCam vision coprocessors.
 *
 * <p>Implementations handle transport-specific configuration and packet delivery while the device
 * layer remains agnostic to NetworkTables, simulation, or replay details.
 */
public interface VisionIO extends AutoCloseable {
    /** Auto-logged raw data batches waiting to be decoded by the device layer. */
    @AutoLog
    public static class VisionIOInputs {
        /** Unread FlatBuffer payloads returned by the coprocessor for this cycle. */
        public byte[][] unreadData = new byte[][] {};
    }

    @Override
    public default void close() {}

    /**
     * Publishes a serialized CamCam configuration to the coprocessor.
     *
     * @param configuration FlatBuffer-encoded configuration payload
     */
    public default void configure(byte[] configuration) {}

    /**
     * Updates the vision inputs with the latest unread payloads from the coprocessor.
     *
     * @param inputs the input object to populate with sensor data
     */
    public default void updateInputs(VisionIOInputs inputs) {}
}
