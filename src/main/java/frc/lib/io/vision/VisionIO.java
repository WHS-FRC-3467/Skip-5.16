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
 * Hardware interface for vision cameras that detect AprilTags for robot localization.
 *
 * <p>This interface defines the contract for vision camera hardware, allowing the robot to read
 * camera results for pose estimation. Implementations handle vendor-specific camera APIs
 * (PhotonVision, Limelight, etc.) while the rest of the robot code remains hardware-agnostic.
 */
public interface VisionIO {
    /**
     * Container for vision camera sensor readings. Logged automatically by AdvantageKit for replay
     * and analysis.
     */
    @AutoLog
    public static class VisionIOInputs {
        /** Whether the camera is connected and responding */
        public boolean connected = false;

        /** Raw unread frame payloads from the camera since last update. */
        public byte[][] rawResults = new byte[0][];
    }

    /**
     * Updates the vision inputs with the latest readings from the camera. Called periodically by
     * the vision device layer.
     *
     * @param inputs The input object to populate with sensor data
     */
    public default void updateInputs(VisionIOInputs inputs) {}
}
