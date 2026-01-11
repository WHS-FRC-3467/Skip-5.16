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

package frc.lib.io.objectdetection;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Standardized interface for ObjectDetection-IO used in FRC. This interface is often implemented
 * through an ML pipeline.
 */
public interface ObjectDetectionIO {

    /*
     * Class defining data type for updateInputs method.
     */
    public class ObjectDetectionIOInputs implements LoggableInputs {
        /** Whether the camera is connected. */
        public boolean connected = false;
        /**
         * Each index of latestPhotonTrackedTargets is a single {@link PhotonTrackedTarget} with all
         * the data needed for each target
         */
        public PhotonTrackedTarget[] latestTargets = new PhotonTrackedTarget[0];

        @Override
        public void toLog(LogTable table)
        {
            int targetsLength = latestTargets.length;
            table.put("TargetsLength", targetsLength);

            String targetsPrefix = "Targets/";
            for (int i = 0; i < targetsLength; i++) {
                String targetKey = targetsPrefix + i;
                table.put(targetKey, latestTargets[i]);
            }
        }

        @Override
        public void fromLog(LogTable table)
        {
            int targetsLength = table.get("TargetsLength", 0);
            latestTargets = new PhotonTrackedTarget[targetsLength];

            String targetsPrefix = "Targets/";
            for (int i = 0; i < targetsLength; i++) {
                String targetKey = targetsPrefix + i;
                latestTargets[i] = table.get(targetKey, (PhotonTrackedTarget) null);
            }
        }
    }

    /*
     * Name of the camera capturing optical data.
     */
    public default String getCamera()
    {
        return "";
    }

    /*
     * Updates the provided ObjectDetectionIOInputs object using the latest camera readings. If the
     * camera is not connected, the ObjectDetectionIOInput fields remain empty.
     */
    public default void updateInputs(ObjectDetectionIOInputs inputs)
    {}
}
