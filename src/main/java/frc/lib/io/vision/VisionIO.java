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

package frc.lib.io.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;

public interface VisionIO {
    public static class VisionIOInputs implements LoggableInputs {
        public boolean connected = false;
        public PhotonPipelineResult[] results = new PhotonPipelineResult[0];

        private boolean hasLoggedIntrinsics = false;
        public double[] cameraMatrix = null;
        public double[] distCoeffs = null;

        public VisionIOInputs(Matrix<N3, N3> cameraMatrix, Matrix<N8, N1> distCoeffs)
        {
            this.cameraMatrix = cameraMatrix.getData();
            this.distCoeffs = distCoeffs.getData();
        }

        @Override
        public void toLog(LogTable table)
        {
            if (!hasLoggedIntrinsics) {
                table.put("CameraMatrix", cameraMatrix);
                table.put("DistCoeffs", distCoeffs);

                hasLoggedIntrinsics = true;
            }

            table.put("Connected", connected);

            int resultsLength = results.length;
            table.put("ResultsLength", resultsLength);
            String resultsPrefix = "Results/";
            for (int i = 0; i < resultsLength; i++) {
                String key = resultsPrefix + i;
                table.put(key, results[i]);
            }
        }

        @Override
        public void fromLog(LogTable table)
        {
            if (!hasLoggedIntrinsics) {
                cameraMatrix = table.get("CameraMatrix", (double[]) null);
                distCoeffs = table.get("DistCoeffs", (double[]) null);

                if (cameraMatrix != null && distCoeffs != null) {
                    hasLoggedIntrinsics = true;
                }
            }

            connected = table.get("Connected", false);

            int resultsLength = table.get("ResultsLength", 0);
            String resultsPrefix = "Results/";
            results = new PhotonPipelineResult[resultsLength];
            for (int i = 0; i < resultsLength; i++) {
                String key = resultsPrefix + i;
                results[i] = table.get(key, new PhotonPipelineResult());
            }
        }
    }

    public default void updateInputs(VisionIOInputs inputs)
    {}
}
