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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * Hardware interface for vision cameras that detect AprilTags for robot localization.
 *
 * <p>This interface defines the contract for vision camera hardware, allowing the robot to read
 * camera results for pose estimation. Implementations handle vendor-specific camera APIs
 * (PhotonVision, Limelight, etc.) while the rest of the robot code remains hardware-agnostic.
 */
public interface VisionIO {
    /**
     * Container for vision camera sensor readings and camera intrinsics. Logged automatically by
     * AdvantageKit for replay and analysis.
     */
    public static class VisionIOInputs implements LoggableInputs {
        /** Whether the camera is connected and responding */
        public boolean connected = false;

        /** Array of pipeline results from the camera since last update */
        public PhotonPipelineResult[] results = new PhotonPipelineResult[0];

        private boolean hasLoggedIntrinsics = false;

        /** Camera intrinsic matrix (3x3) */
        public double[] cameraMatrix = null;

        /** Camera distortion coefficients (8x1) */
        public double[] distCoeffs = null;

        /** Constructs vision inputs without camera intrinsic parameters. */
        public VisionIOInputs() {}

        /**
         * Constructs vision inputs with camera intrinsic parameters.
         *
         * @param cameraMatrix 3x3 camera intrinsic matrix (may be null if unavailable)
         * @param distCoeffs 8x1 distortion coefficients (may be null if unavailable)
         */
        public VisionIOInputs(Matrix<N3, N3> cameraMatrix, Matrix<N8, N1> distCoeffs) {
            this.cameraMatrix = cameraMatrix != null ? cameraMatrix.getData() : null;
            this.distCoeffs = distCoeffs != null ? distCoeffs.getData() : null;
        }

        @Override
        public void toLog(LogTable table) {
            if (!hasLoggedIntrinsics) {
                if (cameraMatrix != null) {
                    table.put("CameraMatrix", cameraMatrix);
                }
                if (distCoeffs != null) {
                    table.put("DistCoeffs", distCoeffs);
                }

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
        public void fromLog(LogTable table) {
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

    /**
     * Updates the vision inputs with the latest readings from the camera. Called periodically by
     * the vision device layer.
     *
     * @param inputs The input object to populate with sensor data
     */
    public default void updateInputs(VisionIOInputs inputs) {}
}
