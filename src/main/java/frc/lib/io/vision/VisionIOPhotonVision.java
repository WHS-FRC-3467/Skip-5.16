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

import frc.lib.devices.AprilTagCamera.CameraProperties;

import org.photonvision.PhotonCamera;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * Real hardware implementation of VisionIO using PhotonVision.
 *
 * <p>Connects to a PhotonVision coprocessor running an AprilTag detection pipeline and reads vision
 * results over NetworkTables. Used for real robot operation.
 */
public class VisionIOPhotonVision implements VisionIO {
    private static final byte[] PHOTON_RESULT_MAGIC = new byte[] {'P', 'H', 'O', 'T', 'O', 'N', 1};

    protected final PhotonCamera photonCamera;

    /**
     * Constructs a PhotonVision camera interface.
     *
     * @param cameraProperties Camera configuration including name and calibration
     */
    public VisionIOPhotonVision(CameraProperties cameraProperties) {
        this.photonCamera = new PhotonCamera(cameraProperties.name());
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = photonCamera.isConnected();

        if (!inputs.connected) {
            inputs.rawResults = new byte[0][];
            inputs.captureTimestampsUs = new long[0];
            inputs.publishTimestampsUs = new long[0];
            return;
        }

        var unreadResults = photonCamera.getAllUnreadResults();
        inputs.rawResults =
                unreadResults.stream()
                        .map(VisionIOPhotonVision::packPhotonResult)
                        .toArray(byte[][]::new);
        inputs.captureTimestampsUs =
                unreadResults.stream()
                        .mapToLong(result -> result.metadata.captureTimestampMicros)
                        .toArray();
        inputs.publishTimestampsUs =
                unreadResults.stream()
                        .mapToLong(result -> result.metadata.publishTimestampMicros)
                        .toArray();
    }

    private static byte[] packPhotonResult(PhotonPipelineResult result) {
        Packet packet = new Packet(512);
        PhotonPipelineResult.photonStruct.pack(packet, result);
        byte[] packedResult = packet.getWrittenDataCopy();
        byte[] rawResult = new byte[PHOTON_RESULT_MAGIC.length + packedResult.length];
        System.arraycopy(PHOTON_RESULT_MAGIC, 0, rawResult, 0, PHOTON_RESULT_MAGIC.length);
        System.arraycopy(
                packedResult, 0, rawResult, PHOTON_RESULT_MAGIC.length, packedResult.length);
        return rawResult;
    }
}
