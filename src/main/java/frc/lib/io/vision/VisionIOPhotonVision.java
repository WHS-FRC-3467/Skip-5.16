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

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import frc.lib.devices.AprilTagCamera.CameraProperties;

public class VisionIOPhotonVision implements VisionIO {
    protected final PhotonCamera photonCamera;

    public VisionIOPhotonVision(CameraProperties cameraProperties)
    {
        this.photonCamera = new PhotonCamera(cameraProperties.name());
    }

    @Override
    public void updateInputs(VisionIOInputs inputs)
    {
        inputs.connected = photonCamera.isConnected();

        if (!inputs.connected) {
            return;
        }

        inputs.results = photonCamera.getAllUnreadResults().toArray(PhotonPipelineResult[]::new);
    }
}
