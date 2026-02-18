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

package frc.lib.io.lights;

import com.ctre.phoenix6.controls.ControlRequest;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/** A simulated lights implementation */
public class LightsIOSim implements LightsIO {

    private Map<String, String> requestInfo;

    /** Constructs a {@link LightsIOSim} object. */
    public LightsIOSim() {}

    @Override
    public void setAnimation(ControlRequest request) {
        this.requestInfo = request.getControlInfo();
        if (requestInfo.containsKey("Slot")) {
            // Logs control request data for each slot
            Logger.recordOutput("LEDs/Slot" + requestInfo.get("Slot"), requestInfo.toString());
        }
    }
}
