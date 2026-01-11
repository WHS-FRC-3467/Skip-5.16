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

package frc.lib.devices;

import java.util.List;
import com.ctre.phoenix6.controls.ControlRequest;

import frc.lib.io.lights.LightsIO;

/**
 * Class for simplified Lights implementation
 */
public class Lights {
    private final LightsIO io;

    /**
     * Constructs Lights.
     *
     * @param io the IO to interact with.
     */
    public Lights(LightsIO io)
    {
        this.io = io;
    }

    /**
     * Passes ControlRequest to IO layer
     *
     * @param request {@link ControlRequest}
     */
    public void setAnimation(ControlRequest request)
    {
        io.setAnimation(request);
    }

    /**
     * Passes ControlRequests to IO layer
     *
     * @param requests {@link ControlRequest}
     */
    public void setAnimations(List<ControlRequest> requests)
    {
        for (ControlRequest request : requests) {
            io.setAnimation(request);
        }
    }
}
