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

package frc.lib.io.beambreak;

import frc.lib.util.LoggedTunableBoolean;

/** A simulated implementation that uses a toggle button on the dashboard */
public class BeamBreakIOSim implements BeamBreakIO {

    private final LoggedTunableBoolean button;

    /**
     * Constructs a {@link BeamBreakIOSim} object with the specified DIO ID
     *
     * @param name A human readable name for this sensor
     */
    public BeamBreakIOSim(String name) {
        this.button = new LoggedTunableBoolean(name, false);
    }

    @Override
    public void updateInputs(BeamBreakInputs inputs) {
        inputs.isBroken = button.getAsBoolean();
    }
}
