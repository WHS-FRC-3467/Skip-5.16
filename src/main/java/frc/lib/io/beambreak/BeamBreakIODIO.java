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

import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.util.Device;

/** A beam break implementation that uses any DIO input */
public class BeamBreakIODIO implements BeamBreakIO {

    private final DigitalInput dio;

    /** Constructs a {@link BeamBreakIODIO} object with the specified DIO ID */
    public BeamBreakIODIO(Device.DIO id) {
        dio = new DigitalInput(id.id());
    }

    @Override
    public void updateInputs(BeamBreakInputs inputs) {
        inputs.isBroken = dio.get();
    }
}
