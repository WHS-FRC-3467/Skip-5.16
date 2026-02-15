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

package frc.lib.devices;

import org.littletonrobotics.junction.Logger;
import frc.lib.io.beambreak.BeamBreakIO;
import frc.lib.io.beambreak.BeamBreakInputsAutoLogged;

/**
 * Wrapper class for beam break sensors that detect when an object breaks an infrared beam. Commonly
 * used for detecting game pieces in the robot. Provides a simplified interface over BeamBreakIO
 * implementations.
 */
public class BeamBreak {
    private final BeamBreakIO io;
    private final BeamBreakInputsAutoLogged inputs = new BeamBreakInputsAutoLogged();
    private final String name;

    /**
     * Constructs a Beam Break.
     *
     * @param name the name to use for logging
     * @param io the IO to interact with.
     */
    public BeamBreak(String name, BeamBreakIO io) {
        this.name = name;
        this.io = io;
    }

    /**
     * Call this method periodically to update sensor readings and log data.
     */
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    /**
     * Whether the beam is broken
     *
     * @return Whether the beam is broken
     */
    public boolean isBroken() {
        return inputs.isBroken;
    }

}
