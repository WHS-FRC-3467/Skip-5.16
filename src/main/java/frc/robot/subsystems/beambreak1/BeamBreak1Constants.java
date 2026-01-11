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

package frc.robot.subsystems.beambreak1;

import frc.lib.io.beambreak.BeamBreakIO;
import frc.lib.io.beambreak.BeamBreakIODIO;
import frc.lib.io.beambreak.BeamBreakIOSim;
import frc.robot.Constants;
import frc.robot.Ports;

public class BeamBreak1Constants {

    public final static String NAME = "Beam Break #1";

    // private final static RangingMode RANGING_MODE = RangingMode.SHORT;
    // private final static RegionOfInterest ROI = new RegionOfInterest(8, 8, 4, 4);
    // private final static TimingBudget TIMING_BUDGET = TimingBudget.TIMING_BUDGET_20MS;


    // public static BeamBreakIOLaserCAN getReal()
    // {
    // return new BeamBreakIOLaserCAN(Ports.laserCAN1, NAME, Millimeters.of(100), RANGING_MODE,
    // ROI, TIMING_BUDGET);

    // }

    public static BeamBreak1 get()
    {
        switch (Constants.currentMode) {
            case REAL:
                return new BeamBreak1(new BeamBreakIODIO(Ports.diobeambreak, NAME));
            case SIM:
                return new BeamBreak1(new BeamBreakIOSim(NAME));
            case REPLAY:
                return new BeamBreak1(new BeamBreakIO() {});
            default:
                throw new IllegalStateException("Unrecognized Robot Mode");
        }
    }
}
