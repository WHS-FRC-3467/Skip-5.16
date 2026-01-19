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

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Meters;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.units.measure.Distance;
import frc.lib.devices.BeamBreak;
import frc.lib.io.beambreak.BeamBreakIO;
import frc.lib.io.beambreak.BeamBreakIOLaserCAN;
import frc.lib.io.beambreak.BeamBreakIOSim;
import frc.robot.Constants;
import frc.robot.Ports;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class TurretSensorConstants {
    public static final String NAME = "IndexerBeamBreak";
    public static final Distance TRIGGET_DISTANCE = Meters.of(0.0);
    public static final RangingMode RANGING_MODE = RangingMode.SHORT;
    public static final TimingBudget TIMING_BUDGET = TimingBudget.TIMING_BUDGET_20MS;
    public static final RegionOfInterest ROI =
        new RegionOfInterest(20, 20, 100, 100);

    public static BeamBreakIOLaserCAN getIOReal()
    {
        return new BeamBreakIOLaserCAN(Ports.indexerLaserCAN, NAME, TRIGGET_DISTANCE, RANGING_MODE,
            ROI, TIMING_BUDGET);
    }

    public static BeamBreakIOSim getIOSim()
    {
        return new BeamBreakIOSim(NAME);
    }

    public static BeamBreakIO getIOReplay()
    {
        return new BeamBreakIO() {};
    }

    public static BeamBreak get()
    {
        return switch (Constants.currentMode) {
            case REAL -> new BeamBreak(getIOReal());
            case SIM -> new BeamBreak(getIOSim());
            case REPLAY -> new BeamBreak(getIOReplay());
        };
    }
}
