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

package frc.lib.util;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;

public class LaserCANConfigurator implements AutoCloseable {
    public enum ConfigurationStatus {
        SUCCESS,
        FAILURE;
    }

    private final LaserCan laserCAN;

    public LaserCANConfigurator(int can_id)
    {
        laserCAN = new LaserCan(can_id);
    }

    public Measurement getMeasurement()
    {
        return laserCAN.getMeasurement();
    }

    public ConfigurationStatus setRangingMode(RangingMode mode)
    {
        try {
            laserCAN.setRangingMode(mode);
            return ConfigurationStatus.SUCCESS;
        } catch (ConfigurationFailedException exception) {
            return ConfigurationStatus.FAILURE;
        }
    }

    public ConfigurationStatus setTimingBudget(TimingBudget budget)
    {
        try {
            laserCAN.setTimingBudget(budget);
            return ConfigurationStatus.SUCCESS;
        } catch (ConfigurationFailedException exception) {
            return ConfigurationStatus.FAILURE;
        }
    }

    public ConfigurationStatus setRegionOfInterest(RegionOfInterest roi)
    {
        try {
            laserCAN.setRegionOfInterest(roi);
            return ConfigurationStatus.SUCCESS;
        } catch (ConfigurationFailedException exception) {
            return ConfigurationStatus.FAILURE;
        }
    }

    @Override
    public void close() throws Exception
    {
        laserCAN.close();
    }
}
