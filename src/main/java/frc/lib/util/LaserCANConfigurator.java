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

package frc.lib.util;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;

public class LaserCANConfigurator implements AutoCloseable {
    private final LaserCan laserCAN;

    /**
     * Constructs a LaserCAN configurator.
     *
     * @param can_id CAN ID of the LaserCAN device
     */
    public LaserCANConfigurator(int can_id)
    {
        laserCAN = new LaserCan(can_id);
    }

    /**
     * Gets the latest measurement from the sensor.
     *
     * @return The most recent measurement
     */
    public Measurement getMeasurement()
    {
        return laserCAN.getMeasurement();
    }

    /**
     * Sets the ranging mode of the sensor.
     *
     * @param mode The ranging mode to use
     * @throws ConfigurationFailedException if configuration fails
     */
    public void setRangingMode(RangingMode mode) throws ConfigurationFailedException
    {
        laserCAN.setRangingMode(mode);
    }

    /**
     * Sets the timing budget of the sensor.
     *
     * @param budget The timing budget to use
     * @throws ConfigurationFailedException if configuration fails
     */
    public void setTimingBudget(TimingBudget budget) throws ConfigurationFailedException
    {
        laserCAN.setTimingBudget(budget);
    }

    /**
     * Sets the region of interest for the sensor.
     *
     * @param roi The region of interest to use
     * @throws ConfigurationFailedException if configuration fails
     */
    public void setRegionOfInterest(RegionOfInterest roi) throws ConfigurationFailedException
    {
        laserCAN.setRegionOfInterest(roi);
    }

    @Override
    public void close() throws Exception
    {
        laserCAN.close();
    }
}
