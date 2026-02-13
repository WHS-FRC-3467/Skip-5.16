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

import static edu.wpi.first.units.Units.Millimeters;
import java.util.logging.Level;
import java.util.logging.Logger;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.util.Device;
import frc.lib.util.CANUpdateThread;
import frc.lib.util.LaserCANConfigurator;

/**
 * A beam break sensor implementation that uses a LaserCAN
 */
public class BeamBreakIOLaserCAN implements BeamBreakIO {
    private static final Logger LOGGER = Logger.getLogger(BeamBreakIOLaserCAN.class.getName());

    private final LaserCANConfigurator laserCAN;
    private final Distance triggerDistance;

    private final CANUpdateThread updateThread = new CANUpdateThread();

    private final Alert laserCANOnWrongBusAlert;
    private final Alert disconnectedAlert;

    /**
     * Constructs a new {@link BeamBreakIOLaserCAN} with specified parameters and configuration.
     *
     * @param id The CAN device ID and bus to which the sensor is connected.
     * @param name A human-readable name for the sensor instance.
     * @param rangingMode The ranging mode to configure on the sensor.
     * @param regionOfInterest The region of interest setting for the sensor.
     * @param timingBudget The timing budget setting that controls measurement speed/accuracy.
     */
    public BeamBreakIOLaserCAN(Device.CAN id, String name, Distance triggerDistance,
        RangingMode rangingMode,
        RegionOfInterest regionOfInterest, TimingBudget timingBudget) {
        this.triggerDistance = triggerDistance;

        laserCANOnWrongBusAlert =
            new Alert("LaserCAN " + name + " must be wired to the RIO's CAN bus",
                AlertType.kError);
        disconnectedAlert = new Alert("LaserCAN " + name + " is not connected", AlertType.kError);

        if (!id.bus().equals("rio")) {
            laserCANOnWrongBusAlert.set(true);
        }

        laserCAN = new LaserCANConfigurator(id.id());

        updateThread.laserCANCheckErrorAndRetry(() -> laserCAN.setRangingMode(rangingMode))
            .exceptionally(ex -> {
                LOGGER.log(Level.SEVERE, ex.toString(), ex);
                return null;
            });

        updateThread
            .laserCANCheckErrorAndRetry(() -> laserCAN.setRegionOfInterest(regionOfInterest))
            .exceptionally(ex -> {
                LOGGER.log(Level.SEVERE, ex.toString(), ex);
                return null;
            });

        updateThread.laserCANCheckErrorAndRetry(() -> laserCAN.setTimingBudget(timingBudget))
            .exceptionally(ex -> {
                LOGGER.log(Level.SEVERE, ex.toString(), ex);
                return null;
            });
    }

    @Override
    public void updateInputs(BeamBreakInputs inputs) {
        Measurement measure = laserCAN.getMeasurement();

        if (measure == null) {
            disconnectedAlert.set(true);

            inputs.isBroken = false;
            return;
        }

        disconnectedAlert.set(false);

        if (measure.status != LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT) {
            inputs.isBroken = false;
            return;
        }

        inputs.isBroken = Millimeters.of(measure.distance_mm).gte(triggerDistance);
    }
}
