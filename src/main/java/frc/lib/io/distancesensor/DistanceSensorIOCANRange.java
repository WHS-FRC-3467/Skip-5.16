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

package frc.lib.io.distancesensor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.Device;
import frc.lib.util.CANUpdateThread;
import lombok.Getter;

/**
 * A distance sensor implementation that uses a CANRange
 */
public class DistanceSensorIOCANRange implements DistanceSensorIO {
    @Getter
    private final String name;
    private final CANrange CANRange;

    private final CANUpdateThread updateThread = new CANUpdateThread();

    private final StatusSignal<Distance> distance;
    private final StatusSignal<Double> ambientSignal;

    /**
     * Constructs a {@link DistanceSensorIOCANRange} object with the specified CAN ID, name, and
     * configuration.
     *
     * @param id The CANDevice identifying the bus and device ID for this sensor.
     * @param name A human-readable name for this sensor instance.
     * @param config The CANrangeConfiguration to apply to the sensor upon initialization.
     */
    public DistanceSensorIOCANRange(Device.CAN id, String name, CANrangeConfiguration config)
    {
        this.name = name;

        CANRange = new CANrange(id.id(), id.bus());

        updateThread.CTRECheckErrorAndRetry(() -> CANRange.getConfigurator().apply(config));

        ambientSignal = CANRange.getAmbientSignal();
        distance = CANRange.getDistance();
    }

    @Override
    public void updateInputs(DistanceSensorInputs inputs)
    {
        inputs.connected = BaseStatusSignal.refreshAll(ambientSignal, distance).isOK();

        if (!inputs.connected) {
            inputs.ambientSignal = 0.0;
            inputs.distance = null;
            return;
        }

        inputs.ambientSignal = ambientSignal.getValue();
        inputs.distance = distance.getValue();
    }

}
