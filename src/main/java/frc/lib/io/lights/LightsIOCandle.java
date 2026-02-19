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

package frc.lib.io.lights;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANdle;
import frc.lib.util.CANUpdateThread;
import frc.lib.util.Device;
import java.util.logging.Level;
import java.util.logging.Logger;

/** A lights implementation that uses a CANdle */
public class LightsIOCandle implements LightsIO {
    private static final Logger LOGGER = Logger.getLogger(LightsIOCandle.class.getName());

    private final CANdle candle;

    private final CANUpdateThread updateThread = new CANUpdateThread();

    /**
     * Constructs a {@link LightsIOCandle} object with the specified name, CAN id, and
     * configuration.
     *
     * @param id The Device identifying the bus and device ID for this sensor.
     * @param config The CANrangeConfiguration to apply to the sensor upon initialization.
     */
    public LightsIOCandle(Device.CAN id, CANdleConfiguration config) {

        candle = new CANdle(id.id(), new CANBus(id.bus()));

        updateThread
                .CTRECheckErrorAndRetry(() -> candle.getConfigurator().apply(config))
                .exceptionally(
                        ex -> {
                            LOGGER.log(Level.SEVERE, ex.toString(), ex);
                            return null;
                        });
    }

    @Override
    public void setAnimation(ControlRequest request) {
        candle.setControl(request);
    }
}
