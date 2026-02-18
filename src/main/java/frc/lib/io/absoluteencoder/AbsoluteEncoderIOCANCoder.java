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

package frc.lib.io.absoluteencoder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Angle;
import frc.lib.util.CANUpdateThread;
import frc.lib.util.Device;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Real hardware implementation of AbsoluteEncoderIO using CTRE CANcoder.
 *
 * <p>Interfaces with a CTRE CANcoder absolute magnetic encoder over the CAN bus. Provides
 * high-resolution absolute position sensing for mechanisms like swerve modules, arms, and turrets.
 * Uses Phoenix 6 API with signal-based updates.
 */
public class AbsoluteEncoderIOCANCoder implements AbsoluteEncoderIO {
    private static final Logger LOGGER =
            Logger.getLogger(AbsoluteEncoderIOCANCoder.class.getName());

    protected final CANcoder CANCoder;

    private final StatusSignal<Angle> angle;

    private final CANUpdateThread updateThread = new CANUpdateThread();

    /**
     * Constructs a CANcoder interface with the specified configuration.
     *
     * @param id CAN device identifier (ID and bus name)
     * @param configuration CANcoder configuration including magnet offset and sensor direction
     */
    public AbsoluteEncoderIOCANCoder(Device.CAN id, CANcoderConfiguration configuration) {
        CANCoder = new CANcoder(id.id(), new CANBus(id.bus()));

        updateThread
                .CTRECheckErrorAndRetry(() -> CANCoder.getConfigurator().apply(configuration))
                .exceptionally(
                        ex -> {
                            LOGGER.log(Level.SEVERE, ex.toString(), ex);
                            return null;
                        });

        angle = CANCoder.getAbsolutePosition();

        updateThread
                .CTRECheckErrorAndRetry(() -> angle.setUpdateFrequency(200))
                .exceptionally(
                        ex -> {
                            LOGGER.log(Level.SEVERE, ex.toString(), ex);
                            return null;
                        });
    }

    @Override
    public void updateInputs(AbsoluteEncoderInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(angle).isOK();

        inputs.angle = angle.getValue();
    }

    @Override
    public void close() {
        CANCoder.close();
    }
}
