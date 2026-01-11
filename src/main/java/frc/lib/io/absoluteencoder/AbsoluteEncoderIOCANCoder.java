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

package frc.lib.io.absoluteencoder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Angle;
import frc.lib.util.CANUpdateThread;
import frc.lib.util.Device;
import lombok.Getter;

public class AbsoluteEncoderIOCANCoder implements AbsoluteEncoderIO {
    @Getter
    private final String name;
    protected final CANcoder CANCoder;

    private final StatusSignal<Angle> angle;

    private final CANUpdateThread updateThread = new CANUpdateThread();

    public AbsoluteEncoderIOCANCoder(
        Device.CAN id,
        String name,
        CANcoderConfiguration configuration)
    {
        this.name = name;
        CANCoder = new CANcoder(id.id(), id.canBus());

        updateThread.CTRECheckErrorAndRetry(() -> CANCoder.getConfigurator().apply(configuration));

        angle = CANCoder.getAbsolutePosition();

        updateThread.CTRECheckErrorAndRetry(() -> angle.setUpdateFrequency(200));
    }

    @Override
    public void updateInputs(AbsoluteEncoderInputs inputs)
    {
        inputs.connected = BaseStatusSignal.refreshAll(angle).isOK();

        inputs.angle = angle.getValue();
    }

    @Override
    public void close()
    {
        CANCoder.close();
    }
}
