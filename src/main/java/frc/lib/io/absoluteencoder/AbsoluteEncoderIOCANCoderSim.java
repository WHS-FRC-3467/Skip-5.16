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

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.sim.CANcoderSimState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.util.Device.CAN;

public class AbsoluteEncoderIOCANCoderSim extends AbsoluteEncoderIOCANCoder
    implements AbsoluteEncoderIOSim {

    private final CANcoderSimState simState;

    public AbsoluteEncoderIOCANCoderSim(CAN id, String name, CANcoderConfiguration configuration)
    {
        super(id, name, configuration);
        simState = CANCoder.getSimState();
    }

    @Override
    public void updateInputs(AbsoluteEncoderInputs inputs)
    {
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        super.updateInputs(inputs);
    }

    @Override
    public void setAngle(Angle angle)
    {
        simState.setRawPosition(angle);
    }

    @Override
    public void setAngularVelocity(AngularVelocity velocity)
    {
        simState.setVelocity(velocity);
    }
}
