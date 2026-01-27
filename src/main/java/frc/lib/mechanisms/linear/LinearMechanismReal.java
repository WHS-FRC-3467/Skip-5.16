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

package frc.lib.mechanisms.linear;

import edu.wpi.first.units.measure.Angle;
import frc.lib.io.motor.MotorIO;

/**
 * A real implementation of the LinearMechanism class that interacts with a physical motor
 * through a MotorIO interface.
 */
public class LinearMechanismReal extends LinearMechanism<MotorIO> {
    public LinearMechanismReal(String name, MotorIO io,
        LinearMechCharacteristics characteristics)
    {
        super(name, characteristics, io);
    }

    // TODO: Verify operation works correctly on real TalonFX
    @Override
    public void setEncoderPosition(Angle position)
    {
        io.setEncoderPosition(position);
    }
}
