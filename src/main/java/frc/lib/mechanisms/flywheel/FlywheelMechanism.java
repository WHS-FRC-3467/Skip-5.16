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

package frc.lib.mechanisms.flywheel;

import frc.lib.io.motor.MotorIO;
import frc.lib.mechanisms.Mechanism;

/**
 * Abstract base class for flywheel mechanisms. Flywheels are rotating mechanisms used for intakes,
 * indexers, and shooters. Provides velocity control for spinning wheels at specific speeds.
 *
 * @param <T> the type of MotorIO implementation used by this flywheel
 */
public abstract class FlywheelMechanism<T extends MotorIO> extends Mechanism<T> {

    public FlywheelMechanism(String name, T io) {
        super(name, io);
    }
}
