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

import java.util.HashMap;

import com.ctre.phoenix6.CANBus;

public sealed interface Device {
    static final HashMap<String, CANBus> canBusMap = new HashMap<>();

    record CAN(int id, String bus, CANBus canBus) implements Device {
        public CAN(int id, String bus) {
            this(id, bus, canBusMap.computeIfAbsent(bus, CANBus::new));
        }
    }

    public record DIO(int id) implements Device {
    }

    public record PWM(int id) implements Device {
    }
}
