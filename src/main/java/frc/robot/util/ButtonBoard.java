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
package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Extended CommandGenericHID with additional functionality for FRC use. This class is intended to
 * provide support for a generic digital button board with multiple buttons/switches and LEDs/buzzers
 * but no axes.
 */
public class ButtonBoard extends CommandGenericHID {
    public ButtonBoard(int port) {
        super(port);
    }

    public Trigger button(int buttonNumber) {
        return super.button(buttonNumber);
    }

    // public Command flashLED(double seconds) {
    //     return Commands.startEnd(
    //         () -> setLED(true),
    //         () -> setLED(false)).withTimeout(seconds);
    // }
}
