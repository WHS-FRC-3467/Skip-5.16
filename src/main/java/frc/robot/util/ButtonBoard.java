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

/** Extended CommandGenericHID with additional functionality for FRC use. */
public class ButtonBoard extends CommandGenericHID {
    public ButtonBoard(int port) {
        super(port);
    }

    public Trigger button1() {
        return button(1);
    }

    public Trigger button2() {
        return button(2);
    }

    public Trigger button3() {
        return button(3);
    }

    public Trigger button4() {
        return button(4);
    }

    public Trigger button5() {
        return button(5);
    }

    public Trigger button6() {
        return button(6);
    }
}
