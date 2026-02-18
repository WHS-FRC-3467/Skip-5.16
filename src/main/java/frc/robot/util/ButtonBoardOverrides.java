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

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class implementing expected operator override methods using a button board. */
public class ButtonBoardOverrides implements OperatorOverrides {
    private GenericHID board;

    public ButtonBoardOverrides(int port) {
        board = new GenericHID(port);
    }

    @Override
    public Trigger emergencyStop() {
        return new Trigger(() -> board.getRawButton(1));
    }

    @Override
    public Trigger shooterSpinDown() {
        return new Trigger(() -> board.getRawButton(2));
    }

    @Override
    public Trigger unjam() {
        return new Trigger(() -> board.getRawButton(3));
    }

    @Override
    public Trigger forceRetractIntake() {
        return new Trigger(() -> board.getRawButton(4));
    }

    @Override
    public Trigger forceShot() {
        return new Trigger(() -> board.getRawButton(5));
    }

    @Override
    public Trigger lockDrive() {
        return new Trigger(() -> board.getRawButton(6));
    }
}
