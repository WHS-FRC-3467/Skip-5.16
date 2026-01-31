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

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.devices.Lights;
import frc.lib.io.lights.LightsIO;
import frc.lib.util.LoggerHelper;

public class LEDs extends SubsystemBase {
    private final Lights lights;

    /**
     * Constructs an LEDs subsystem.
     * 
     * @param io The lights IO interface for controlling the LED hardware
     */
    public LEDs(LightsIO io)
    {
        lights = new Lights(io);
    }

    @Override
    public void periodic()
    {
        LoggerHelper.recordCurrentCommand(LEDsConstants.NAME, this);
    }

    /**
     * Creates a command to run the disabled animation on the LEDs.
     * Turns off the LEDs when the command ends.
     * 
     * @return A command that runs the disabled animation
     */
    public Command runDisabledAnimation()
    {
        return this.startEnd(
            () -> lights.setAnimations(LEDsConstants.disabledAnimation),
            () -> lights.setAnimations(LEDsConstants.offAnimation))
            .withName("Disabled Animation");
    }

    /**
     * Creates a command to run the autonomous animation on the LEDs.
     * Turns off the LEDs when the command ends.
     * 
     * @return A command that runs the autonomous animation
     */
    public Command runAutoAnimation()
    {
        return this.startEnd(
            () -> lights.setAnimations(LEDsConstants.autoAnimation),
            () -> lights.setAnimations(LEDsConstants.offAnimation))
            .withName("Auto Animation");
    }
}
