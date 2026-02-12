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

package frc.lib.util;

import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Extended Xbox controller with additional functionality for FRC use.
 *
 * <p>
 * This class extends WPILib's CommandXboxController to add configurable deadbands, input curves,
 * and rumble commands. These features help improve driver control precision and provide tactile
 * feedback during matches.
 *
 * <p>
 * Example usage:
 *
 * <pre>{@code
 * CommandXboxControllerExtended driver = new CommandXboxControllerExtended(0)
 *     .withDeadband(0.1) // Add 10% deadband to all sticks
 *     .applyCurve(true); // Square inputs for finer control at low speeds
 *
 * // Use rumble for feedback
 * Command scoreSignal = driver.rumbleForTime(RumbleType.kBothRumble, 0.5, Seconds.of(0.3));
 * }</pre>
 */
public class CommandXboxControllerExtended extends CommandXboxController {
    private GenericHID hid;
    private double deadband = 0.0;
    private boolean applyCurve = false;

    /**
     * Constructs an extended Xbox controller with additional functionality.
     *
     * @param port The port the controller is plugged into
     */
    public CommandXboxControllerExtended(int port) {
        super(port);
        hid = this.getHID();
    }

    /**
     * Apply a deadband to all sticks
     *
     * @param deadband The percent deadband to apply
     * @return this
     */
    public CommandXboxControllerExtended withDeadband(double deadband) {
        this.deadband = deadband;
        return this;
    }

    /**
     * Square the output of the controllers to provide precise control
     *
     * @param applyCurve Whether or not to apply the curve
     * @return this
     */
    public CommandXboxControllerExtended applyCurve(boolean applyCurve) {
        this.applyCurve = applyCurve;
        return this;
    }

    /**
     * Rumble controller until command ends
     *
     * @param side Which motor to rumble
     * @param intensity Percentage for rumble intensity
     * @return Command to rumble the controller
     */
    public Command rumble(RumbleType side, double intensity) {
        return Commands.startEnd(() -> hid.setRumble(side, intensity),
            () -> hid.setRumble(side, 0.0));
    }

    /**
     * Rumble controller for a set amount of time
     *
     * @param side Which motor to rumble
     * @param intensity Percentage for rumble intensity
     * @param time Length of time to rumble
     * @return Command to rumble the controller
     */
    public Command rumbleForTime(RumbleType side, double intensity, Time time) {
        return Commands.runOnce(() -> hid.setRumble(side, intensity))
            .andThen(Commands.waitSeconds(time.in(Seconds)))
            .andThen(() -> hid.setRumble(side, 0.0));
    }

    double applyModifiers(double joystickInput) {
        if (applyCurve) {
            joystickInput = joystickInput * joystickInput;
        }

        return MathUtil.applyDeadband(joystickInput, deadband);
    }

    @Override
    public double getLeftX() {
        return applyModifiers(super.getLeftX());
    }

    @Override
    public double getLeftY() {
        return applyModifiers(super.getLeftY());
    }

    @Override
    public double getRightX() {
        return applyModifiers(super.getRightX());
    }

    @Override
    public double getRightY() {
        return applyModifiers(super.getRightY());
    }
}
