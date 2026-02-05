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

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.devices.Lights;
import frc.lib.io.lights.LightsIO;
import frc.lib.util.LoggerHelper;
import frc.robot.RobotContainer;
import lombok.RequiredArgsConstructor;

/**
 * Subsystem that controls the robot's LED lights for visual feedback and animations. Provides
 * commands for different animation patterns during disabled, autonomous, and teleop modes.
 */
public class LEDs extends SubsystemBase {
    public RobotContainer robotContainer;

    /**
     * The states for the lights in order from highest priority to low
     */
    @RequiredArgsConstructor
    @SuppressWarnings("ImmutableEnumChecker")
    public enum State {
        // list of states with their respective prioritys, ie if both RUNNING_AUTO and
        // RUNNING_INTAKE are true it will set to RUNNING_AUTO

        RUNNING_AUTO,
        SHOOTING,
        READY_TO_SHOOT,
        RUNNING_INTAKE,
        NONE;
    }

    private final Lights lights;

    private State setState = State.NONE; // the set state

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
        Logger.recordOutput(LEDsConstants.NAME, setState);
    }

    /**
     * Creates a command to run the disabled animation on the LEDs. Turns off the LEDs when the
     * command ends.
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
     * Creates a command to run the autonomous animation on the LEDs. Turns off the LEDs when the
     * command ends.
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

    // checks if the supplied states priotity is higher than the set states priority and if it
    // is it will set the led to the supplied state and set the setState to the supplied state
    public void smartHandler(State state)
    {
        if (state.ordinal() < setState.ordinal()) {
            setLED(state);
            setState = state;
        }
    }

    // same thing as smartHandler but it sets the state without checking anything as the logic is
    // handled in robotContainer

    public void dumbHandler(State state)
    {
        setLED(state);
        setState = state;
    }

    private void setLED(State state)
    {
        // just matches states with their animations, only 1 works
        switch (state) {
            case RUNNING_INTAKE -> runDisabledAnimation();

            case READY_TO_SHOOT -> runDisabledAnimation();

            case SHOOTING -> runDisabledAnimation();

            case RUNNING_AUTO -> runAutoAnimation();
            default -> {
            }
        }
    }



}
