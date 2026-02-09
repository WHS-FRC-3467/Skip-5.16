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

import java.util.List;
import java.util.Optional;
import java.util.TreeSet;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.devices.Lights;
import frc.lib.io.lights.LightsIO;
import frc.lib.util.LoggerHelper;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/**
 * Subsystem that controls the robot's LED lights for visual feedback and animations. Provides
 * commands for different animation patterns during disabled, autonomous, and teleop modes.
 */
public class LEDs extends SubsystemBase {
    /**
     * The states for the lights in order from highest priority to low
     */
    @Getter
    @RequiredArgsConstructor
    @SuppressWarnings("ImmutableEnumChecker")
    public enum State {
        // list of states with their respective priorities, ie if both RUNNING_AUTO and
        // RUNNING_INTAKE are true it will set to RUNNING_AUTO

        RUNNING_AUTO(0, LEDsConstants.autoAnimation),
        READY_TO_SHOOT(1, LEDsConstants.offAnimation),
        RUNNING_INTAKE(2, LEDsConstants.offAnimation),
        NONE(3, LEDsConstants.offAnimation);

        // Lower priority first
        private final int priority;
        private final List<ControlRequest> animation;
    }

    private final Lights lights;

    private final TreeSet<State> stateQueue;
    private State currentState = State.NONE;

    /**
     * Constructs an LEDs subsystem.
     * 
     * @param io The lights IO interface for controlling the LED hardware
     */
    public LEDs(LightsIO io)
    {
        lights = new Lights(io);

        stateQueue = new TreeSet<>((a, b) -> Integer.compare(a.getPriority(), b.getPriority()));

        lights.setAnimations(currentState.getAnimation());
    }

    private Optional<State> getCurrentStateFromQueue()
    {

        State currentState = stateQueue.isEmpty() ? null : stateQueue.first();

        return Optional.ofNullable(currentState);
    }

    private boolean updateCurrentState()
    {
        Optional<State> newState = getCurrentStateFromQueue();

        boolean hasChanged = !this.currentState.equals(newState.orElse(null));

        this.currentState = newState.orElse(State.NONE);
        return hasChanged;
    }

    private void updateState()
    {
        boolean hasChanged = updateCurrentState();
        if (hasChanged) {
            lights.setAnimations(currentState.getAnimation());
        }
    }

    @Override
    public void periodic()
    {
        updateState();


        LoggerHelper.recordCurrentCommand(LEDsConstants.NAME, this);
        Logger.recordOutput(LEDsConstants.NAME + "/State", currentState.name());
        Logger.recordOutput(LEDsConstants.NAME + "/StateQueue",
            stateQueue
                .stream()
                .map(s -> s.name())
                .toArray(String[]::new));
    }

    public Command scheduleStateCommand(State state)
    {
        return this.runOnce(() -> stateQueue.add(state));
    }

    public Command unscheduleStateCommand(State state)
    {
        return this.runOnce(() -> stateQueue.remove(state));
    }
}
