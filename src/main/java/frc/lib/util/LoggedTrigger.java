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

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * A Trigger that automatically logs state changes to AdvantageKit.
 *
 * <p>
 * This class extends WPILib's Trigger and adds automatic logging of the trigger's state whenever it
 * changes. The logging is handled internally without requiring external method calls.
 *
 * <p>
 * Example usage:
 *
 * <pre>{@code
 * // In a subsystem, create a logged trigger that tracks when a beam break is triggered
 * public final LoggedTrigger broken = new LoggedTrigger("BeamBreak/Broken", beamBreak::isBroken);
 *
 * // The state will be automatically logged to "BeamBreak/Broken" whenever it changes
 * // Use it just like a normal Trigger
 * broken.onTrue(Commands.print("Beam is broken!"));
 * }</pre>
 */
public class LoggedTrigger extends Trigger {
    private final String name;

    /**
     * Creates a new LoggedTrigger that logs state changes.
     *
     * @param name The name to use as the log key for AdvantageKit logging
     * @param condition The condition to evaluate (typically a method reference or lambda)
     */
    public LoggedTrigger(String name, BooleanSupplier condition) {
        super(new LoggingBooleanSupplier(name, condition));
        this.name = name;
    }


    /**
     * Internal BooleanSupplier wrapper that logs state changes.
     */
    private static class LoggingBooleanSupplier implements BooleanSupplier {
        private final String logKey;
        private final BooleanSupplier condition;
        private boolean lastState = false;
        private boolean initialized = false;

        public LoggingBooleanSupplier(String logKey, BooleanSupplier condition) {
            this.logKey = logKey;
            this.condition = condition;
        }

        @Override
        public boolean getAsBoolean() {
            boolean currentState = condition.getAsBoolean();

            // Log on state change or first evaluation
            if (!initialized || currentState != lastState) {
                Logger.recordOutput(logKey, currentState);
                lastState = currentState;
                initialized = true;
            }

            return currentState;
        }
    }

    /**
     * Composes two logged triggers with logical AND.
     *
     * <p>
     * The resulting trigger uses a combined log name derived from both component triggers, so its
     * AdvantageKit signal clearly represents the composite condition rather than reusing the base
     * trigger's log key.
     *
     * @param trigger the condition to compose with
     * @return A trigger which is active when both component triggers are active.
     */
    public LoggedTrigger and(LoggedTrigger trigger) {
        String combinedName = this.name + "_AND_" + trigger.name;
        return new LoggedTrigger(combinedName,
            () -> (this.getAsBoolean() && trigger.getAsBoolean()));
    }


    /**
     * Creates a new debounced trigger from this trigger - it will become active when this trigger
     * has been active for longer than the specified period.
     *
     * @param seconds The debounce period.
     * @return The debounced trigger (rising edges debounced only)
     */
    @Override
    public LoggedTrigger debounce(double seconds) {
        return debounce(seconds, Debouncer.DebounceType.kRising);
    }

    /**
     * Creates a new debounced trigger from this trigger - it will become active when this trigger
     * has been active for longer than the specified period.
     *
     * @param seconds The debounce period.
     * @param type The debounce type.
     * @return The debounced trigger.
     */
    @Override
    public LoggedTrigger debounce(double seconds, Debouncer.DebounceType type) {
        return new LoggedTrigger(name, super.debounce(seconds, type));
    }
}
