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

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * A Trigger that automatically logs state changes to AdvantageKit.
 * 
 * <p>This class extends WPILib's Trigger and adds automatic logging of the trigger's state
 * whenever it changes. The logging is handled internally without requiring external method calls.
 * 
 * <p>Example usage:
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
    /**
     * Creates a new LoggedTrigger that logs state changes.
     *
     * @param name The name to use as the log key for AdvantageKit logging
     * @param condition The condition to evaluate (typically a method reference or lambda)
     */
    public LoggedTrigger(String name, BooleanSupplier condition)
    {
        super(new LoggingBooleanSupplier(name, condition));
    }
    
    /**
     * Internal BooleanSupplier wrapper that logs state changes.
     */
    private static class LoggingBooleanSupplier implements BooleanSupplier {
        private final String logKey;
        private final BooleanSupplier condition;
        private boolean lastState = false;
        private boolean initialized = false;
        
        public LoggingBooleanSupplier(String logKey, BooleanSupplier condition)
        {
            this.logKey = logKey;
            this.condition = condition;
        }
        
        @Override
        public boolean getAsBoolean()
        {
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
}
