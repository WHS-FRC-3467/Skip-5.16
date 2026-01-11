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

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class LoggedTriggerTest {
    
    private boolean conditionValue = false;
    
    @BeforeEach
    void setup()
    {
        assertTrue(HAL.initialize(500, 0)); // initialize the HAL, crash if failed
    }
    
    @AfterEach
    void shutdown()
    {
        // Cleanup if needed
    }
    
    @Test
    void testTriggerBehavior()
    {
        // Create a LoggedTrigger with a simple condition
        LoggedTrigger trigger = new LoggedTrigger("Test/Trigger", () -> conditionValue);
        
        // Initially the condition is false
        assertFalse(trigger.getAsBoolean());
        
        // Change the condition to true
        conditionValue = true;
        assertTrue(trigger.getAsBoolean());
        
        // Change back to false
        conditionValue = false;
        assertFalse(trigger.getAsBoolean());
    }
    
    @Test
    void testMultiplePolls()
    {
        // Create a LoggedTrigger
        LoggedTrigger trigger = new LoggedTrigger("Test/MultiPoll", () -> conditionValue);
        
        // Poll multiple times with same value - should work correctly
        assertFalse(trigger.getAsBoolean());
        assertFalse(trigger.getAsBoolean());
        assertFalse(trigger.getAsBoolean());
        
        // Change state
        conditionValue = true;
        assertTrue(trigger.getAsBoolean());
        assertTrue(trigger.getAsBoolean());
        assertTrue(trigger.getAsBoolean());
    }
}
