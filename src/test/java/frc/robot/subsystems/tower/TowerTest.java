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

package frc.robot.subsystems.tower;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.TestUtil;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TowerTest {
    Tower tower;

    @BeforeEach // this method will run before each test
    void setup() {
        assertTrue(HAL.initialize(500, 0)); // initialize the HAL, crash if failed

        CommandScheduler.getInstance().cancelAll();
        tower = TowerConstants.get();

        /* enable the robot */
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        /* delay ~100ms so the devices can start up and enable */
        Timer.delay(0.100);
    }

    @AfterEach // this method will run after each test
    void shutdown() {
        try {
            CommandScheduler.getInstance().cancelAll();
            tower.close();
        } catch (Exception e) {
            fail("Failed to close KickerRoller subsystem: " + e.getMessage());
        }
    }

    @Test // marks this method as a test
    void shoot() {
        TestUtil.runTest(tower.shoot(), 2, tower);
        try {
            // Check velocity to check if the subsystem is actually in tolerance of intake velocity.
            assertTrue(tower.nearSetpoint());
        } catch (Exception e) {
            fail("Failed to run KickerRoller to shoot: " + e.getMessage());
        }
    }

    @Test // marks this method as a test
    void feed() {
        TestUtil.runTest(tower.feed(), 2, tower);
        try {
            // Check velocity to check if the subsystem is actually in tolerance of intake velocity.
            assertTrue(tower.nearSetpoint());
        } catch (Exception e) {
            fail("Failed to run KickerRoller to idle: " + e.getMessage());
        }
    }

    @Test
    void stop() {
        TestUtil.runTest(tower.stopCommand(), 2, tower);
        try {
            // Check velocity to check if the subsystem is actually in tolerance of stopped
            // velocity.
            assertTrue(tower.getSpeed() < 0.1);
        } catch (Exception e) {
            fail("Failed to stop indexer: " + e.getMessage());
        }
    }

    @Test
    void tuningModeToggleWithNoScheduledCommandDoesNotThrow() {
        CommandScheduler.getInstance().cancelAll();

        assertDoesNotThrow(
                () -> {
                    tower.enableTuningMode();
                    tower.disableTuningMode();
                });
    }
}
