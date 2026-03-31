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

package frc.robot.subsystems.indexer;

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

public class IndexerTest {
    Indexer indexer;

    @BeforeEach // this method will run before each test
    void setup() {
        assertTrue(HAL.initialize(500, 0)); // initialize the HAL, crash if failed

        CommandScheduler.getInstance().cancelAll();
        indexer = IndexerConstants.get();

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
            indexer.close();
        } catch (Exception e) {
            fail("Failed to close IndexerSuperstructure subsystem: " + e.getMessage());
        }
    }

    @Test // marks this method as a test
    void shoot() {
        TestUtil.runTest(indexer.shoot(), 5, indexer);
        try {
            // Check velocity to check if the subsystem is actually in tolerance of intake velocity.
            assertTrue(indexer.nearSetpoint());
        } catch (Exception e) {
            fail("Failed to run Indexer to pull: " + e.getMessage());
        }
    }

    @Test
    void stop() {
        TestUtil.runTest(indexer.stopCommand(), 2, indexer);
        try {
            // Check velocity to check if the subsystem is actually in tolerance of stopped
            // velocity.
            assertTrue(indexer.getFloorSpeed() < 0.1);
        } catch (Exception e) {
            fail("Failed to stop Indexer: " + e.getMessage());
        }
    }

    @Test
    void tuningModeToggleWithNoScheduledCommandDoesNotThrow() {
        CommandScheduler.getInstance().cancelAll();

        assertDoesNotThrow(
                () -> {
                    indexer.enableTuningMode();
                    indexer.disableTuningMode();
                });
    }
}
