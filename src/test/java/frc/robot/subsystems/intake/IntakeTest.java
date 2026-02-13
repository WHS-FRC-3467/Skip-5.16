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

package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.TestUtil;

public class IntakeTest {
    IntakeSuperstructure intake;

    @BeforeEach // this method will run before each test
    void setup()
    {
        assertTrue(HAL.initialize(500, 0)); // initialize the HAL, crash if failed

        intake = IntakeSuperstructureConstants.get();

        /* enable the robot */
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        /* delay ~100ms so the devices can start up and enable */
        Timer.delay(0.100);
    }

    @AfterEach // this method will run after each test
    void shutdown()
    {
        try {
            intake.close();
        } catch (Exception e) {
            fail("Failed to close Intake subsystem: " + e.getMessage());
        }
    }

    @Test // marks this method as a test
    void intake()
    {
        TestUtil.runTest(
            intake.extendIntake(),
            2,
            intake);
        try {
            // Check velocity to check if the subsystem is actually in tolerance of intake velocity.
            assertTrue(intake.isIntaking());
        } catch (Exception e) {
            fail("Failed to run Intake to intake: " + e.getMessage());
        }
    }

}
