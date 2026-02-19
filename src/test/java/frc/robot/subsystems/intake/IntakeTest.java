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

import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.TestUtil;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class IntakeTest {
    IntakeSuperstructure intake;

    @BeforeEach // this method will run before each test
    void setup() {
        assertTrue(HAL.initialize(500, 0)); // initialize the HAL, crash if failed

        intake = IntakeSuperstructureConstants.get();

        /* enable the robot */
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        /* delay ~100ms so the devices can start up and enable */
        Timer.delay(0.100);
    }

    @AfterEach // this method will run after each test
    void shutdown() {
        try {
            intake.close();
        } catch (Exception e) {
            fail("Failed to close Intake subsystem: " + e.getMessage());
        }
    }

    @Test // marks this method as a test
    void intake() {
        TestUtil.runTest(intake.extendIntake(), 2, intake);
        try {
            // Check velocity to check if the subsystem is actually in tolerance of intake velocity.
            assertTrue(intake.isIntaking());
        } catch (Exception e) {
            fail("Failed to run Intake to intake: " + e.getMessage());
        }
    }

    /**
     * Mirrors the "Intake Linear/Extend" SmartDashboard command that the simulation-tester agent
     * exercises via NetworkTables. After the command completes the motor shaft position (logged by
     * AdvantageKit to /AdvantageKit/Intake Linear/position) should equal MAX_DISTANCE / DRUM_RADIUS
     * ≈ 22.748 rad.
     */
    @Test
    void extendedLinearPosition() {
        // Run the same command that SmartDashboard exposes as "Intake Linear/Extend".
        TestUtil.runTest(intake.extendIntake(), 3, intake);

        // Derive the motor shaft position in radians from the reported linear extension.
        // DistanceControlledMechanism.getLinearPosition() = position_rad * radius_m, so
        // position_rad = extension_m / radius_m.  This is the value AdvantageKit publishes
        // to /AdvantageKit/Intake Linear/position.
        double positionRad =
                intake.getExtension().in(Meters) / IntakeLinearConstants.DRUM_RADIUS.in(Meters);

        // Expected: MAX_DISTANCE / DRUM_RADIUS = 11.375 in / 0.5 in = 22.75 rad ≈ 22.748 rad.
        double expectedRad =
                IntakeLinearConstants.MAX_DISTANCE.in(Meters)
                        / IntakeLinearConstants.DRUM_RADIUS.in(Meters);
        assertEquals(
                expectedRad,
                positionRad,
                0.5, // ±0.5 rad tolerance (~0.25 in linear)
                "Motor position should be ~22.748 rad (MAX_DISTANCE / DRUM_RADIUS) when fully"
                        + " extended");
    }
}
