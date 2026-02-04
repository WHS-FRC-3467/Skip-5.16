/* Copyright (C) 2026 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

package frc.robot.subsystems.climber;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.TestUtil;

public class ClimberTest implements AutoCloseable {
    Climber climber;

    @BeforeEach // this method will run before each test
    void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

        climber = ClimberConstants.get();

        /* enable the robot */
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
      
        /* delay ~100ms so the devices can start up and enable */
        Timer.delay(0.100);
    }
  
    @SuppressWarnings("PMD.SignatureDeclareThrowsException")
    @AfterEach // this method will run after each test
    void shutdown() throws Exception {
      close();
    }
 
    @Test // marks this method as a test
    void home() {
        TestUtil.runTest(climber.homeCommand(), 0.1, climber);
        try {
            // Check position to check if it is homed, and within tolerance of STOW setpoint.
            assertTrue(climber.nearGoal(Climber.Setpoint.STOW.getSetpoint()));
        } catch (Exception e) {
            fail("Failed to home climber Subsystem: " + e.getMessage());
        }
    }

    @Test
    void goToGoal() {
        TestUtil.runTest(climber.setGoal(Climber.Setpoint.RAISED), 1.5, climber);
        try {
            // Check to see if climber subsystem is within tolerance of RAISED setpoint.
            assertTrue(climber.nearGoal(Climber.Setpoint.RAISED.getSetpoint()));
        } catch (Exception e) {
            fail("Failed to run climber Subsystem to RAISED: " + e.getMessage());
        }
    }
    
    @Override
    public void close() {
       climber.close();
    }
}
