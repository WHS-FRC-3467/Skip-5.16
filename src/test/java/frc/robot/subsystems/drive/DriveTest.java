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

package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.TestUtil;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriveTest {
    static final double DELTA = 1e-2; // acceptable deviation range
    Drive drive;

    @BeforeEach // this method will run before each test
    void setup() {
        assertTrue(HAL.initialize(500, 0)); // initialize the HAL, crash if failed
        drive =
                new Drive(
                        new GyroIO() {},
                        new ModuleIOSim("FrontLeft", DriveConstants.FrontLeft),
                        new ModuleIOSim("FrontRight", DriveConstants.FrontRight),
                        new ModuleIOSim("BackLeft", DriveConstants.BackLeft),
                        new ModuleIOSim("BackRight", DriveConstants.BackRight));

        /* enable the robot */
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        /* delay ~100ms so the devices can start up and enable */
        Timer.delay(0.100);
    }

    @Test
    void robotIsEnabled() {
        /* verify that the robot is enabled */
        try {
            assertTrue(DriverStation.isEnabled());
        } catch (Exception e) {
            fail("Robot is not enabled: " + e.getMessage());
        }
    }

    @Test // marks this method as a test
    void testStop() {
        TestUtil.runTest(Commands.runOnce(() -> drive.stop()), 0.1, drive);
        try {
            assertEquals(0.0, drive.getFFCharacterizationVelocity(), DELTA); // make sure that the
            // speed of the motor
            // is 0
        } catch (Exception e) {
            fail("Failed to stop the drive: " + e.getMessage());
        }
    }

    @Test
    void testDriveVelocity() {
        TestUtil.runTest(
                Commands.run(() -> drive.runVelocity(new ChassisSpeeds(1.5, 1.5, 0.0))), 1, drive);
        try {
            assertEquals(1.5, drive.getChassisSpeeds().vxMetersPerSecond, DELTA);
            assertEquals(1.5, drive.getChassisSpeeds().vyMetersPerSecond, DELTA);
        } catch (Exception e) {
            fail(
                    "Failed to run drive linear velocity of 1.5 m/s in the x direction and 3 m/s in the y direction: "
                            + e.getMessage());
        }
    }

    @Test
    void testSteerVelocity() {
        TestUtil.runTest(
                Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 1.5))), 1, drive);
        try {
            assertEquals(1.5, drive.getChassisSpeeds().omegaRadiansPerSecond, DELTA);
        } catch (Exception e) {
            fail("Failed to run drive rotational velocity of 1.5 rad/s: " + e.getMessage());
        }
    }

    @Test
    void testX() {
        TestUtil.runTest(drive.runOnce(() -> drive.stopWithX()), 0.1, drive);
        try {
            SwerveModulePosition[] swerveModulePositions = drive.getModulePositions();
            Rotation2d[] targetAngles = {
                new Rotation2d(
                        Math.atan(
                                DriveConstants.FrontLeft.LocationY
                                        / DriveConstants.FrontLeft.LocationX)), // Front
                // Left
                new Rotation2d(
                        Math.atan(
                                DriveConstants.FrontRight.LocationY
                                        / DriveConstants.FrontRight.LocationX)), // Front
                // Right
                new Rotation2d(
                        Math.atan(
                                DriveConstants.BackLeft.LocationY
                                        / DriveConstants.BackLeft.LocationX)), // Back
                // Left
                new Rotation2d(
                        Math.atan(
                                DriveConstants.BackRight.LocationY
                                        / DriveConstants.BackRight.LocationX)) // Back
                // Right
            };
            // Test position of modules
            for (int i = 0; i < swerveModulePositions.length; i++) {
                assertEquals(
                        targetAngles[i].getRadians(),
                        swerveModulePositions[i].angle.getRadians(),
                        DELTA);
            }
            assertEquals(0.0, drive.getChassisSpeeds().vxMetersPerSecond, DELTA); // make sure that
            // the drivetrain
            // reaches a
            // velocity of
            // zero
            assertEquals(0.0, drive.getChassisSpeeds().vyMetersPerSecond, DELTA);
            assertEquals(0.0, drive.getChassisSpeeds().omegaRadiansPerSecond, DELTA);

        } catch (Exception e) {
            fail(
                    "Failed to turn the Drive modules into an X arrangement to resist movement: "
                            + e.getMessage());
        }
    }
}
