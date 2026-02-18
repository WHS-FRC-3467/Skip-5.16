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

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.TestUtil;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.Arrays;
import org.apache.commons.lang3.ArrayUtils;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriveTest {
    static final double DELTA = 1e-2; // acceptable deviation range
    static final String SKID_DETECTION_ENABLED_KEY = "Drive/Enable Skid Detection";
    static final String FRONT_LEFT_SKID_SIM_KEY = "Drive/FrontLeft/Enable Skid Simulation";
    Drive drive;

    @BeforeEach // this method will run before each test
    void setup() {
        assertTrue(HAL.initialize(500, 0)); // initialize the HAL, crash if failed
        setTunableBoolean(SKID_DETECTION_ENABLED_KEY, true);
        setTunableBoolean(FRONT_LEFT_SKID_SIM_KEY, false);

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

    @Test
    void testSkidDetectionNoOutlierAtNominalDrive() {
        installSyntheticModules(
                new double[] {0.0, 0.0, 0.0, 0.0}, new double[] {0.04, 0.04, 0.04, 0.04});
        boolean[] badWheels = invokeSkidMaskForSyntheticSample(1, 0.02);

        assertArrayEquals(
                new boolean[] {false, false, false, false},
                badWheels,
                "Expected no wheels flagged as skidding during nominal drive");
    }

    @Test
    void testSkidDetectionFlagsFrontLeftOutlier() {
        installSyntheticModules(
                new double[] {0.0, 0.0, 0.0, 0.0}, new double[] {0.12, 0.04, 0.04, 0.04});
        boolean[] badWheels = invokeSkidMaskForSyntheticSample(1, 0.02);

        assertTrue(
                badWheels[0],
                "Expected front-left wheel to be flagged for a clear translational outlier");
        assertTrue(
                Arrays.stream(ArrayUtils.toObject(badWheels)).anyMatch(w -> w),
                "Expected at least one wheel to be flagged for a translational outlier");
    }

    @Test
    void testSkidDetectionCanBeDisabled() {
        setTunableBoolean(FRONT_LEFT_SKID_SIM_KEY, true);
        setTunableBoolean(SKID_DETECTION_ENABLED_KEY, false);

        TestUtil.runTest(
                Commands.run(() -> drive.runVelocity(new ChassisSpeeds(2.0, 0.0, 0.0))),
                1.0,
                drive);
        assertArrayEquals(
                new boolean[] {false, false, false, false},
                readSkidBadWheelsLatest(),
                "Expected skid mask to stay clear when skid detection is disabled");
    }

    private static void setTunableBoolean(String key, boolean value) {
        NetworkTableInstance.getDefault().getEntry("/Tuning/" + key).setBoolean(value);
        NetworkTableInstance.getDefault().flushLocal();
    }

    private boolean[] readSkidBadWheelsLatest() {
        try {
            Field skidMaskField = Drive.class.getDeclaredField("skidBadWheelsLatest");
            skidMaskField.setAccessible(true);
            return ((boolean[]) skidMaskField.get(drive)).clone();
        } catch (ReflectiveOperationException e) {
            fail("Failed to read skid mask from Drive: " + e.getMessage());
            return new boolean[] {false, false, false, false};
        }
    }

    private void installSyntheticModules(double[] previousMeters, double[] currentMeters) {
        try {
            Field modulesField = Drive.class.getDeclaredField("modules");
            modulesField.setAccessible(true);
            Module[] modules = (Module[]) modulesField.get(drive);

            for (int i = 0; i < modules.length; i++) {
                modules[i] =
                        new Module(
                                new SyntheticModuleIO(
                                        getModuleConstants(i).WheelRadius,
                                        previousMeters[i],
                                        currentMeters[i]),
                                i,
                                getModuleConstants(i));
                modules[i].periodic();
            }
        } catch (ReflectiveOperationException e) {
            fail("Failed to install synthetic module data: " + e.getMessage());
        }
    }

    private boolean[] invokeSkidMaskForSyntheticSample(int sampleIndex, double dtSeconds) {
        try {
            Method skidMethod =
                    Drive.class.getDeclaredMethod(
                            "computeSkidMaskForSample",
                            int.class,
                            double[].class,
                            SwerveModulePosition[].class);
            skidMethod.setAccessible(true);

            double[] sampleTimestamps = new double[] {0.0, dtSeconds};
            SwerveModulePosition[] positionsNow = new SwerveModulePosition[4];
            Field modulesField = Drive.class.getDeclaredField("modules");
            modulesField.setAccessible(true);
            Module[] modules = (Module[]) modulesField.get(drive);

            for (int i = 0; i < 4; i++) {
                positionsNow[i] = modules[i].getOdometryPositions()[sampleIndex];
            }

            return (boolean[])
                    skidMethod.invoke(drive, sampleIndex, sampleTimestamps, positionsNow);
        } catch (ReflectiveOperationException e) {
            fail("Failed to invoke skid mask calculation: " + e.getMessage());
            return new boolean[] {false, false, false, false};
        }
    }

    private static SwerveModuleConstants<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            getModuleConstants(int index) {
        return switch (index) {
            case 0 -> DriveConstants.FrontLeft;
            case 1 -> DriveConstants.FrontRight;
            case 2 -> DriveConstants.BackLeft;
            default -> DriveConstants.BackRight;
        };
    }

    private static class SyntheticModuleIO implements ModuleIO {
        private final double wheelRadiusMeters;
        private final double[] odometryDistancesMeters;

        SyntheticModuleIO(
                double wheelRadiusMeters,
                double previousDistanceMeters,
                double currentDistanceMeters) {
            this.wheelRadiusMeters = wheelRadiusMeters;
            this.odometryDistancesMeters =
                    new double[] {previousDistanceMeters, currentDistanceMeters};
        }

        @Override
        public void updateInputs(ModuleIOInputs inputs) {
            inputs.driveConnected = true;
            inputs.turnConnected = true;
            inputs.turnEncoderConnected = true;
            inputs.turnPosition = new Rotation2d();
            inputs.turnAbsolutePosition = new Rotation2d();
            inputs.drivePositionRad = odometryDistancesMeters[1] / wheelRadiusMeters;
            inputs.driveVelocityRadPerSec =
                    (odometryDistancesMeters[1] - odometryDistancesMeters[0])
                            / 0.02
                            / wheelRadiusMeters;
            inputs.odometryTimestamps = new double[] {0.0, 0.02};
            inputs.odometryDrivePositionsRad =
                    new double[] {
                        odometryDistancesMeters[0] / wheelRadiusMeters,
                        odometryDistancesMeters[1] / wheelRadiusMeters
                    };
            inputs.odometryTurnPositions = new Rotation2d[] {new Rotation2d(), new Rotation2d()};
        }
    }
}
