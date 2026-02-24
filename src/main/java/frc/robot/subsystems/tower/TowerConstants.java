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

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.lib.devices.DistanceSensor;
import frc.lib.io.distancesensor.DistanceSensorIO;
import frc.lib.io.distancesensor.DistanceSensorIOLaserCAN;
import frc.lib.io.distancesensor.DistanceSensorIOSim;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.io.motor.MotorIOTalonFX;
import frc.lib.io.motor.MotorIOTalonFXSim;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.mechanisms.flywheel.FlywheelMechanismReal;
import frc.lib.mechanisms.flywheel.FlywheelMechanismSim;
import frc.lib.util.PID;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;

public class TowerConstants {
    public static String NAME = "Tower";

    public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(116.6);
    public static final AngularAcceleration MAX_ACCELERATION =
            RotationsPerSecondPerSecond.of(406.0);

    private static final double GEARING = (36.0 / 12.0);

    public static final AngularVelocity TOLERANCE = MAX_VELOCITY.times(0.2);

    private static final DCMotor DCMOTOR = DCMotor.getKrakenX44Foc(1);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.01);

    // Velocity PID
    public static final PID SLOT0_PID = new PID(80.0, 0.0, 0.0).withV(10.0);

    // LaserCAN shared configs
    private static final RangingMode LASERCAN_RANGING_MODE = RangingMode.SHORT;
    private static final RegionOfInterest LASERCAN_ROI = new RegionOfInterest(8, 8, 4, 4);
    private static final TimingBudget TIMING_BUDGET = TimingBudget.TIMING_BUDGET_20MS;
    public static final Distance MINIMUM_TRIP_DISTANCE = Millimeters.of(0.0);
    public static final Distance MAXIMUM_TRIP_DISTANCE = Millimeters.of(15);

    // LaserCAN #1
    public static final String LASERCAN1_NAME = NAME + "/LaserCAN1";

    // LaserCAN #2
    public static final String LASERCAN2_NAME = NAME + "/LaserCAN2";

    /**
     * Creates a TalonFX motor controller configuration for the tower mechanism. Configures current
     * limits, voltage limits, neutral mode, gearing ratios, and PID gains.
     *
     * @return configured TalonFXConfiguration for the tower motor
     */
    public static TalonFXConfiguration getFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

        config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.StatorCurrentLimit = 80.0;

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        config.Feedback.RotorToSensorRatio = 1.0;

        config.Feedback.SensorToMechanismRatio = GEARING;

        config.Slot0 = Slot0Configs.from(SLOT0_PID.toSlotConfigs());

        return config;
    }

    /**
     * Creates and configures a Tower subsystem based on the current robot mode. Selects the
     * appropriate flywheel mechanism implementation (real, sim, or replay).
     *
     * @return configured Tower instance
     */
    public static Tower get() {
        FlywheelMechanism<?> mechanism;
        switch (Constants.currentMode) {
            case REAL:
                mechanism =
                        new FlywheelMechanismReal(
                                NAME, new MotorIOTalonFX(NAME, getFXConfig(), Ports.tower));
                break;
            case SIM:
                mechanism =
                        new FlywheelMechanismSim(
                                NAME,
                                new MotorIOTalonFXSim(NAME, getFXConfig(), Ports.tower),
                                DCMOTOR,
                                MOI,
                                TOLERANCE);
                break;
            case REPLAY:
                mechanism = new FlywheelMechanism<>(NAME, new MotorIO() {}) {};
                break;
            default:
                throw new IllegalStateException("Unrecognized Robot Mode");
        }
        mechanism.enableTunablePID(PIDSlot.SLOT_0, SLOT0_PID);
        return new Tower(mechanism);
    }

    // Return an IO implementation of distance sensor IO based on current robot state
    public static DistanceSensor getLaserCAN1() {
        return new DistanceSensor(
                LASERCAN1_NAME,
                switch (Constants.currentMode) {
                    case REAL ->
                            new DistanceSensorIOLaserCAN(
                                    Ports.towerLaserCAN1,
                                    LASERCAN1_NAME,
                                    LASERCAN_RANGING_MODE,
                                    LASERCAN_ROI,
                                    TIMING_BUDGET);

                    case SIM -> new DistanceSensorIOSim(LASERCAN1_NAME);

                    case REPLAY -> new DistanceSensorIO() {};

                    default -> throw new IllegalArgumentException("Unrecognized Robot Mode");
                });
    }

    public static DistanceSensor getLaserCAN2() {
        return new DistanceSensor(
                LASERCAN2_NAME,
                switch (Constants.currentMode) {
                    case REAL ->
                            new DistanceSensorIOLaserCAN(
                                    Ports.towerLaserCAN2,
                                    LASERCAN2_NAME,
                                    LASERCAN_RANGING_MODE,
                                    LASERCAN_ROI,
                                    TIMING_BUDGET);

                    case SIM -> new DistanceSensorIOSim(LASERCAN2_NAME);

                    case REPLAY -> new DistanceSensorIO() {};

                    default -> throw new IllegalArgumentException("Unrecognized Robot Mode");
                });
    }
}
