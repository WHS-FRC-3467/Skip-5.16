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

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;

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

public class IndexerCenterConstants {

    public static final String NAME = "Indexer Center";

    public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(80);
    public static final AngularAcceleration MAX_ACCELERATION = MAX_VELOCITY.per(Second).times(10);
    public static final AngularVelocity TOLERANCE = RotationsPerSecond.of(5.0);

    public static final Distance RADIUS = Inches.of(0.5);

    private static final double GEARING = (40.0 / 16.0);
    private static final DCMotor DCMOTOR = DCMotor.getKrakenX44Foc(1);
    private static final MomentOfInertia MOI = KilogramSquareMeters.of(0.01);

    // Velocity PID
    public static final PID SLOT0_PID = new PID(100.0, 0.0, 0.0).withV(0.0);

    /**
     * Creates and configures a TalonFX motor controller configuration for the indexer.
     *
     * @return The configured TalonFX configuration
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

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted =
                RobotBase.isSimulation()
                        ? InvertedValue.CounterClockwise_Positive
                        : InvertedValue.Clockwise_Positive;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        config.Feedback.RotorToSensorRatio = 1.0;

        config.Feedback.SensorToMechanismRatio = GEARING;

        config.Slot0 = Slot0Configs.from(SLOT0_PID.toSlotConfigs());

        return config;
    }

    /**
     * Factory method to create an IndexerSuperstructure subsystem instance. Creates the appropriate
     * mechanism based on the current robot mode (REAL, SIM, or REPLAY).
     *
     * @return A fully configured IndexerSuperstructure subsystem
     */
    public static FlywheelMechanism<?> get() {
        FlywheelMechanism<?> mechanism;
        switch (Constants.currentMode) {
            case REAL:
                mechanism =
                        new FlywheelMechanismReal(
                                NAME,
                                new MotorIOTalonFX(NAME, getFXConfig(), Ports.indexerCentering));
                break;
            case SIM:
                mechanism =
                        new FlywheelMechanismSim(
                                NAME,
                                new MotorIOTalonFXSim(NAME, getFXConfig(), Ports.indexerCentering),
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
        return mechanism;
    }
}
