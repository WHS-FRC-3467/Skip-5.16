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

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Velocity;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.io.motor.MotorIOTalonFX;
import frc.lib.io.motor.MotorIOTalonFXSim;
import frc.lib.mechanisms.linear.*;
import frc.lib.mechanisms.linear.LinearMechanism.LinearMechCharacteristics;
import frc.lib.util.MechanismUtil.DistanceAngleConverter;
import frc.lib.util.PID;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;

public class ClimberConstants {
    public static String NAME = "Climber";

    public static final Distance TOLERANCE = Inches.of(2.0);

    public static final AngularVelocity CRUISE_VELOCITY =
        Units.RadiansPerSecond.of(2 * Math.PI).times(10.0);
    public static final AngularAcceleration ACCELERATION =
        CRUISE_VELOCITY.div(0.1).per(Units.Second);
    public static final Velocity<AngularAccelerationUnit> JERK = ACCELERATION.per(Second);

    private static final double GEARING = (2.0 / 1.0);
    private static final Distance MIN_DISTANCE = Inches.of(0.0);
    private static final Distance MAX_DISTANCE = Inches.of(36.0);
    private static final Distance STARTING_DISTANCE = Inches.of(0.0);

    private static final Distance DRUM_RADIUS = Inches.of(1.0);
    private static final Mass CARRIAGE_MASS = Kilograms.of(.01);

    private static final DCMotor DCMOTOR = DCMotor.getKrakenX60(1);

    public static final DistanceAngleConverter CONVERTER = new DistanceAngleConverter(DRUM_RADIUS);

    // Orientation for the linear mechanism.
    // Uses WPILib's counter-clockwise positive convention around Y-axis:
    // A pitch of -90 degrees represents a vertical mechanism extending upward (like an elevator).
    // Pitch of 0 degrees would be horizontal extending forward.
    // Roll and yaw can be used for mechanisms that extend in other directions.
    public static final Rotation3d ORIENTATION =
        new Rotation3d(0.0, Degrees.of(0.0).in(Units.Radians), 0.0);

    private static final LinearMechCharacteristics CHARACTERISTICS =
        new LinearMechCharacteristics(MIN_DISTANCE, MAX_DISTANCE,
            STARTING_DISTANCE, CONVERTER, ORIENTATION);

    public static final PID SLOT0_PID = new PID(80.0, 0.0, 0.0);

    /**
     * Creates and configures a TalonFX motor controller configuration for the climber mechanism.
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

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
            CONVERTER.toAngle(MAX_DISTANCE).in(Rotations);

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
            CONVERTER.toAngle(MIN_DISTANCE).in(Rotations);

        config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY.in(RotationsPerSecond);
        config.MotionMagic.MotionMagicAcceleration = ACCELERATION.in(RotationsPerSecondPerSecond);

        config.Feedback.RotorToSensorRatio = 1.0;

        config.Feedback.SensorToMechanismRatio = GEARING;

        config.Slot0 = Slot0Configs.from(SLOT0_PID.toSlotConfigs());

        return config;
    }

    /**
     * Factory method to create a Climber instance for the climber subsystem. Creates the
     * appropriate subsystem based on the current robot mode (REAL, SIM, or REPLAY).
     *
     * @return A configured Climber instance
     */
    public static Climber get() {
        LinearMechanism<?> mechanism;
        switch (Constants.currentMode) {
            case REAL:
                mechanism = new LinearMechanismReal(NAME,
                    new MotorIOTalonFX(NAME, getFXConfig(), Ports.climber), CHARACTERISTICS);
                break;
            case SIM:
                mechanism = new LinearMechanismSim(NAME,
                    new MotorIOTalonFXSim(NAME, getFXConfig(), Ports.climber),
                    DCMOTOR, CARRIAGE_MASS, CHARACTERISTICS, false);
                break;
            case REPLAY:
                mechanism = new LinearMechanism<>(NAME, CHARACTERISTICS, new MotorIO() {}) {};
                break;
            default:
                throw new IllegalStateException("Unrecognized Robot Mode");
        }
        mechanism.enableTunablePID(PIDSlot.SLOT_0, SLOT0_PID);
        return new Climber(mechanism);
    }
}
