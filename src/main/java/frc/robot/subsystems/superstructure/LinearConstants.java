// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

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
import frc.lib.io.motor.MotorIOTalonFX;
import frc.lib.io.motor.MotorIOTalonFXSim;
import frc.lib.mechanisms.linear.*;
import frc.lib.mechanisms.linear.LinearMechanism.LinearMechCharacteristics;
import frc.lib.util.MechanismUtil.DistanceAngleConverter;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;

/** Add your docs here. */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class LinearConstants {
    public static final String NAME = "Linear";

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
    private static final Rotation3d ORIENTATION =
        new Rotation3d(0.0, Degrees.of(-90.0).in(Units.Radians), 0.0);

    private static final LinearMechCharacteristics CHARACTERISTICS =
        new LinearMechCharacteristics(MIN_DISTANCE, MAX_DISTANCE,
            STARTING_DISTANCE, CONVERTER, ORIENTATION);

    // Positional PID
    public static final Slot0Configs SLOT_0_CONFIG = new Slot0Configs()
        .withKP(50.0)
        .withKI(0.0)
        .withKD(0.0);

    public static TalonFXConfiguration getFXConfig()
    {
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

        config.Feedback.RotorToSensorRatio = 1.0;

        config.Feedback.SensorToMechanismRatio = GEARING;

        config.Slot0 = SLOT_0_CONFIG;
        config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY.in(RotationsPerSecond);
        config.MotionMagic.MotionMagicAcceleration = ACCELERATION.in(RotationsPerSecondPerSecond);
        config.MotionMagic.MotionMagicJerk = JERK.in(RotationsPerSecondPerSecond.per(Second));

        return config;
    }

    public static LinearMechanism get()
    {
        switch (Constants.currentMode) {
            case REAL:
                return new LinearMechanismReal(NAME,
                    new MotorIOTalonFX(NAME, getFXConfig(), Ports.linear), CHARACTERISTICS);
            case SIM:
                return new LinearMechanismSim(NAME,
                    new MotorIOTalonFXSim(NAME, getFXConfig(), Ports.linear),
                    DCMOTOR, CARRIAGE_MASS, CHARACTERISTICS, true);
            case REPLAY:
                return new LinearMechanism(NAME, CHARACTERISTICS) {};
            default:
                throw new IllegalStateException("Unrecognized Robot Mode");
        }
    }
}
