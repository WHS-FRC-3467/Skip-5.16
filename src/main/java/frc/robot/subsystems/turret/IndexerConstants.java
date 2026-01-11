// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Foot;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import java.util.Optional;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.lib.io.motor.MotorIOTalonFX;
import frc.lib.io.motor.MotorIOTalonFXSim;
import frc.lib.mechanisms.rotary.*;
import frc.lib.mechanisms.rotary.RotaryMechanism.RotaryAxis;
import frc.lib.mechanisms.rotary.RotaryMechanism.RotaryMechCharacteristics;
import frc.robot.Constants;
import frc.robot.Ports;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;

/** Add your docs here. */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class IndexerConstants {
    public static final String NAME = "Indexer";

    public static final Angle TOLERANCE = Degrees.of(1.0);

    public static final AngularVelocity CRUISE_VELOCITY =
        RadiansPerSecond.of(10);
    public static final AngularAcceleration ACCELERATION =
        RadiansPerSecondPerSecond.of(10);

    private static final double ROTOR_TO_SENSOR = (2.0 / 1.0);
    private static final double SENSOR_TO_MECHANISM = 1.0;

    public static final Angle STARTING_ANGLE = Radians.zero();
    public static final Distance ARM_LENGTH = Foot.one();

    public static final RotaryMechCharacteristics CONSTANTS =
        new RotaryMechCharacteristics(
            ARM_LENGTH,
            Rotations.of(Double.NEGATIVE_INFINITY),
            Rotations.of(Double.POSITIVE_INFINITY),
            STARTING_ANGLE,
            RotaryAxis.PITCH);

    public static final DCMotor DCMOTOR = DCMotor.getKrakenX60(1);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.01);

    // Positional PID
    private static final Slot0Configs SLOT_0_CONFIG = new Slot0Configs()
        .withKP(1.0)
        .withKI(0.0)
        .withKD(0.0);

    public static TalonFXConfiguration getFXConfig()
    {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

        config.CurrentLimits.StatorCurrentLimitEnable = false;
        config.CurrentLimits.StatorCurrentLimit = 120.0;

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        config.Feedback.RotorToSensorRatio = ROTOR_TO_SENSOR;
        config.Feedback.SensorToMechanismRatio = SENSOR_TO_MECHANISM;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        config.Slot0 = SLOT_0_CONFIG;
        config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY.in(RotationsPerSecond);
        config.MotionMagic.MotionMagicAcceleration = ACCELERATION.in(RotationsPerSecondPerSecond);

        return config;
    }

    public static RotaryMechanism get()
    {
        switch (Constants.currentMode) {
            case REAL:
                return new RotaryMechanismReal(NAME,
                    new MotorIOTalonFX(NAME, getFXConfig(), Ports.indexer),
                    CONSTANTS,
                    Optional.empty());
            case SIM:
                return new RotaryMechanismSim(NAME,
                    new MotorIOTalonFXSim(NAME, getFXConfig(), Ports.indexer),
                    DCMOTOR, MOI, false, CONSTANTS,
                    Optional.empty());
            case REPLAY:
                return new RotaryMechanism(NAME, CONSTANTS) {};
            default:
                throw new IllegalStateException("Unrecognized Robot Mode");
        }
    }
}
