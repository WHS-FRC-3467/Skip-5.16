// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import java.util.Optional;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.io.motor.MotorIOTalonFX;
import frc.lib.io.motor.MotorIOTalonFXSim;
import frc.lib.mechanisms.rotary.*;
import frc.lib.mechanisms.rotary.RotaryMechanism.RotaryAxis;
import frc.lib.mechanisms.rotary.RotaryMechanism.RotaryMechCharacteristics;
import frc.lib.util.PID;
import frc.robot.Constants;
import frc.robot.Ports;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;

/**
 * Defines configuration and physical constants for the turret hood mechanism, including motion
 * constraints, geometry, motor model, and control gains used to construct the
 * {@link RotaryMechanism} instance for different robot modes.
 */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class HoodConstants {
    public static final String NAME = "Hood";

    public static final Angle TOLERANCE = Degrees.of(1.0);

    public static final AngularVelocity CRUISE_VELOCITY =
        RadiansPerSecond.of(10);
    public static final AngularAcceleration ACCELERATION =
        RadiansPerSecondPerSecond.of(100);

    private static final double GEARING = (48.0 / 14.0) * (164.0 / 10.0);

    // Actual arm angle from horizontal is 17 to 44 deg
    public static final Angle MIN_ANGLE_OFFSET = Degrees.of(17.0);

    public static final Angle MIN_ANGLE = Degrees.of(0.0);
    public static final Angle MAX_ANGLE = Degrees.of(27.0);
    public static final Angle STARTING_ANGLE = Degrees.of(0.0);
    public static final Distance ARM_LENGTH = Inches.of(6.9);

    public static final RotaryMechCharacteristics CONSTANTS =
        new RotaryMechCharacteristics(
            ARM_LENGTH,
            MIN_ANGLE,
            MAX_ANGLE,
            STARTING_ANGLE,
            RotaryAxis.PITCH);

    public static final DCMotor DCMOTOR = DCMotor.getKrakenX60(1);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.25);

    // Positional PID
    public static final PID SLOT0_PID = new PID(10.0, 2.0, 8.0)
        .withS(0.07)
        .withV(0.1);

    /**
     * Creates a TalonFX motor controller configuration for the hood mechanism. Configures current
     * limits, voltage limits, neutral mode, soft limits, gearing ratios, feedback sensor source,
     * and motion magic parameters.
     * 
     * @return configured TalonFXConfiguration for the hood motor
     */
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

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE.in(Units.Rotations);

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE.in(Units.Rotations);

        config.Feedback.SensorToMechanismRatio = GEARING;

        config.Slot0 = Slot0Configs.from(SLOT0_PID.toSlotConfigs());
        config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY.in(RotationsPerSecond);
        config.MotionMagic.MotionMagicAcceleration = ACCELERATION.in(RotationsPerSecondPerSecond);

        return config;
    }

    /**
     * Creates and configures the hood mechanism based on the current robot mode. Selects the
     * appropriate implementation (real, sim, or replay) and enables tunable PID.
     * 
     * @return configured hood mechanism
     */
    public static RotaryMechanism<?, ?> get()
    {
        RotaryMechanism<?, ?> mechanism;
        switch (Constants.currentMode) {
            case REAL:
                mechanism = new RotaryMechanismReal(NAME,
                    new MotorIOTalonFX(NAME, getFXConfig(), Ports.hood),
                    CONSTANTS,
                    Optional.empty());
                break;
            case SIM:
                mechanism = new RotaryMechanismSim(NAME,
                    new MotorIOTalonFXSim(NAME, getFXConfig(), Ports.hood),
                    DCMOTOR, MOI, false, CONSTANTS,
                    Optional.empty());
                break;
            case REPLAY:
                mechanism = new RotaryMechanism<>(NAME, CONSTANTS, new MotorIO() {},
                    Optional.empty()) {};
                break;
            default:
                throw new IllegalStateException("Unrecognized Robot Mode");
        }
        mechanism.enableTunablePID(PIDSlot.SLOT_0, SLOT0_PID);
        return mechanism;
    }
}
