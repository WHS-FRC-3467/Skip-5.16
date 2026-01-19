// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Second;
import java.util.Optional;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.lib.io.motor.MotorIOTalonFX;
import frc.lib.io.motor.MotorIOTalonFXSim;
import frc.lib.mechanisms.rotary.RotaryMechanism;
import frc.lib.mechanisms.rotary.RotaryMechanism.RotaryAxis;
import frc.lib.mechanisms.rotary.RotaryMechanism.RotaryMechCharacteristics;
import frc.lib.mechanisms.rotary.RotaryMechanismReal;
import frc.lib.mechanisms.rotary.RotaryMechanismSim;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Foot;
import static edu.wpi.first.units.Units.Rotations;

/** Add your docs here. */
public class IndexerConstants {

    public static String NAME = "Indexer";

    public static final AngularVelocity MAX_VELOCITY =
        Units.RadiansPerSecond.of(2 * Math.PI);
    public static final AngularAcceleration MAX_ACCELERATION = MAX_VELOCITY.per(Second);

    private static final double GEARING = (2.0 / 1.0);

    public static final AngularVelocity TOLERANCE = MAX_VELOCITY.times(0.1);

    private static final DCMotor DCMOTOR = DCMotor.getKrakenX60(1);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(1.0);
    public static final Angle STARTING_ANGLE = Radians.zero();
    public static final Distance ARM_LENGTH = Foot.one();

    public static final RotaryMechCharacteristics CONSTANTS =
        new RotaryMechCharacteristics(
            ARM_LENGTH,
            Rotations.of(Double.NEGATIVE_INFINITY),
            Rotations.of(Double.POSITIVE_INFINITY),
            STARTING_ANGLE,
            RotaryAxis.PITCH);
    // Velocity PID
    private static Slot0Configs SLOT0CONFIG = new Slot0Configs()
        .withKP(1000.0)
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

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        config.Feedback.RotorToSensorRatio = 1.0;

        config.Feedback.SensorToMechanismRatio = GEARING;

        config.Slot0 = SLOT0CONFIG;

        return config;
    }

    public static Indexer get()
    {
        switch (Constants.currentMode) {
            case REAL:
                return new Indexer(new RotaryMechanismReal(NAME,
                    new MotorIOTalonFX(NAME, getFXConfig(), Ports.indexer),
                    CONSTANTS,
                    Optional.empty()));
            case SIM:
                return new Indexer(new RotaryMechanismSim(NAME,
                    new MotorIOTalonFXSim(NAME, getFXConfig(), Ports.indexer),
                    DCMOTOR, MOI, false, CONSTANTS,
                    Optional.empty()));
            case REPLAY:
                return new Indexer(new RotaryMechanism(NAME, CONSTANTS) {});
            default:
                throw new IllegalStateException("Unrecognized Robot Mode");
        }
    }

}
