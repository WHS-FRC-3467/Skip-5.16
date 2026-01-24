// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Second;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorIOTalonFX;
import frc.lib.io.motor.MotorIOTalonFXSim;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.mechanisms.flywheel.FlywheelMechanismReal;
import frc.lib.mechanisms.flywheel.FlywheelMechanismSim;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;

/** Add your docs here. */
public class IndexerConstants {

    public static String NAME = "Indexer";

    public static final AngularVelocity MAX_VELOCITY =
        Units.RadiansPerSecond.of(2 * Math.PI);
    public static final AngularAcceleration MAX_ACCELERATION = MAX_VELOCITY.per(Second);

    private static final double GEARING = (2.0 / 1.0);

    public static final AngularVelocity TOLERANCE = MAX_VELOCITY.times(0.1);

    private static final DCMotor DCMOTOR = DCMotor.getKrakenX60(1);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.01);

    // Velocity PID
    private static Slot0Configs SLOT0CONFIG = new Slot0Configs()
        .withKP(80.0)
        .withKI(0.0)
        .withKD(0.0)
        .withKV(10.0);

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
                return new Indexer(new FlywheelMechanismReal(NAME,
                    new MotorIOTalonFX(NAME, getFXConfig(), Ports.indexer)));
            case SIM:
                return new Indexer(new FlywheelMechanismSim(NAME,
                    new MotorIOTalonFXSim(NAME, getFXConfig(), Ports.indexer), DCMOTOR, MOI,
                    TOLERANCE));
            case REPLAY:
                return new Indexer(new FlywheelMechanism<>(NAME, new MotorIO() {}) {});
            default:
                throw new IllegalStateException("Unrecognized Robot Mode");
        }
    }

}
