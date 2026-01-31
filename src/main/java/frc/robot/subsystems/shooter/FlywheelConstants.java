// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.io.motor.MotorIOTalonFX;
import frc.lib.io.motor.MotorIOTalonFX.TalonFXFollower;
import frc.lib.io.motor.MotorIOTalonFXSim;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.mechanisms.flywheel.FlywheelMechanismReal;
import frc.lib.mechanisms.flywheel.FlywheelMechanismSim;
import frc.lib.util.PID;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;

/** Add your docs here. */
public class FlywheelConstants {

    public static String NAME = "Flywheel";

    public static final AngularVelocity MAX_VELOCITY =
        RotationsPerSecond.of(100.0);
    public static final AngularAcceleration MAX_ACCELERATION =
        RotationsPerSecondPerSecond.of(300.0);

    private static final double GEARING = (2.0 / 1.0);

    public static final AngularVelocity TOLERANCE = MAX_VELOCITY.times(0.1);

    private static final DCMotor DCMOTOR = DCMotor.getKrakenX60(1);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.1);

    public static final Distance FLYWHEEL_RADIUS = Units.Meters.of(0.0508); // 2 inches

    // Velocity PID
    public static final PID SLOT0_PID = new PID(1000.0, 0.0, 0.0);
    public static final PID SLOT1_PID = new PID(0.0, 0.0, 0.0);
    public static final PID SLOT2_PID = new PID(0.0, 0.0, 0.0);
    private static Slot0Configs SLOT0CONFIG = new Slot0Configs()
        .withKP(SLOT0_PID.P())
        .withKI(SLOT0_PID.I())
        .withKD(SLOT0_PID.D());

    /**
     * Creates a TalonFX motor controller configuration for the flywheel mechanism.
     * Configures current limits, voltage limits, neutral mode, gearing ratios, and PID gains.
     * 
     * @param invert whether to invert the motor direction
     * @return configured TalonFXConfiguration for the flywheel motor
     */
    public static TalonFXConfiguration getFXConfig(boolean invert)
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
        config.MotorOutput.Inverted =
            invert ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        config.Feedback.RotorToSensorRatio = 1.0;

        config.Feedback.SensorToMechanismRatio = GEARING;

        config.Slot0 = SLOT0CONFIG;
        config.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY.in(RotationsPerSecond);
        config.MotionMagic.MotionMagicAcceleration =
            MAX_ACCELERATION.in(RotationsPerSecondPerSecond);

        return config;
    }


    /**
     * Creates and configures the left flywheel mechanism based on the current robot mode.
     * Selects the appropriate implementation (real, sim, or replay) and enables tunable PID.
     * 
     * @return configured left flywheel mechanism
     */
    public static FlywheelMechanism<?> getLeft()
    {
        FlywheelMechanism<?> mechanism;
        switch (Constants.currentMode) {
            case REAL:
                mechanism = new FlywheelMechanismReal("Left " + NAME,
                    new MotorIOTalonFX("Left " + NAME, getFXConfig(false), Ports.leftFlywheelMain,
                        new TalonFXFollower(Ports.leftFlywheelFollower, false)));
                break;
            case SIM:
                mechanism = new FlywheelMechanismSim("Left " + NAME,
                    new MotorIOTalonFXSim("Left " + NAME, getFXConfig(false),
                        Ports.leftFlywheelMain,
                        new TalonFXFollower(Ports.leftFlywheelFollower, false)),
                    DCMOTOR, MOI, TOLERANCE);
                break;
            case REPLAY:
                mechanism = new FlywheelMechanism<>("Left " + NAME, new MotorIO() {}) {};
                break;
            default:
                throw new IllegalStateException("Unrecognized Robot Mode");
        }
        mechanism.enableTunablePID(PIDSlot.SLOT_0, SLOT0_PID);
        mechanism.enableTunablePID(PIDSlot.SLOT_1, SLOT1_PID);
        mechanism.enableTunablePID(PIDSlot.SLOT_2, SLOT2_PID);
        return mechanism;
    }

    /**
     * Creates and configures the right flywheel mechanism based on the current robot mode.
     * Selects the appropriate implementation (real, sim, or replay) and enables tunable PID.
     * 
     * @return configured right flywheel mechanism
     */
    public static FlywheelMechanism<?> getRight()
    {
        FlywheelMechanism<?> mechanism;
        switch (Constants.currentMode) {
            case REAL:
                mechanism = new FlywheelMechanismReal("Right " + NAME,
                    new MotorIOTalonFX("Right " + NAME, getFXConfig(true), Ports.rightFlywheelMain,
                        new TalonFXFollower(Ports.rightFlywheelFollower, false)));
                break;
            case SIM:
                mechanism = new FlywheelMechanismSim("Right " + NAME,
                    new MotorIOTalonFXSim("Right " + NAME, getFXConfig(true),
                        Ports.rightFlywheelMain,
                        new TalonFXFollower(Ports.rightFlywheelFollower, false)),
                    DCMOTOR, MOI, TOLERANCE);
                break;
            case REPLAY:
                mechanism = new FlywheelMechanism<>("Right " + NAME, new MotorIO() {}) {};
                break;
            default:
                throw new IllegalStateException("Unrecognized Robot Mode");
        }
        mechanism.enableTunablePID(PIDSlot.SLOT_0, SLOT0_PID);
        mechanism.enableTunablePID(PIDSlot.SLOT_1, SLOT1_PID);
        mechanism.enableTunablePID(PIDSlot.SLOT_2, SLOT2_PID);
        return mechanism;
    }
}
