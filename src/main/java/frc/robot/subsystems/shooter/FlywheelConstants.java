// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

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

    public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(85.9);
    public static final AngularAcceleration MAX_ACCELERATION =
            RotationsPerSecondPerSecond.of(189.5);

    private static final double GEARING = (32.0 / 24.0);

    public static final AngularVelocity TOLERANCE = RotationsPerSecond.of(1.0);

    private static final DCMotor DCMOTOR = DCMotor.getKrakenX60(4);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.001);

    public static final Distance FLYWHEEL_RADIUS = Inches.of(4.0);

    private static PID getPID() {
        if (RobotBase.isReal()) {
            return new PID(8.0, 0.0, 0.0).withS(5.5).withV(0.15);
        } else {
            return new PID(27.0, 1.0, 0.0).withS(8.0);
        }
    }

    // Velocity PID
    public static final PID SLOT0_PID = getPID();

    /**
     * Creates a TalonFX motor controller configuration for the flywheel mechanism. Configures
     * current limits, voltage limits, neutral mode, gearing ratios, and PID gains.
     *
     * @return configured TalonFXConfiguration for the flywheel motor
     */
    public static TalonFXConfiguration getFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        config.Feedback.RotorToSensorRatio = 1.0;

        config.Feedback.SensorToMechanismRatio = GEARING;

        if (Robot.isReal()) {
            config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
            config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
        }

        config.Slot0 = Slot0Configs.from(SLOT0_PID.toSlotConfigs());
        config.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY.in(RotationsPerSecond);
        config.MotionMagic.MotionMagicAcceleration =
                MAX_ACCELERATION.in(RotationsPerSecondPerSecond);

        return config;
    }

    /**
     * Creates and configures the flywheel mechanism based on the current robot mode. Selects the
     * appropriate implementation (real, sim, or replay) and enables tunable PID.
     *
     * @return configured flywheel mechanism
     */
    public static FlywheelMechanism<?> get() {
        FlywheelMechanism<?> mechanism;
        switch (Constants.currentMode) {
            case REAL:
                mechanism =
                        new FlywheelMechanismReal(
                                NAME,
                                new MotorIOTalonFX(
                                        NAME,
                                        getFXConfig(),
                                        Ports.topLeftFlywheel,
                                        new TalonFXFollower(Ports.topRightFlywheel, true),
                                        new TalonFXFollower(Ports.bottomLeftFlywheel, false),
                                        new TalonFXFollower(Ports.bottomRightFlywheel, true)));

                break;
            case SIM:
                mechanism =
                        new FlywheelMechanismSim(
                                NAME,
                                new MotorIOTalonFXSim(
                                        NAME,
                                        getFXConfig(),
                                        Ports.topLeftFlywheel,
                                        new TalonFXFollower(Ports.topRightFlywheel, true),
                                        new TalonFXFollower(Ports.bottomLeftFlywheel, false),
                                        new TalonFXFollower(Ports.bottomRightFlywheel, true)),
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
        mechanism.withRadius(FLYWHEEL_RADIUS);
        return mechanism;
    }
}
