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

package frc.robot.subsystems.intakeLinear;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.lib.io.motor.MotorIOTalonFX;
import frc.lib.io.motor.MotorIOTalonFXSim;
import frc.lib.mechanisms.linear.LinearMechanism;
import frc.lib.mechanisms.linear.LinearMechanism.LinearMechCharacteristics;
import frc.lib.mechanisms.linear.LinearMechanismReal;
import frc.lib.mechanisms.linear.LinearMechanismSim;
import frc.lib.util.MechanismUtil.DistanceAngleConverter;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class IntakeLinearConstants {

    public static final String NAME = "Intake Linear";

    private static final double GEARING = (2.0 / 1.0);

    private static final Distance MIN_DISTANCE = Inches.of(0.0);
    private static final Distance MAX_DISTANCE = Inches.of(36.0);
    private static final Distance STARTING_DISTANCE = Inches.of(0.0);

    private static final Distance DRUM_RADIUS = Inches.of(1.0);
    private static final Mass CARRIAGE_MASS = Kilograms.of(.01);
    public static final DistanceAngleConverter CONVERTER = new DistanceAngleConverter(DRUM_RADIUS);

    private static final DCMotor DCMOTOR = DCMotor.getKrakenX60(1);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.01);

    // Orientation for the linear mechanism.
    // Uses WPILib's counter-clockwise positive convention around Y-axis:
    // A pitch of -90 degrees represents a vertical mechanism extending upward (like an elevator).
    // Pitch of 0 degrees would be horizontal extending forward.
    // Roll and yaw can be used for mechanisms that extend in other directions.
    private static final Rotation3d ORIENTATION =
        new Rotation3d(0.0, Degrees.of(0.0).in(Units.Radians), 0.0);

    private static final LinearMechCharacteristics CHARACTERISTICS =
        new LinearMechCharacteristics(MIN_DISTANCE, MAX_DISTANCE,
            STARTING_DISTANCE, CONVERTER, ORIENTATION);

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

    public static LinearMechanism getMechanism()
    {
        switch (Constants.currentMode) {
            case REAL:
                return new LinearMechanismReal(NAME,
                    new MotorIOTalonFX(NAME, getFXConfig(), Ports.intakeLinear), CHARACTERISTICS);
            case SIM:
                return new LinearMechanismSim(NAME,
                    new MotorIOTalonFXSim(NAME, getFXConfig(), Ports.intakeLinear),
                    DCMOTOR, CARRIAGE_MASS, CHARACTERISTICS, false);
            case REPLAY:
                return new LinearMechanism(NAME, CHARACTERISTICS) {};
            default:
                throw new IllegalStateException("Unrecognized Robot Mode");
        }
    }

    public static IntakeLinear get()
    {
        return new IntakeLinear(getMechanism());
    }
}
