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
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.io.motor.MotorIOTalonFX;
import frc.lib.io.motor.MotorIOTalonFXSim;
import frc.lib.mechanisms.linear.LinearMechanism;
import frc.lib.mechanisms.linear.LinearMechanism.LinearMechCharacteristics;
import frc.lib.mechanisms.linear.LinearMechanismReal;
import frc.lib.mechanisms.linear.LinearMechanismSim;
import frc.lib.util.MechanismUtil.DistanceAngleConverter;
import frc.lib.util.PID;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class IntakeLinearConstants {

    public static final String NAME = "Intake Linear";

    public static final double GEARING = (42.0 / 12.0);

    public static final Distance MIN_DISTANCE = Inches.of(0.0);
    public static final Distance MAX_DISTANCE = Inches.of(11.375);
    public static final Distance STARTING_DISTANCE = Inches.of(0.0);

    public static final Distance DRUM_RADIUS = Inches.of(0.5);
    public static final Mass CARRIAGE_MASS = Kilograms.of(.1);
    public static final DistanceAngleConverter CONVERTER = new DistanceAngleConverter(DRUM_RADIUS);

    public static final DCMotor DCMOTOR = DCMotor.getKrakenX44Foc(1);

    // Orientation for the linear mechanism.
    // Uses WPILib's counter-clockwise positive convention around Y-axis:
    // A pitch of -90 degrees represents a vertical mechanism extending upward (like an elevator).
    // Pitch of 0 degrees would be horizontal extending forward.
    // Roll and yaw can be used for mechanisms that extend in other directions.
    public static final Rotation3d ORIENTATION =
        new Rotation3d(0.0, Degrees.of(0.0).in(Units.Radians), 0.0);

    public static final LinearMechCharacteristics CHARACTERISTICS =
        new LinearMechCharacteristics(MIN_DISTANCE, MAX_DISTANCE,
            STARTING_DISTANCE, CONVERTER, ORIENTATION);

    public static final PID SLOT0_PID = new PID(80.0, 0.0, 0.0).withV(10.0);


    /**
     * Creates and configures a TalonFX motor controller configuration for the intake linear
     * mechanism.
     * 
     * @return The configured TalonFX configuration
     */
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

        config.Slot0 = Slot0Configs.from(SLOT0_PID.toSlotConfigs());

        return config;
    }

    /**
     * Factory method to create a LinearMechanism instance for the intake linear subsystem. Creates
     * the appropriate mechanism based on the current robot mode (REAL, SIM, or REPLAY).
     * 
     * @return A configured LinearMechanism instance
     */
    public static LinearMechanism<?> getMechanism()
    {
        LinearMechanism<?> mechanism;
        switch (Constants.currentMode) {
            case REAL:
                mechanism = new LinearMechanismReal(NAME,
                    new MotorIOTalonFX(NAME, getFXConfig(), Ports.intakeLinear), CHARACTERISTICS);
                break;
            case SIM:
                mechanism = new LinearMechanismSim(NAME,
                    new MotorIOTalonFXSim(NAME, getFXConfig(), Ports.intakeLinear),
                    DCMOTOR, CARRIAGE_MASS, CHARACTERISTICS, false);
                break;
            case REPLAY:
                mechanism = new LinearMechanism<>(NAME, CHARACTERISTICS, new MotorIO() {}) {};
                break;
            default:
                throw new IllegalStateException("Unrecognized Robot Mode");
        }
        mechanism.enableTunablePID(PIDSlot.SLOT_0, SLOT0_PID);
        return mechanism;
    }

    /**
     * Factory method to create an IntakeLinear subsystem instance.
     * 
     * @return A fully configured IntakeLinear subsystem
     */
    public static IntakeLinear get()
    {
        return new IntakeLinear(getMechanism());
    }
}
