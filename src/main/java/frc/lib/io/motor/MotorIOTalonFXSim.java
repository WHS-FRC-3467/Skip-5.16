/*
 * Copyright (C) 2025 Windham Windup
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

package frc.lib.io.motor;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.util.Device.CAN;

/**
 * Abstraction for a simulated CTRE TalonFX motor implementing the {@link MotorIOSim} interface.
 * Wraps motor setup, control modes, telemetry polling, and error handling.
 */
public class MotorIOTalonFXSim extends MotorIOTalonFX implements MotorIOSim {

    private double rotorToSensorRatio;
    private double sensorToMechanismRatio;
    private TalonFXSimState simState;

    /**
     * Constructs and initializes a TalonFX motor simulation.
     *
     * @param name The name of the motor(s)
     * @param config Configuration to apply to the motor(s)
     * @param main CAN ID of the main motor
     * @param followerData Configuration data for the follower(s)
     */
    public MotorIOTalonFXSim(String name, TalonFXConfiguration config, CAN main,
        TalonFXFollower... followerData)
    {
        super(name, config, main, followerData);

        rotorToSensorRatio =
            config.Feedback.RotorToSensorRatio;
        sensorToMechanismRatio =
            config.Feedback.SensorToMechanismRatio;
        simState = super.motor.getSimState();
    }

    @Override
    public void setPosition(Angle position)
    {
        simState.setRawRotorPosition(position.times(rotorToSensorRatio * sensorToMechanismRatio));
    }

    @Override
    public void setRotorVelocity(AngularVelocity velocity)
    {
        simState.setRotorVelocity(velocity);
    }

    @Override
    public void setRotorAcceleration(AngularAcceleration acceleration)
    {
        simState.setRotorAcceleration(acceleration);
    }

    @Override
    public double getRotorToSensorRatio()
    {
        return rotorToSensorRatio;
    }

    @Override
    public double getSensorToMechanismRatio()
    {
        return sensorToMechanismRatio;
    }

    @Override
    public void setEncoderPosition(Angle position)
    {
        super.setEncoderPosition(position.times(rotorToSensorRatio * sensorToMechanismRatio));
    }

    @Override
    public void updateInputs(MotorInputs inputs)
    {
        inputs.connected = BaseStatusSignal.refreshAll(
            super.position,
            super.velocity,
            super.supplyVoltage,
            super.supplyCurrent,
            super.torqueCurrent,
            super.temperature,
            super.closedLoopError,
            super.closedLoopReference,
            super.closedLoopReferenceSlope)
            .isOK();

        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        inputs.position = super.position.getValue();
        inputs.velocity = super.velocity.getValue();
        inputs.appliedVoltage = simState.getMotorVoltageMeasure();
        inputs.supplyCurrent = simState.getSupplyCurrentMeasure();
        inputs.torqueCurrent = simState.getTorqueCurrentMeasure();
        inputs.temperature = super.temperature.getValue();

        // Interpret control-loop status signals conditionally based on current mode
        Double closedLoopErrorValue = super.closedLoopError.getValue();
        Double closedLoopTargetValue = super.closedLoopReference.getValue();

        boolean isRunningPositionControl = super.isRunningPositionControl();
        boolean isRunningMotionMagic = super.isRunningMotionMagic();
        boolean isRunningVelocityControl = super.isRunningVelocityControl();

        inputs.positionError = isRunningPositionControl
            ? Rotations.of(closedLoopErrorValue)
            : Rotations.zero();

        inputs.activeTrajectoryPosition =
            isRunningPositionControl && isRunningMotionMagic
                ? Rotations.of(closedLoopTargetValue)
                : Rotations.zero();

        inputs.goalPosition = isRunningPositionControl
            ? goalPosition
            : Rotations.zero();

        if (isRunningVelocityControl) {
            inputs.velocityError = RotationsPerSecond.of(closedLoopErrorValue);
            inputs.activeTrajectoryVelocity = RotationsPerSecond.of(closedLoopTargetValue);
        } else if (isRunningPositionControl && isRunningMotionMagic) {
            var targetVelocity = closedLoopReferenceSlope.getValue();
            inputs.velocityError = RotationsPerSecond.of(
                targetVelocity - inputs.velocity.in(RotationsPerSecond));
            inputs.activeTrajectoryVelocity = RotationsPerSecond.of(targetVelocity);
        } else {
            inputs.velocityError = RotationsPerSecond.zero();
            inputs.activeTrajectoryVelocity = RotationsPerSecond.zero();
        }

        inputs.controlType = getCurrentControlType();
    }
}
