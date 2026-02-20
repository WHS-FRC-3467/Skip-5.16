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

package frc.lib.io.motor;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.util.CANUpdateThread;
import frc.lib.util.Device;
import frc.lib.util.PID;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Abstraction for a CTRE TalonFX motor implementing the {@link MotorIO} interface. Wraps motor
 * setup, control modes, telemetry polling, and error handling.
 */
public class MotorIOTalonFX implements MotorIO {
    private static final Logger LOGGER = Logger.getLogger(MotorIOTalonFX.class.getName());

    public record TalonFXFollower(Device.CAN id, boolean opposesMain) {}

    protected final TalonFX motor;
    protected final TalonFX[] followers;

    // Cached signals for performance and easier access
    protected final StatusSignal<Angle> position;
    protected final StatusSignal<AngularVelocity> velocity;
    protected final StatusSignal<Voltage> supplyVoltage;
    protected final StatusSignal<Current> supplyCurrent;
    protected final StatusSignal<Current> torqueCurrent;
    protected final StatusSignal<Temperature> temperature;
    protected final StatusSignal<Double> closedLoopError;
    protected final StatusSignal<Double> closedLoopReference;
    protected final StatusSignal<Double> closedLoopReferenceSlope;

    // Preconfigured control objects reused for efficiency
    protected final CoastOut coastControl = new CoastOut();
    protected final StaticBrake brakeControl = new StaticBrake();
    protected final VoltageOut voltageControl = new VoltageOut(0).withEnableFOC(true);
    protected final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0);
    protected final DutyCycleOut dutyCycleControl = new DutyCycleOut(0).withEnableFOC(true);
    protected final MotionMagicTorqueCurrentFOC positionControl =
            new MotionMagicTorqueCurrentFOC(0);
    protected final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0);

    private final CANUpdateThread updateThread = new CANUpdateThread();
    private final Alert[] followerOnWrongBusAlert;
    private final Alert[] disconnectAlerts;
    private final Debouncer[] disconnectDebouncers;

    private volatile TalonFXConfiguration currentConfig;
    protected volatile Angle goalPosition = Rotations.of(0.0);

    /**
     * Constructs and initializes a TalonFX motor.
     *
     * @param name The name of the motor(s)
     * @param config Configuration to apply to the motor(s)
     * @param main CAN ID of the main motor
     * @param followerData Configuration data for the follower(s)
     */
    public MotorIOTalonFX(
            String name,
            TalonFXConfiguration config,
            Device.CAN main,
            TalonFXFollower... followerData) {
        currentConfig = config;

        motor = new TalonFX(main.id(), new CANBus(main.bus()));
        updateThread
                .CTRECheckErrorAndRetry(() -> motor.getConfigurator().apply(config))
                .exceptionally(
                        ex -> {
                            LOGGER.log(Level.SEVERE, ex.toString(), ex);
                            return null;
                        });

        // Initialize lists
        disconnectAlerts = new Alert[followerData.length + 1];
        disconnectAlerts[0] = new Alert(name + " is disconnected!", AlertType.kError);

        disconnectDebouncers = new Debouncer[followerData.length + 1];
        disconnectDebouncers[0] = new Debouncer(0.5);

        followerOnWrongBusAlert = new Alert[followerData.length];
        followers = new TalonFX[followerData.length];

        for (int i = 0; i < followerData.length; i++) {
            disconnectAlerts[i + 1] =
                    new Alert(name + " follower " + i + " is disconnected!", AlertType.kError);
            disconnectDebouncers[i + 1] = new Debouncer(0.5);

            Device.CAN id = followerData[i].id();

            if (!id.bus().equals(main.bus())) {
                followerOnWrongBusAlert[i] =
                        new Alert(
                                name + " follower " + i + " is on a different CAN bus than main!",
                                AlertType.kError);
                followerOnWrongBusAlert[i].set(true);
            }

            followers[i] = new TalonFX(id.id(), new CANBus(id.bus()));

            TalonFX follower = followers[i];
            updateThread
                    .CTRECheckErrorAndRetry(() -> follower.getConfigurator().apply(config))
                    .exceptionally(
                            ex -> {
                                LOGGER.log(Level.SEVERE, ex.toString(), ex);
                                return null;
                            });
            follower.setControl(
                    new Follower(
                            main.id(),
                            followerData[i].opposesMain()
                                    ? MotorAlignmentValue.Opposed
                                    : MotorAlignmentValue.Aligned));
        }

        position = motor.getPosition();
        velocity = motor.getVelocity();
        supplyVoltage = motor.getSupplyVoltage();
        supplyCurrent = motor.getSupplyCurrent();
        torqueCurrent = motor.getTorqueCurrent();
        temperature = motor.getDeviceTemp();
        closedLoopError = motor.getClosedLoopError();
        closedLoopReference = motor.getClosedLoopReference();
        closedLoopReferenceSlope = motor.getClosedLoopReferenceSlope();

        updateThread
                .CTRECheckErrorAndRetry(
                        () ->
                                BaseStatusSignal.setUpdateFrequencyForAll(
                                        main.bus().equals("rio") ? 50 : 100,
                                        position,
                                        velocity,
                                        supplyVoltage,
                                        supplyCurrent,
                                        torqueCurrent,
                                        temperature,
                                        closedLoopError,
                                        closedLoopReference,
                                        closedLoopReferenceSlope))
                .exceptionally(
                        ex -> {
                            LOGGER.log(Level.SEVERE, ex.toString(), ex);
                            return null;
                        });

        motor.optimizeBusUtilization(0, 1.0);
    }

    /**
     * Checks if the motor is currently running a position control mode.
     *
     * @return True if the motor is using a position control mode.
     */
    protected boolean isRunningPositionControl() {
        var control = motor.getAppliedControl();
        return (control instanceof PositionTorqueCurrentFOC)
                || (control instanceof PositionVoltage)
                || (control instanceof MotionMagicTorqueCurrentFOC)
                || (control instanceof DynamicMotionMagicTorqueCurrentFOC)
                || (control instanceof MotionMagicVoltage);
    }

    /**
     * Checks if the motor is currently running a velocity control mode.
     *
     * @return True if the motor is using a velocity control mode.
     */
    protected boolean isRunningVelocityControl() {
        var control = motor.getAppliedControl();
        return (control instanceof VelocityTorqueCurrentFOC)
                || (control instanceof VelocityVoltage)
                || (control instanceof MotionMagicVelocityTorqueCurrentFOC)
                || (control instanceof MotionMagicVelocityVoltage);
    }

    /**
     * Checks if the motor is running any Motion Magic mode.
     *
     * @return True if the motor is using a Motion Magic mode.
     */
    protected boolean isRunningMotionMagic() {
        var control = motor.getAppliedControl();
        return (control instanceof MotionMagicTorqueCurrentFOC)
                || (control instanceof DynamicMotionMagicTorqueCurrentFOC)
                || (control instanceof MotionMagicVelocityTorqueCurrentFOC)
                || (control instanceof MotionMagicVoltage)
                || (control instanceof MotionMagicVelocityVoltage);
    }

    /**
     * Returns the current control type.
     *
     * @return The current control type.
     */
    protected ControlType getCurrentControlType() {
        var control = motor.getAppliedControl();

        if (control instanceof StaticBrake) {
            return ControlType.BRAKE;
        } else if (control instanceof VoltageOut) {
            return ControlType.VOLTAGE;
        } else if (control instanceof TorqueCurrentFOC) {
            return ControlType.CURRENT;
        } else if (control instanceof DutyCycleOut) {
            return ControlType.DUTYCYCLE;
        } else if (isRunningPositionControl()) {
            return ControlType.POSITION;
        } else if (isRunningVelocityControl()) {
            return ControlType.VELOCITY;
        }

        return ControlType.COAST;
    }

    /**
     * Updates the passed-in MotorInputs structure with the latest sensor readings.
     *
     * @param inputs Motor input structure to populate.
     */
    @Override
    public void updateInputs(MotorInputs inputs) {
        inputs.connected =
                BaseStatusSignal.refreshAll(
                                position,
                                velocity,
                                supplyVoltage,
                                supplyCurrent,
                                torqueCurrent,
                                temperature,
                                closedLoopError,
                                closedLoopReference,
                                closedLoopReferenceSlope)
                        .isOK();

        disconnectAlerts[0].set(disconnectDebouncers[0].calculate(!inputs.connected));
        for (int i = 0; i < followers.length; i++) {
            disconnectAlerts[i + 1].set(
                    disconnectDebouncers[i + 1].calculate(!followers[i].isConnected()));
        }

        inputs.position = position.getValue();
        inputs.velocity = velocity.getValue();
        inputs.appliedVoltage = supplyVoltage.getValue();
        inputs.supplyCurrent = supplyCurrent.getValue();
        inputs.torqueCurrent = torqueCurrent.getValue();
        inputs.temperature = temperature.getValue();

        // Interpret control-loop status signals conditionally based on current mode
        Double closedLoopErrorValue = closedLoopError.getValue();
        Double closedLoopTargetValue = closedLoopReference.getValue();

        boolean isRunningPositionControl = isRunningPositionControl();
        boolean isRunningMotionMagic = isRunningMotionMagic();
        boolean isRunningVelocityControl = isRunningVelocityControl();

        inputs.positionError =
                isRunningPositionControl ? Rotations.of(closedLoopErrorValue) : Rotations.zero();

        inputs.activeTrajectoryPosition =
                isRunningPositionControl && isRunningMotionMagic
                        ? Rotations.of(closedLoopTargetValue)
                        : Rotations.zero();

        inputs.goalPosition = isRunningPositionControl ? goalPosition : Rotations.zero();

        if (isRunningVelocityControl) {
            inputs.velocityError = RotationsPerSecond.of(closedLoopErrorValue);
            inputs.activeTrajectoryVelocity = RotationsPerSecond.of(closedLoopTargetValue);
        } else if (isRunningPositionControl && isRunningMotionMagic) {
            var targetVelocity = closedLoopReferenceSlope.getValue();
            inputs.velocityError =
                    RotationsPerSecond.of(targetVelocity - inputs.velocity.in(RotationsPerSecond));
            inputs.activeTrajectoryVelocity = RotationsPerSecond.of(targetVelocity);
        } else {
            inputs.velocityError = RotationsPerSecond.zero();
            inputs.activeTrajectoryVelocity = RotationsPerSecond.zero();
        }

        inputs.controlType = getCurrentControlType();
    }

    /** Sets the motor to coast mode. */
    @Override
    public void runCoast() {
        motor.setControl(coastControl);
    }

    /** Sets the motor to brake mode. */
    @Override
    public void runBrake() {
        motor.setControl(brakeControl);
    }

    /**
     * Runs the motor using direct voltage control.
     *
     * @param voltage Desired voltage output.
     */
    @Override
    public void runVoltage(Voltage voltage) {
        motor.setControl(voltageControl.withOutput(voltage));
    }

    /**
     * Runs the motor with a specified current output.
     *
     * @param current Desired torque-producing current.
     */
    @Override
    public void runCurrent(Current current) {
        motor.setControl(currentControl.withOutput(current).withMaxAbsDutyCycle(1.0));
    }

    /**
     * Runs the motor with a specified current output and duty cycle.
     *
     * @param current Desired torque-producing current.
     * @param dutyCycle Desired dutycycle of current output, limiting top speed
     */
    @Override
    public void runCurrent(Current current, double dutyCycle) {
        double dutyCyclePercent = MathUtil.clamp(dutyCycle, 0.0, 1.0);
        motor.setControl(currentControl.withOutput(current).withMaxAbsDutyCycle(dutyCyclePercent));
    }

    /**
     * Runs the motor using duty cycle (percentage of available voltage).
     *
     * @param dutyCycle Fractional output between -1 and 1.
     */
    @Override
    public void runDutyCycle(double dutyCycle) {
        double dutyCyclePercent = MathUtil.clamp(dutyCycle, -1.0, 1.0);
        motor.setControl(dutyCycleControl.withOutput(dutyCyclePercent));
    }

    /**
     * Runs the motor to a specific position.
     *
     * @param position Target position.
     * @param slot PID slot index.
     */
    @Override
    public void runPosition(Angle position, PIDSlot slot) {
        this.goalPosition = position;
        motor.setControl(positionControl.withPosition(position).withSlot(slot.getNum()));
    }

    /**
     * Runs the motor at a target velocity.
     *
     * @param velocity Desired velocity.
     * @param acceleration Max acceleration.
     * @param slot PID slot index.
     */
    @Override
    public void runVelocity(
            AngularVelocity velocity, AngularAcceleration acceleration, PIDSlot slot) {
        motor.setControl(
                velocityControl
                        .withVelocity(velocity)
                        .withAcceleration(acceleration)
                        .withSlot(slot.getNum()));
    }

    @Override
    public void setEncoderPosition(Angle position) {
        motor.setPosition(position);
    }

    private void setPIDSlot0(PID pid) {
        currentConfig
                .Slot0
                .withKP(pid.P())
                .withKI(pid.I())
                .withKD(pid.D())
                .withKA(pid.A())
                .withKV(pid.V())
                .withKG(pid.G())
                .withKS(pid.S());

        updateThread
                .CTRECheckErrorAndRetry(() -> motor.getConfigurator().apply(currentConfig))
                .exceptionally(
                        ex -> {
                            LOGGER.log(Level.SEVERE, ex.toString(), ex);
                            return null;
                        });
    }

    private void setPIDSlot1(PID pid) {
        currentConfig
                .Slot1
                .withKP(pid.P())
                .withKI(pid.I())
                .withKD(pid.D())
                .withKA(pid.A())
                .withKV(pid.V())
                .withKG(pid.G())
                .withKS(pid.S());

        updateThread
                .CTRECheckErrorAndRetry(() -> motor.getConfigurator().apply(currentConfig))
                .exceptionally(
                        ex -> {
                            LOGGER.log(Level.SEVERE, ex.toString(), ex);
                            return null;
                        });
    }

    private void setPIDSlot2(PID pid) {
        currentConfig
                .Slot2
                .withKP(pid.P())
                .withKI(pid.I())
                .withKD(pid.D())
                .withKA(pid.A())
                .withKV(pid.V())
                .withKG(pid.G())
                .withKS(pid.S());

        updateThread
                .CTRECheckErrorAndRetry(() -> motor.getConfigurator().apply(currentConfig))
                .exceptionally(
                        ex -> {
                            LOGGER.log(Level.SEVERE, ex.toString(), ex);
                            return null;
                        });
    }

    @Override
    public void setPID(PIDSlot slot, PID pid) {
        switch (slot) {
            case SLOT_0 -> setPIDSlot0(pid);
            case SLOT_1 -> setPIDSlot1(pid);
            case SLOT_2 -> setPIDSlot2(pid);
        }
    }

    @Override
    public void close() {
        motor.close();
        for (TalonFX follower : followers) {
            follower.close();
        }

        updateThread.close();
    }
}
