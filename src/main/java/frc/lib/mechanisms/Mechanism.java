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

package frc.lib.mechanisms;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.io.motor.MotorInputsAutoLogged;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.PID;

/**
 * Abstract base class for all robot mechanisms that use motors. Provides common functionality for
 * motor control, PID tuning, logging, and visualization. Mechanisms include flywheels, linear
 * actuators, and rotary arms.
 *
 * @param <T> the type of MotorIO implementation used by this mechanism
 */
public abstract class Mechanism<T extends MotorIO> {

    protected final String name;
    protected final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();
    protected final T io;
    private final List<TunablePidConfig> tunablePidConfigs = new ArrayList<>();

    protected Mechanism(String name, T io) {
        this.name = name;
        this.io = io;
    }

    private static final class TunablePidConfig {
        private final PIDSlot slot;
        private final LoggedTunableNumber kp;
        private final LoggedTunableNumber ki;
        private final LoggedTunableNumber kd;
        private final LoggedTunableNumber ka;
        private final LoggedTunableNumber kv;
        private final LoggedTunableNumber kg;
        private final LoggedTunableNumber ks;
        private final int id;

        private TunablePidConfig(
            PIDSlot slot,
            LoggedTunableNumber kp,
            LoggedTunableNumber ki,
            LoggedTunableNumber kd,
            LoggedTunableNumber ka,
            LoggedTunableNumber kv,
            LoggedTunableNumber kg,
            LoggedTunableNumber ks,
            int id) {
            this.slot = slot;
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
            this.ka = ka;
            this.kv = kv;
            this.kg = kg;
            this.ks = ks;
            this.id = id;
        }
    }

    /**
     * Enables tunable PID for a slot using this mechanism's name as the logging prefix.
     *
     * @param slot The slot to update
     * @param defaultPid The default PID values
     */
    public void enableTunablePID(PIDSlot slot, PID defaultPid) {
        LoggedTunableNumber kp =
            new LoggedTunableNumber(name + "/PID/" + slot + "/kP", defaultPid.P());
        LoggedTunableNumber ki =
            new LoggedTunableNumber(name + "/PID/" + slot + "/kI", defaultPid.I());
        LoggedTunableNumber kd =
            new LoggedTunableNumber(name + "/PID/" + slot + "/kD", defaultPid.D());
        LoggedTunableNumber ka =
            new LoggedTunableNumber(name + "/PID/" + slot + "/kA", defaultPid.A());
        LoggedTunableNumber kv =
            new LoggedTunableNumber(name + "/PID/" + slot + "/kV", defaultPid.V());
        LoggedTunableNumber kg =
            new LoggedTunableNumber(name + "/PID/" + slot + "/kG", defaultPid.G());
        LoggedTunableNumber ks =
            new LoggedTunableNumber(name + "/PID/" + slot + "/kS", defaultPid.S());
        int id = Objects.hash(this, slot);
        tunablePidConfigs.add(new TunablePidConfig(slot, kp, ki, kd, ka, kv, kg, ks, id));
    }

    /** Call this method periodically */
    public void periodic() {
        for (TunablePidConfig config : tunablePidConfigs) {
            LoggedTunableNumber.ifChanged(
                config.id,
                () -> io.setPID(
                    config.slot,
                    new PID(
                        config.kp.get(),
                        config.ki.get(),
                        config.kd.get(),
                        config.ka.get(),
                        config.kv.get(),
                        config.kg.get(),
                        config.ks.get())),
                config.kp,
                config.ki,
                config.kd,
                config.ka,
                config.kv,
                config.kg,
                config.ks);
        }
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    /**
     * Sets the mechanism to coast mode.
     */
    public void runCoast() {
        io.runCoast();
    }

    /**
     * Sets the mechanism to brake mode.
     */
    public void runBrake() {
        io.runBrake();
    }

    /**
     * Runs the mechanism using direct voltage control.
     *
     * @param voltage Desired voltage output.
     */
    public void runVoltage(Voltage voltage) {
        io.runVoltage(voltage);
    }

    /**
     * Runs the mechanism with a specified current output.
     *
     * @param current Desired torque-producing current.
     */
    public void runCurrent(Current current) {
        io.runCurrent(current);
    }

    /**
     * Runs the mechanism using duty cycle (percentage of available voltage).
     *
     * @param dutyCycle Fractional output between 0 and 1.
     */
    public void runDutyCycle(double dutyCycle) {
        io.runDutyCycle(dutyCycle);
    }

    /**
     * Runs the mechanism to a specific position.
     *
     * @param position Target position.
     * @param slot PID slot index.
     */
    public void runPosition(Angle position, PIDSlot slot) {
        io.runPosition(position, slot);

    }


    /**
     * Runs the mechanism at a target velocity.
     *
     * @param velocity Desired velocity.
     * @param acceleration Max acceleration.
     * @param slot PID slot index.
     */
    public void runVelocity(AngularVelocity velocity, AngularAcceleration acceleration,
        PIDSlot slot) {
        io.runVelocity(velocity, acceleration, slot);
    }

    /**
     * Updates one PID slot on the motor
     *
     * @param slot The slot to update
     * @param pid The PID to set
     */
    public void setPID(PIDSlot slot, PID pid) {
        io.setPID(slot, pid);
    }

    /**
     * Sets the position of the motor's internal encoder
     *
     * @param position Desired position to set encoder to
     */
    public void setEncoderPosition(Angle position) {
        io.setEncoderPosition(position);
    }

    /**
     * Gets the supply current draw of the mechanism.
     *
     * @return The supply current
     */
    public Current getSupplyCurrent() {
        return inputs.supplyCurrent;
    }

    /**
     * Getter for angle of the motor
     *
     * @return Angle of the motor or fused encoder
     */
    public Angle getPosition() {
        return inputs.position;
    }

    /**
     * Gets the error between the target position and current position of the motor.
     *
     * @return The position error in angle units
     */
    public Angle getPositionError() {
        return inputs.positionError;
    }

    /**
     * Gets the torque-producing current of the mechanism.
     *
     * @return The torque current
     */
    public Current getTorqueCurrent() {
        return inputs.torqueCurrent;
    }

    /**
     * Gets the velocity of the mechanism.
     *
     * @return The angular velocity
     */
    public AngularVelocity getVelocity() {
        return inputs.velocity;
    }

    /**
     * Gets the error between the target velocity and current velocity of the motor.
     *
     * @return The velocity error in angular velocity units
     */
    public AngularVelocity getVelocityError() {
        return inputs.velocityError;
    }

    /**
     * Closes the mechanism and releases resources.
     */
    public void close() {
        io.close();
    }

    /**
     * Supplier for the Pose3d of the mechanism
     *
     * @return Supplier for the Pose3d
     */
    public Supplier<Pose3d> getPoseSupplier() {
        return () -> Pose3d.kZero;
    }
}
