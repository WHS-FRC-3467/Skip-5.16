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

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.io.motor.MotorInputsAutoLogged;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.PID;

import lombok.Getter;

import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Abstract base class for all robot mechanisms that use motors. Provides common functionality for
 * motor control, PID tuning, logging, and visualization. Mechanisms include flywheels, linear
 * actuators, and rotary arms.
 *
 * @param <T> the type of MotorIO implementation used by this mechanism
 */
public abstract class Mechanism<T extends MotorIO> {
    @Getter protected final String name;
    protected final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();
    protected final T io;
    private final List<TunablePidConfig> tunablePidConfigs = new ArrayList<>();

    /**
     * Effective radius used for linear-to-angular conversion (meters). {@code null} means linear
     * control is not configured for this mechanism.
     */
    private Double radiusMeters = null;

    protected Mechanism(String name, T io) {
        this.name = name;
        this.io = io;
    }

    /**
     * Configures this mechanism with a linear radius for distance-based control. Once set, the
     * mechanism can use {@link #runLinearPosition}, {@link #runLinearVelocity}, and related getters
     * to work in linear (distance/velocity) units.
     *
     * @param radius Effective radius used for linear-to-angular conversion
     * @return This mechanism, for chaining
     * @throws IllegalArgumentException if radius is zero or negative
     */
    public Mechanism<T> withRadius(Distance radius) {
        double r = radius.in(Meters);
        if (r <= 0.0) {
            throw new IllegalArgumentException("radius must be greater than 0 meters");
        }
        this.radiusMeters = r;
        return this;
    }

    /**
     * Returns {@code true} if this mechanism has been configured with a radius.
     *
     * @return Whether linear control is available
     */
    public boolean hasRadius() {
        return radiusMeters != null;
    }

    public static final class TunablePidConfig {
        public final PIDSlot slot;
        public final LoggedTunableNumber kp;
        public final LoggedTunableNumber ki;
        public final LoggedTunableNumber kd;
        public final LoggedTunableNumber ka;
        public final LoggedTunableNumber kv;
        public final LoggedTunableNumber kg;
        public final LoggedTunableNumber ks;
        public final int id;

        public TunablePidConfig(
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
                    () ->
                            io.setPID(
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

        // If a radius has been configured, also log linear equivalents of position/velocity.
        if (radiusMeters != null) {
            Logger.recordOutput(this.name + "/LinearPosition", getLinearPosition());
            Logger.recordOutput(this.name + "/LinearPositionError", getLinearPositionError());
            Logger.recordOutput(this.name + "/LinearVelocity", getLinearVelocity());
            Logger.recordOutput(this.name + "/LinearVelocityError", getLinearVelocityError());
        }
    }

    /** Sets the mechanism to coast mode. */
    public void runCoast() {
        io.runCoast();
    }

    /** Sets the mechanism to brake mode. */
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
     * Runs the mechanism with a specified current output.
     *
     * @param current Desired torque-producing current.
     * @param dutyCycle Desired dutycycle of current output, limiting top speed
     */
    public void runCurrent(Current current, double dutyCycle) {
        io.runCurrent(current, dutyCycle);
    }

    /**
     * Runs the mechanism using duty cycle (percentage of available voltage).
     *
     * @param dutyCycle Fractional output between 0 and 1.
     */
    public void runDutyCycle(double dutyCycle, boolean ignoringSoftLimits) {
        io.runDutyCycle(dutyCycle, ignoringSoftLimits);
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
     * Runs the mechanism to a specific position without a motion profile.
     *
     * @param position Target position.
     * @param slot PID slot index.
     */
    public void runUnprofiledPosition(Angle position, PIDSlot slot) {
        io.runUnprofiledPosition(position, slot);
    }

    /**
     * Runs the mechanism at a target velocity.
     *
     * @param velocity Desired velocity.
     * @param acceleration Max acceleration.
     * @param slot PID slot index.
     */
    public void runVelocity(
            AngularVelocity velocity, AngularAcceleration acceleration, PIDSlot slot) {
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
     * Gets the applied voltage draw of the mechanism.
     *
     * @return The applied voltage
     */
    public Voltage getAppliedVoltage() {
        return inputs.appliedVoltage;
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
     * Checks that this mechanism was configured with a radius and throws if not.
     *
     * @throws IllegalStateException if {@link #withRadius} was never called
     */
    private void requireRadius() {
        if (radiusMeters == null) {
            throw new IllegalStateException(
                    "Mechanism '"
                            + name
                            + "' has no radius configured. Call withRadius() before using linear"
                            + " control methods.");
        }
    }

    /**
     * Runs the mechanism to a target linear position.
     *
     * @param position Desired linear position
     * @param slot PID slot to use
     * @throws IllegalStateException if {@link #withRadius} was never called
     */
    public void runLinearPosition(Distance position, PIDSlot slot) {
        requireRadius();
        Angle angle = Radians.of(position.in(Meters) / radiusMeters);
        runPosition(angle, slot);
    }

    /**
     * Runs the mechanism to a target linear position without a motion profile.
     *
     * @param position Desired linear position
     * @param slot PID slot to use
     * @throws IllegalStateException if {@link #withRadius} was never called
     */
    public void runUnprofiledLinearPosition(Distance position, PIDSlot slot) {
        requireRadius();
        Angle angle = Radians.of(position.in(Meters) / radiusMeters);
        runUnprofiledPosition(angle, slot);
    }

    /**
     * Runs the mechanism at a target linear velocity with acceleration limiting.
     *
     * @param velocity Desired linear velocity
     * @param acceleration Maximum linear acceleration
     * @param slot PID slot to use
     * @throws IllegalStateException if {@link #withRadius} was never called
     */
    public void runLinearVelocity(
            LinearVelocity velocity, LinearAcceleration acceleration, PIDSlot slot) {
        requireRadius();
        AngularVelocity angularVelocity =
                RadiansPerSecond.of(velocity.in(MetersPerSecond) / radiusMeters);
        AngularAcceleration angularAcceleration =
                RadiansPerSecondPerSecond.of(
                        acceleration.in(MetersPerSecondPerSecond) / radiusMeters);
        runVelocity(angularVelocity, angularAcceleration, slot);
    }

    /**
     * Gets the current linear position of the mechanism.
     *
     * @return Linear position
     * @throws IllegalStateException if {@link #withRadius} was never called
     */
    public Distance getLinearPosition() {
        requireRadius();
        return Meters.of(inputs.position.in(Radians) * radiusMeters);
    }

    /**
     * Gets the error between the target and current linear position.
     *
     * @return Linear position error
     * @throws IllegalStateException if {@link #withRadius} was never called
     */
    public Distance getLinearPositionError() {
        requireRadius();
        return Meters.of(inputs.positionError.in(Radians) * radiusMeters);
    }

    /**
     * Gets the current linear velocity of the mechanism.
     *
     * @return Linear velocity
     * @throws IllegalStateException if {@link #withRadius} was never called
     */
    public LinearVelocity getLinearVelocity() {
        requireRadius();
        return MetersPerSecond.of(inputs.velocity.in(RadiansPerSecond) * radiusMeters);
    }

    /**
     * Gets the error between the target and current linear velocity.
     *
     * @return Linear velocity error
     * @throws IllegalStateException if {@link #withRadius} was never called
     */
    public LinearVelocity getLinearVelocityError() {
        requireRadius();
        return MetersPerSecond.of(inputs.velocityError.in(RadiansPerSecond) * radiusMeters);
    }

    /** Closes the mechanism and releases resources. */
    public void close() {
        io.close();
    }
}
