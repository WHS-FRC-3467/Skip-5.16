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

import static edu.wpi.first.units.Units.Amps;
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

public abstract class Mechanism<T extends MotorIO> {

    protected final String name;
    protected final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();
    protected final T io;

    protected Mechanism(String name, T io)
    {
        this.name = name;
        this.io = io;
    }

    /** Call this method periodically */
    public void periodic()
    {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    /**
     * Sets the mechanism to coast mode.
     */
    public void runCoast()
    {
        io.runCoast();
    }

    /**
     * Sets the mechanism to brake mode.
     */
    public void runBrake()
    {
        io.runBrake();
    }

    /**
     * Runs the mechanism using direct voltage control.
     *
     * @param voltage Desired voltage output.
     */
    public void runVoltage(Voltage voltage)
    {
        io.runVoltage(voltage);
    }

    /**
     * Runs the mechanism with a specified current output.
     *
     * @param current Desired torque-producing current.
     */
    public void runCurrent(Current current)
    {
        io.runCurrent(current);
    }

    /**
     * Runs the mechanism using duty cycle (percentage of available voltage).
     *
     * @param dutyCycle Fractional output between 0 and 1.
     */
    public void runDutyCycle(double dutyCycle)
    {
        io.runDutyCycle(dutyCycle);
    }

    /**
     * Runs the mechanism to a specific position.
     *
     * @param position Target position.
     * @param slot PID slot index.
     */
    public void runPosition(Angle position, PIDSlot slot)
    {
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
        PIDSlot slot)
    {
        io.runVelocity(velocity, acceleration, slot);
    }

    /**
     * Sets the position of the motor's internal encoder
     * 
     * @param position Desired position to set encoder to
     */
    public void setEncoderPosition(Angle position)
    {}

    public Current getSupplyCurrent()
    {
        return Amps.of(0.0);
    }

    /**
     * Getter for angle of the motor
     * 
     * @return Angle of the motor or fused encoder
     */
    public Angle getPosition()
    {
        return inputs.position;
    }

    public Current getTorqueCurrent()
    {
        return Amps.of(0.0);
    }

    public AngularVelocity getVelocity()
    {
        return inputs.velocity;
    }

    public void close()
    {
        io.close();
    }

    /**
     * Supplier for the Pose3d of the mechanism
     * 
     * @return Supplier for the Pose3d
     */
    public Supplier<Pose3d> getPoseSupplier()
    {
        return () -> Pose3d.kZero;
    }
}
