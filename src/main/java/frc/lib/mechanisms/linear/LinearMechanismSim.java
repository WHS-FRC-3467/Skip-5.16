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

package frc.lib.mechanisms.linear;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.io.motor.MotorIOSim;

/**
 * A simulated implementation of the LinearMechanism interface that uses ElevatorSim to simulate the
 * behavior of a linear mechanism at any orientation. The orientation's pitch component (Y-axis
 * rotation) determines the angle from horizontal, affecting gravity simulation.
 */
public class LinearMechanismSim extends LinearMechanism {

    private final MotorIOSim io;
    private final ElevatorSim sim;

    private Time lastTime = Seconds.zero();

    /**
     * Creates a new LinearMechanismSim.
     *
     * @param name The name of the mechanism
     * @param io The motor IO simulation
     * @param dcMotor The DC motor characteristics
     * @param mass The mass of the carriage
     * @param constraints The mechanism characteristics including orientation
     * @param useGravity Whether to simulate gravity effects (applies when orientation is vertical)
     */
    public LinearMechanismSim(String name, MotorIOSim io, DCMotor dcMotor, Mass mass,
        LinearMechCharacteristics constraints, Boolean useGravity)
    {
        super(name, constraints);

        this.io = io;

        // ElevatorSim is used as the underlying physics simulation.
        // Note: ElevatorSim assumes vertical orientation for gravity simulation.
        // The visualization and 3D pose will correctly reflect any orientation angle,
        // but physics simulation is most accurate when useGravity=true and orientation
        // is vertical (pitch = -90° for upward, 90° for downward), or when useGravity=false for
        // horizontal mechanisms.
        sim = new ElevatorSim(
            dcMotor,
            io.getRotorToSensorRatio() * io.getSensorToMechanismRatio(),
            mass.in(Kilograms),
            constraints.converter().getDrumRadius().in(Meters),
            constraints.minDistance().in(Meters),
            constraints.maxDistance().in(Meters),
            useGravity,
            constraints.startingDistance().in(Meters));
    }

    @Override
    public void periodic()
    {
        super.periodic();

        Time currentTime = RobotController.getMeasureTime();
        double deltaTime = currentTime.minus(lastTime).in(Seconds);

        // Note: ElevatorSim internally simulates gravity for vertical mechanisms (pitch = -90° for
        // upward, 90° for downward), matching the convention in LinearMechanism.java and
        // LinearConstants.java.
        // For non-vertical mechanisms, the gravity simulation is an approximation since
        // ElevatorSim doesn't support dynamic gravity angle changes.
        // The visualization and 3D pose correctly reflect the orientation, but physics
        // simulation uses the orientation set at construction time.
        // For accurate physics at arbitrary angles, consider using a custom LinearSystemSim.
        sim.setInputVoltage(inputs.appliedVoltage.in(Volts));
        sim.update(deltaTime);
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

        lastTime = currentTime;

        io.setPosition(converter.toAngle(Meters.of(sim.getPositionMeters())));

        io.setRotorVelocity(
            converter.toAngle(Meters.of(sim.getVelocityMetersPerSecond())).per(Seconds));

        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    @Override
    public void runCoast()
    {
        io.runCoast();
    }

    @Override
    public void runBrake()
    {
        io.runBrake();
    }

    @Override
    public void runVoltage(Voltage voltage)
    {
        io.runVoltage(voltage);
    }

    @Override
    public void runCurrent(Current current)
    {
        io.runCurrent(current);
    }

    @Override
    public void runDutyCycle(double dutyCycle)
    {
        io.runDutyCycle(dutyCycle);
    }

    @Override
    public void runPosition(Angle position, PIDSlot slot)
    {
        io.runPosition(position, slot);
    }

    @Override
    public void runVelocity(AngularVelocity velocity, AngularAcceleration acceleration,
        PIDSlot slot)
    {
        io.runVelocity(velocity, acceleration, slot);
    }

    @Override
    public void setEncoderPosition(Angle position)
    {
        sim.setState(converter.toDistance(position).in(Meters), 0);
    }

    @Override
    public Current getSupplyCurrent()
    {
        return inputs.supplyCurrent;
    }

    @Override
    public Angle getPosition()
    {
        return inputs.position;
    }

    @Override
    public AngularVelocity getVelocity()
    {
        return inputs.velocity;
    }

    @Override
    public void close()
    {
        io.close();
    }
}
