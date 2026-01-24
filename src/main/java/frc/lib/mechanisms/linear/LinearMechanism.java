// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.mechanisms.linear;

import frc.lib.io.motor.MotorIO;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.io.motor.MotorIO.ControlType;
import frc.lib.io.motor.MotorInputsAutoLogged;
import frc.lib.mechanisms.Mechanism;
import frc.lib.util.MechanismUtil.DistanceAngleConverter;

/**
 * Abstract class for linear mechanisms, which are mechanisms that move in a straight line. This
 * class implements the Mechanism interface and provides characteristics specific to linear
 * mechanisms. Supports mechanisms at any orientation angle, including dynamic angle updates for
 * pivoting mechanisms.
 */
public abstract class LinearMechanism<T extends MotorIO> implements Mechanism {

    /**
     * Characteristics for a linear mechanism.
     *
     * @param minDistance Minimum extension distance
     * @param maxDistance Maximum extension distance
     * @param startingDistance Starting extension distance
     * @param converter Converts between rotational and linear units
     * @param orientation The 3D orientation of the linear mechanism's axis of motion. The mechanism
     *        extends along the positive X-axis in its local frame, then rotated by this
     *        orientation. Uses WPILib's counter-clockwise positive convention around Y-axis:
     *        <ul>
     *        <li>Pitch of 0° = horizontal mechanism extending forward</li>
     *        <li>Pitch of -90° (-π/2 radians) = vertical mechanism extending upward (elevator)</li>
     *        <li>Pitch of 90° (π/2 radians) = vertical mechanism extending downward</li>
     *        </ul>
     */
    public record LinearMechCharacteristics(
        Distance minDistance,
        Distance maxDistance,
        Distance startingDistance,
        DistanceAngleConverter converter,
        Rotation3d orientation) {
    }

    protected final String name;
    protected final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();
    protected final T io;
    protected final DistanceAngleConverter converter;

    private final LinearMechanismVisualizer visualizer;

    public LinearMechanism(String name, LinearMechCharacteristics characteristics, T io)
    {
        this.name = name;
        this.io = io;
        visualizer = new LinearMechanismVisualizer(name, characteristics);
        converter = characteristics.converter();
    }

    private Optional<Distance> getTrajectoryDistance()
    {
        if (inputs.controlType != ControlType.POSITION || inputs.positionError == null) {
            return Optional.empty();
        }

        return Optional.of(converter.toDistance(inputs.activeTrajectoryPosition));
    }

    private Optional<Distance> getGoalDistance()
    {
        if (inputs.controlType != ControlType.POSITION || inputs.positionError == null) {
            return Optional.empty();
        }

        return Optional.of(converter.toDistance(inputs.goalPosition));
    }

    // Checks if mechanism is near a goal position within a specified tolerance
    public boolean nearGoal(Distance goalPosition, Distance tolerance)
    {
        return MathUtil.isNear(
            converter.toDistance(getPosition()).in(BaseUnits.DistanceUnit),
            goalPosition.in(BaseUnits.DistanceUnit),
            tolerance.in(BaseUnits.DistanceUnit));
    }

    @Override
    public void periodic()
    {
        visualizer.setMeasuredDistance(converter.toDistance(inputs.position));
        visualizer.setTrajectoryDistance(getTrajectoryDistance());
        visualizer.setGoalDistance(getGoalDistance());

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
        io.setEncoderPosition(position);
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

    /**
     * Updates the orientation of the linear mechanism. This is useful for pivoting linear
     * mechanisms where the angle changes dynamically.
     *
     * @param orientation The new orientation of the mechanism
     */
    public void setOrientation(Rotation3d orientation)
    {
        visualizer.setOrientation(orientation);
    }

    /**
     * Gets the current orientation of the linear mechanism.
     *
     * @return The current orientation
     */
    public Rotation3d getOrientation()
    {
        return visualizer.getOrientation();
    }

    @Override
    public Supplier<Pose3d> getPoseSupplier()
    {
        return visualizer.getPoseSupplier();
    }
}
