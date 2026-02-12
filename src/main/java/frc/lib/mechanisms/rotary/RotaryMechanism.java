// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.mechanisms.rotary;

import frc.lib.io.absoluteencoder.AbsoluteEncoderIO;
import frc.lib.io.absoluteencoder.AbsoluteEncoderInputsAutoLogged;
import frc.lib.io.motor.MotorIO.ControlType;
import frc.lib.mechanisms.Mechanism;
import frc.lib.mechanisms.MechanismConstant;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.io.motor.MotorIO;

/**
 * Abstract base class for rotary mechanisms that pivot around an axis.
 * Examples include arms, hoods, and pivots. Supports position control with optional
 * absolute encoder feedback and 3D visualization.
 * 
 * @param <T> the type of MotorIO implementation used by this mechanism
 * @param <E> the type of AbsoluteEncoderIO implementation (if used)
 */
public abstract class RotaryMechanism<T extends MotorIO, E extends AbsoluteEncoderIO>
    extends Mechanism<T> {

    public enum RotaryAxis {
        PITCH,
        YAW,
        ROLL
    }

    public static record RotaryMechCharacteristics(
        Distance armLength,
        Angle minAngle,
        Angle maxAngle,
        Angle startingAngle,
        RotaryAxis axis) {
    }

    protected final AbsoluteEncoderInputsAutoLogged absoluteEncoderInputs =
        new AbsoluteEncoderInputsAutoLogged();
    protected final Optional<E> absoluteEncoder;
    private final String encoderName;

    private final RotaryVisualizer visualizer;

    public RotaryMechanism(String name, RotaryMechCharacteristics characteristics, T io,
        Optional<E> absoluteEncoder, MechanismConstant mech)
    {
        super(name, io, mech);
        this.absoluteEncoder = absoluteEncoder;
        this.encoderName = encoderName;
        visualizer = new RotaryVisualizer(name, characteristics);
    }

    private Optional<Angle> getTrajectoryAngle()
    {
        if (inputs.controlType != ControlType.POSITION || inputs.positionError == null) {
            return Optional.empty();
        }

        return Optional.of(inputs.activeTrajectoryPosition);
    }

    private Optional<Angle> getGoalAngle()
    {
        if (inputs.controlType != ControlType.POSITION || inputs.positionError == null) {
            return Optional.empty();
        }

        return Optional.of(inputs.goalPosition);
    }

    // Checks if mechanism is near a goal position within a specified tolerance
    public boolean nearGoal(Angle goalAngle, Angle tolerance)
    {
        return MathUtil.isNear(
            getPosition().in(BaseUnits.AngleUnit),
            goalAngle.in(BaseUnits.AngleUnit),
            tolerance.in(BaseUnits.AngleUnit));
    }

    @Override
    public void periodic()
    {
        // First, refresh motor inputs from hardware in the base class.
        super.periodic();

        // Then, use the up-to-date inputs to update the visualizer.
        visualizer.setCurrentAngle(inputs.position);
        visualizer.setTrajectoryAngle(getTrajectoryAngle());
        visualizer.setGoalAngle(getGoalAngle());

        // Finally, update and log absolute encoder inputs if present.
        absoluteEncoder.ifPresent(encoder -> {
            encoder.updateInputs(absoluteEncoderInputs);
            Logger.processInputs(encoderName, absoluteEncoderInputs);
        });
    }

    @Override
    public void close()
    {
        super.close();
        absoluteEncoder.ifPresent(AbsoluteEncoderIO::close);
    }
}
