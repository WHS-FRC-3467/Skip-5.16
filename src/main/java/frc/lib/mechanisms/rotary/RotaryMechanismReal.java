// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.mechanisms.rotary;

import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.io.absoluteencoder.AbsoluteEncoderIO;
import frc.lib.io.absoluteencoder.AbsoluteEncoderInputsAutoLogged;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorIO.PIDSlot;

/**
 * A real implementation of the RotaryMechanism interface that interacts with a physical motor
 * through a MotorIO interface.
 */
public class RotaryMechanismReal extends RotaryMechanism {
    private final MotorIO io;

    private final AbsoluteEncoderInputsAutoLogged absoluteEncoderInputs =
        new AbsoluteEncoderInputsAutoLogged();
    private final Optional<AbsoluteEncoderIO> absoluteEncoder;

    public RotaryMechanismReal(String name, MotorIO io,
        RotaryMechCharacteristics characteristics, Optional<AbsoluteEncoderIO> absoluteEncoder)
    {
        super(name, characteristics);
        this.io = io;
        this.absoluteEncoder = absoluteEncoder;
    }

    @Override
    public void periodic()
    {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);

        absoluteEncoder.ifPresent(encoder -> {
            encoder.updateInputs(absoluteEncoderInputs);
            Logger.processInputs(encoder.getName(), absoluteEncoderInputs);
        });
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
        absoluteEncoder.ifPresent(AbsoluteEncoderIO::close);
    }
}
