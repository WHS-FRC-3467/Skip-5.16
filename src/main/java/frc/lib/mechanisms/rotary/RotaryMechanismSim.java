// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.mechanisms.rotary;


import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.io.absoluteencoder.AbsoluteEncoderIOSim;
import frc.lib.io.absoluteencoder.AbsoluteEncoderInputsAutoLogged;
import frc.lib.io.motor.MotorIOSim;

/**
 * A simulated implementation of the RotaryMechanism interface that uses SingleJointedArmSim to
 * simulate the behavior of a rotary mechanism.
 */
public class RotaryMechanismSim extends RotaryMechanism {

    private final MotorIOSim io;
    private final SingleJointedArmSim sim;

    private final AbsoluteEncoderInputsAutoLogged absoluteEncoderInputs =
        new AbsoluteEncoderInputsAutoLogged();
    private final Optional<AbsoluteEncoderIOSim> absoluteEncoderSim;
    private Time lastTime = Seconds.zero();

    public RotaryMechanismSim(String name, MotorIOSim io, DCMotor dcMotor,
        MomentOfInertia momentOfInertia, Boolean useGravity,
        RotaryMechCharacteristics characteristics,
        Optional<AbsoluteEncoderIOSim> absoluteEncoderSim)
    {
        super(name, characteristics);

        if (momentOfInertia.isEquivalent(KilogramSquareMeters.zero()))
            throw new IllegalArgumentException(
                "momentOfInertia must be greater than zero!");

        this.io = io;
        sim = new SingleJointedArmSim(
            dcMotor,
            io.getRotorToSensorRatio() * io.getSensorToMechanismRatio(),
            momentOfInertia.in(KilogramSquareMeters),
            characteristics.armLength().in(Meters),
            characteristics.minAngle().in(Radians),
            characteristics.maxAngle().in(Radians),
            useGravity,
            characteristics.startingAngle().in(Radians));

        this.absoluteEncoderSim = absoluteEncoderSim;
    }

    @Override
    public void periodic()
    {
        super.periodic();

        Time currentTime = RobotController.getMeasureTime();
        double deltaTime = currentTime.minus(lastTime).in(Seconds);

        sim.setInputVoltage(inputs.appliedVoltage.in(Volts));
        sim.update(deltaTime);
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

        lastTime = currentTime;

        io.setPosition(Radians.of(sim.getAngleRads()));
        io.setRotorVelocity(RadiansPerSecond.of(sim.getVelocityRadPerSec())
            .times(io.getRotorToSensorRatio() * io.getSensorToMechanismRatio()));

        Logger.recordOutput(name + " Sim Angle", sim.getAngleRads());

        absoluteEncoderSim.ifPresent(encoderSim -> {
            encoderSim
                .setAngle(Radians.of(sim.getAngleRads()).times(io.getSensorToMechanismRatio()));
            encoderSim
                .setAngularVelocity(RadiansPerSecond.of(sim.getVelocityRadPerSec())
                    .times(io.getSensorToMechanismRatio()));

            encoderSim.updateInputs(absoluteEncoderInputs);
            Logger.processInputs(encoderSim.getName(), absoluteEncoderInputs);
        });

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
    public Angle getPosition()
    {
        return inputs.position;
    }

    @Override
    public AngularVelocity getVelocity()
    {
        return inputs.velocity;
    }
}
