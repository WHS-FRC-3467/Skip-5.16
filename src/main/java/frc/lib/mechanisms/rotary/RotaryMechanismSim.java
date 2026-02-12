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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.io.absoluteencoder.AbsoluteEncoderIOSim;
import frc.lib.io.motor.MotorIOSim;
import frc.lib.mechanisms.MechanismConstant;

/**
 * A simulated implementation of the RotaryMechanism base class that uses SingleJointedArmSim to
 * simulate the behavior of a rotary mechanism.
 */
public class RotaryMechanismSim extends RotaryMechanism<MotorIOSim, AbsoluteEncoderIOSim> {
    private final SingleJointedArmSim sim;

    private Time lastTime = Seconds.zero();

    public RotaryMechanismSim(String name, MotorIOSim io, DCMotor dcMotor,
        MomentOfInertia momentOfInertia, Boolean useGravity,
        RotaryMechCharacteristics characteristics,
        Optional<AbsoluteEncoderIOSim> absoluteEncoder, MechanismConstant mech)
    {
        super(name, characteristics, io, absoluteEncoder, mech);

        if (momentOfInertia.isEquivalent(KilogramSquareMeters.zero()))
            throw new IllegalArgumentException(
                "momentOfInertia must be greater than zero!");

        sim = new SingleJointedArmSim(
            dcMotor,
            io.getRotorToSensorRatio() * io.getSensorToMechanismRatio(),
            momentOfInertia.in(KilogramSquareMeters),
            characteristics.armLength().in(Meters),
            characteristics.minAngle().in(Radians),
            characteristics.maxAngle().in(Radians),
            useGravity,
            characteristics.startingAngle().in(Radians));
    }

    @Override
    public void periodic()
    {
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

        absoluteEncoder.ifPresent(encoderSim -> {
            encoderSim
                .setAngle(Radians.of(sim.getAngleRads()).times(io.getSensorToMechanismRatio()));
            encoderSim
                .setAngularVelocity(RadiansPerSecond.of(sim.getVelocityRadPerSec())
                    .times(io.getSensorToMechanismRatio()));
        });

        super.periodic();
    }
}
