// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.rotary.RotaryMechanism;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/** Add your docs here. */
public class Indexer extends SubsystemBase { // Don't extend if contained in superstructure


    private final RotaryMechanism io;

    private String stateName;


    @RequiredArgsConstructor
    @SuppressWarnings("Immutable")
    @Getter
    public enum State {
        NONE(
            Units.RadiansPerSecond.of(0.0)),
        PULL(
            IndexerConstants.MAX_VELOCITY),
        EXPEL(
            IndexerConstants.MAX_VELOCITY.times(-1.0));

        private final AngularVelocity stateVelocity;

    }



    public Indexer(RotaryMechanism io)
    {
        this.io = io;
    }

    @Override
    public void periodic()
    {
        Logger.recordOutput("Indexer/State", this.stateName);
        io.periodic();

    }

    public Command intakeCommand(State state)
    {

        return this.runOnce(() -> io.runVelocity(state.stateVelocity,
            IndexerConstants.MAX_ACCELERATION, PIDSlot.SLOT_0)


        ).andThen(this.runOnce(() -> this.stateName = state.name()))
            .withName("Shoot");
    }



    public Command stop()
    {
        return this.runOnce(() -> io.runCoast()).withName("Stop");
    }

    // For unit testing
    protected Command shootAmps()
    {
        return this.runOnce(() -> io.runCurrent(Amps.of(30))).withName("Shoot Amps");
    }

    public Current getTorqueCurrent()
    {
        return io.getTorqueCurrent();
    }

    public AngularVelocity getVelocity()
    {
        return io.getVelocity();
    }

    public void close()
    {
        io.close();
    }
}
