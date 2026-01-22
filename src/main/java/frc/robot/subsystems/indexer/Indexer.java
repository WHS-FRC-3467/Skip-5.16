// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.rotary.RotaryMechanism;
import frc.robot.subsystems.intake.IntakeConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/** Add your docs here. */
public class Indexer extends SubsystemBase {
    private final RotaryMechanism io;

    private State state = State.STOP;

    @RequiredArgsConstructor
    @SuppressWarnings("Immutable")
    @Getter
    public enum State {
        STOP(
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
        Logger.recordOutput("Indexer/State", this.state.name());
        io.periodic();
    }

    private void setState(State state)
    {
        this.state = state;
        io.runVelocity(state.stateVelocity,
            IndexerConstants.MAX_ACCELERATION, PIDSlot.SLOT_0);
    }

    /**
     * Sets the subsystem's state
     * 
     * In a sequence, this command is non-blocking (finishes instantly), but still requires the
     * subsystem (you cannot set the subsystem's state twice in a {@link ParallelCommandGroup}))
     * 
     * @param state The state to hold
     * @return The command sequence
     */
    public Command setStateCommand(State state)
    {
        return this.runOnce(() -> setState(state))
            .withName(state.name());
    }

    /**
     * Holds a state until the command is interrupted. Once the command is interrupted, its state
     * will automatically be set to {@link State#STOP}
     * 
     * In a sequence, this command is blocking and requires this subsystem
     * 
     * @param state The state to hold
     * @return The command sequence
     */
    public Command holdStateUntilInterrupted(State state)
    {
        return this.startEnd(() -> setState(state), () -> setState(State.STOP))
            .withName(state.name() + " Until Interrupted");
    }

    public boolean nearSetpoint()
    {
        return MathUtil.isNear(
            state.stateVelocity.in(RotationsPerSecond),
            io.getVelocity().in(RotationsPerSecond),
            IntakeConstants.TOLERANCE.in(RotationsPerSecond));
    }

    public void close()
    {
        io.close();
    }
}
