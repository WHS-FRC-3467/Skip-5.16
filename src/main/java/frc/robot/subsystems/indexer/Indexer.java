// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.MechanismSubsystem;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/**
 * Subsystem that controls the indexer mechanism for moving game pieces within the robot.
 * The indexer can pull game pieces in, expel them, or stop.
 * Uses a flywheel mechanism for velocity control.
 */
public class Indexer extends MechanismSubsystem {
    private State state = State.STOP;

    @RequiredArgsConstructor
    @SuppressWarnings("Immutable")
    @Getter
    public enum State {
        STOP(
            RotationsPerSecond.zero()),
        PULL(
            IndexerConstants.MAX_VELOCITY),
        EXPEL(
            IndexerConstants.MAX_VELOCITY.times(-1.0));

        private final AngularVelocity stateVelocity;
    }

    /**
     * Constructs an Indexer subsystem.
     * 
     * @param io The flywheel mechanism for controlling the indexer motor
     */
    public Indexer(FlywheelMechanism<?> io)
    {
       super(io);
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

    /**
     * Checks if the indexer velocity is near the current state's setpoint.
     * 
     * @return true if the indexer is within tolerance of the setpoint, false otherwise
     */
    public boolean nearSetpoint()
    {
        return MathUtil.isNear(
            state.stateVelocity.in(RotationsPerSecond),
            io.getVelocity().in(RotationsPerSecond),
            IndexerConstants.TOLERANCE.in(RotationsPerSecond));
    }

    /**
     * Closes the indexer mechanism and releases resources.
     */
    public void close()
    {
        io.close();
    }

    /**
     * Gets the current velocity of the indexer motor.
     * 
     * @return The velocity in rotations per second
     */
    public double getSpeed()
    {
        return io.getVelocity().in(RotationsPerSecond);
    }
}
