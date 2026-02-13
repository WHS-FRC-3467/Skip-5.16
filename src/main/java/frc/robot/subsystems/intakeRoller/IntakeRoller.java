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

package frc.robot.subsystems.intakeRoller;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.util.LoggedTunableMeasure;
import frc.robot.subsystems.indexer.IndexerConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/**
 * Subsystem that controls the intake roller mechanism. The roller can intake game pieces, eject
 * them, or stop. Uses a flywheel mechanism for velocity control.
 */
public class IntakeRoller extends SubsystemBase implements AutoCloseable {

    private static final LoggedTunableMeasure<AngularVelocity> INTAKE_SETPOINT =
        new LoggedTunableMeasure<>(
            IntakeRollerConstants.NAME + "/Intake",
            RotationsPerSecond,
            10.0);
    private static final LoggedTunableMeasure<AngularVelocity> EJECT_SETPOINT =
        new LoggedTunableMeasure<>(
            IntakeRollerConstants.NAME + "/Eject", RotationsPerSecond,
            -10.0);

    private final FlywheelMechanism<?> io;

    @Getter
    private State state = State.STOP;

    @RequiredArgsConstructor
    @SuppressWarnings("ImmutableEnumChecker")
    @Getter
    public enum State {
        INTAKE(() -> INTAKE_SETPOINT.get()),
        EJECT(() -> EJECT_SETPOINT.get()),
        STOP(() -> RotationsPerSecond.of(0.0));

        private final Supplier<AngularVelocity> angularVelocity;

    }

    /** Constructor for the Intake subsystem - accepts a FlywheelMechanism. */
    public IntakeRoller(FlywheelMechanism<?> intakeIO)
    {
        this.io = intakeIO;
    }

    private void setState(State state)
    {
        this.state = state;
        io.runVelocity(state.angularVelocity.get(),
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
     * Creates a command to stop the intake roller and set the state to STOP.
     * 
     * @return A command that stops the roller
     */
    public Command stop()
    {
        return this.runOnce(() -> io.runBrake())
            .andThen(this.runOnce(() -> this.state = State.STOP)).withName("STOP");
    }

    /**
     * Checks if the intake roller velocity is near the specified state's setpoint.
     * 
     * @param state The state whose setpoint to check against
     * @return true if the roller is within tolerance of the state's setpoint, false otherwise
     */
    public boolean nearSetpoint(State state)
    {
        return MathUtil.isNear(
            state.angularVelocity.get().in(RotationsPerSecond),
            io.getVelocity().in(RotationsPerSecond),
            IntakeRollerConstants.TOLERANCE.in(RotationsPerSecond));
    }

    /**
     * Gets the current velocity of the intake roller motor.
     * 
     * @return The current angular velocity
     */
    public AngularVelocity getVelocity()
    {
        return io.getVelocity();
    }

    @Override
    public void periodic()
    {
        Logger.recordOutput(IntakeRollerConstants.NAME + "/State", state.name());
        io.periodic();
    }

    @Override
    public void close()
    {
        io.close();
    }
}
