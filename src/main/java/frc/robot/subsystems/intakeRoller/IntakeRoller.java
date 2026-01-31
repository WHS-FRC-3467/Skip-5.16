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
import frc.lib.util.LoggedTunableNumber;
import frc.robot.subsystems.indexer.IndexerConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class IntakeRoller extends SubsystemBase implements AutoCloseable {

    private static final LoggedTunableNumber INTAKE_SETPOINT =
        new LoggedTunableNumber(IntakeRollerConstants.NAME + "/IntakeRPS", 10.0);
    private static final LoggedTunableNumber EJECT_SETPOINT =
        new LoggedTunableNumber(IntakeRollerConstants.NAME + "/EjectRPS", -10.0);

    private final FlywheelMechanism<?> io;
    private State state = State.STOP;

    @RequiredArgsConstructor
    @SuppressWarnings("ImmutableEnumChecker")
    @Getter
    public enum State {
        INTAKE(() -> RotationsPerSecond.of(INTAKE_SETPOINT.get())),
        EJECT(() -> RotationsPerSecond.of(EJECT_SETPOINT.get())),
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

    public Command stop()
    {
        return this.runOnce(() -> io.runBrake())
            .andThen(this.runOnce(() -> this.state = State.STOP)).withName("STOP");
    }

    /* Checks to see if the intake is near the setpoint */
    public boolean nearSetpoint(State state)
    {
        return MathUtil.isNear(
            state.angularVelocity.get().in(RotationsPerSecond),
            io.getVelocity().in(RotationsPerSecond),
            IntakeRollerConstants.TOLERANCE.in(RotationsPerSecond));
    }

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
