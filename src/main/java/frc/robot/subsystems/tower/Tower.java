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

package frc.robot.subsystems.tower;

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
import lombok.Getter;
import lombok.RequiredArgsConstructor;


public class Tower extends SubsystemBase {
    private static final LoggedTunableNumber stopRPS =
        new LoggedTunableNumber(TowerConstants.NAME + "/StopRPS", 0.0);
    private static final LoggedTunableNumber idleRPS =
        new LoggedTunableNumber(TowerConstants.NAME + "/IdleRPS", -0.1);
    private static final LoggedTunableNumber shootRPS =
        new LoggedTunableNumber(TowerConstants.NAME + "/IdleRPS",
            TowerConstants.MAX_VELOCITY.in(RotationsPerSecond));

    private final FlywheelMechanism<?> io;
    private State state = State.STOP;

    @RequiredArgsConstructor
    @SuppressWarnings("Immutable")
    @Getter
    public enum State {
        STOP(() -> RotationsPerSecond.of(stopRPS.get())),
        IDLE(
            () -> RotationsPerSecond.of(idleRPS.get())),
        SHOOT(
            () -> RotationsPerSecond.of(shootRPS.get()));

        private final Supplier<AngularVelocity> stateVelocity;
    }

    public Tower(FlywheelMechanism<?> io)
    {
        this.io = io;
    }

    @Override
    public void periodic()
    {
        Logger.recordOutput(TowerConstants.NAME + "/State", this.state.name());
        io.periodic();
    }

    private void setState(State state)
    {
        this.state = state;
        io.runVelocity(state.stateVelocity.get(),
            TowerConstants.MAX_ACCELERATION, PIDSlot.SLOT_0);
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
            state.stateVelocity.get().in(RotationsPerSecond),
            io.getVelocity().in(RotationsPerSecond),
            TowerConstants.TOLERANCE.in(RotationsPerSecond));
    }

    public void close()
    {
        io.close();
    }

    public double getSpeed()
    {
        return io.getVelocity().in(RotationsPerSecond);
    }
}
