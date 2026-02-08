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

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.linear.LinearMechanism;
import frc.lib.util.LoggedTrigger;
import frc.lib.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/**
 * Climber subsystem that controls a linear climbing mechanism.
 *
 * <p>
 * This class wraps a {@link LinearMechanism} to command the climber to various named positions
 * using closed-loop control. It exposes a set of logical {@link State}s ({@code HOME},
 * {@code STOW}, {@code RAISED}, {@code DOWN}) that are converted to mechanism angle and distance
 * via {@link ClimberConstants#CONVERTER}.
 *
 * <p>
 * The subsystem also supports a homing routine, where the climber is driven slowly until a current
 * spike is detected, at which point the encoder is zeroed at the {@code HOME} position. Commands
 * such as {@link #setGoal(State)} and {@link #homeCommand()} are intended to be scheduled by
 * higher-level routines or operator controls.
 */
public class Climber extends SubsystemBase {
    private final LinearMechanism<?> io;
    private LoggedTrigger homedTrigger;

    @Getter
    private LoggedTrigger atGoalTrigger;
    private static final LoggedTunableNumber STOW_STATE =
        new LoggedTunableNumber("Stow Height", 0.0);
    private static final LoggedTunableNumber RAISED_STATE =
        new LoggedTunableNumber("Raised Height", 20.0);
    private static final LoggedTunableNumber DOWN_STATE =
        new LoggedTunableNumber("Down Height", 2.0);

    @RequiredArgsConstructor
    @SuppressWarnings("ImmutableEnumChecker")
    @Getter
    public enum State {
        HOME(Inches.of(0.0)),
        STOW(Inches.of(STOW_STATE.get())),
        RAISED(Inches.of(RAISED_STATE.get())),
        DOWN(Inches.of(DOWN_STATE.get()));

        private final Distance state;

        public Angle getAngle()
        {
            return ClimberConstants.CONVERTER.toAngle(state);
        }
    }

    // Current desired state of the climber
    private State state = State.STOW;

    public Climber(LinearMechanism<?> io)
    {
        this.io = io;
        homedTrigger =
            new LoggedTrigger(ClimberConstants.NAME + "/homed",
                () -> io.getSupplyCurrent().gte(Amps.of(10))).debounce(0.1);
        atGoalTrigger = new LoggedTrigger(ClimberConstants.NAME + "/atGoal", () -> nearGoal());
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
    public Command setGoal(State state)
    {
        this.state = state;
        return this
            .runOnce(() -> io.runPosition(state.getAngle(), PIDSlot.SLOT_0))
            .withName("Go To " + state.toString() + " State");
    }

    /**
     * Returns true if the climber is near the specified goal position, within a tolerance.
     * 
     * @return True if the climber is near the specified goal position, within a tolerance.
     */
    public boolean nearGoal()
    {
        return MathUtil.isNear(
            state.getAngle().in(Rotations),
            io.getPosition().in(Rotations),
            ClimberConstants.CONVERTER.toAngle(ClimberConstants.TOLERANCE).in(Rotations));
    }

    /**
     * Sets the climber's position and waits until it reaches the goal.
     * 
     * @param position The target/goal position.
     * @return a command that sets the climber to a specific position and ends when the climber is
     *         near the goal.
     */
    public Command waitUntilGoalCommand(Distance position)
    {
        return Commands.waitUntil(() -> {
            return nearGoal();
        });
    }

    /**
     * Homes the climber subsystem.
     * 
     * @return a command that runs the homing routine for the climber. Ends then the climber is
     *         homed and zeroed.
     */
    public Command homeCommand()
    {
        return Commands.sequence(
            this.runOnce(() -> io.runVoltage(Volts.of(-2))),
            Commands.waitUntil(homedTrigger),
            this.runOnce(() -> io.setEncoderPosition(State.HOME.getAngle())),
            setGoal(State.STOW))
            .withName("Homing");
    }

    /**
     * Gets the linear extension of the climber by converting the motor's rotation
     * 
     * @return The estimated linear extension of the subsystem
     */
    public Distance getPosition()
    {
        return ClimberConstants.CONVERTER.toDistance(io.getPosition());

    }

    @Override
    public void periodic()
    {
        Logger.recordOutput(ClimberConstants.NAME + "/State", state.name());
        io.periodic();
    }

    /**
     * Closes the climber subsystem, releasing any resources.
     */
    public void close()
    {
        io.close();
    }
}
