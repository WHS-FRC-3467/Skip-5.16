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
package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.mechanisms.linear.LinearMechanism;
import frc.lib.util.LoggedTrigger;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.subsystems.indexer.IndexerConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class IntakeSuperstructure extends SubsystemBase implements AutoCloseable {
    private static final LoggedTunableNumber INTAKE_SETPOINT =
        new LoggedTunableNumber(IntakeRollerConstants.NAME + "/IntakeRPS", 10.0);
    private static final LoggedTunableNumber EJECT_SETPOINT =
        new LoggedTunableNumber(IntakeRollerConstants.NAME + "/EjectRPS", -10.0);
    private static final LoggedTunableNumber INTAKE_CURRENT =
        new LoggedTunableNumber(IntakeLinearConstants.NAME + "/IntakeCurrentTorqueCurrent", 60.0);

    @RequiredArgsConstructor
    @SuppressWarnings("ImmutableEnumChecker")
    @Getter
    public enum State {
        INTAKE(() -> RotationsPerSecond.of(INTAKE_SETPOINT.get())),
        EJECT(() -> RotationsPerSecond.of(EJECT_SETPOINT.get())),
        STOP(() -> RotationsPerSecond.of(0.0));

        private final Supplier<AngularVelocity> angularVelocity;

    }

    @Getter
    private State state = State.STOP;
    public final LoggedTrigger linearStopped;
    public final LoggedTrigger isExtended;
    public final LoggedTrigger isRetracted;

    private final LinearMechanism<?> intakeLinearIO;
    private final FlywheelMechanism<?> intakeRollerIO;

    public IntakeSuperstructure(
        LinearMechanism<?> intakeLinearIO,
        FlywheelMechanism<?> intakeRollerIO) {
        this.intakeLinearIO = intakeLinearIO;
        this.intakeRollerIO = intakeRollerIO;

        this.linearStopped =
            new LoggedTrigger(IntakeLinearConstants.NAME + "/isLinearStopped",
                () -> isLinearStopped());
        this.isExtended = new LoggedTrigger(IntakeLinearConstants.NAME + "/isExtended",
            () -> this.intakeLinearIO.getTorqueCurrent().in(Amps) > INTAKE_CURRENT.get() * 0.8)
            .and(this.linearStopped);
        this.isRetracted = new LoggedTrigger(IntakeLinearConstants.NAME + "/isRetracted",
            () -> this.intakeLinearIO.getTorqueCurrent().in(Amps) < -INTAKE_CURRENT.get() * 0.8)
            .and(this.linearStopped);
    }


    private void setState(State state) {
        this.state = state;
        intakeRollerIO.runVelocity(state.angularVelocity.get(),
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
    public Command setStateCommand(State state) {
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
    public Command holdStateUntilInterrupted(State state) {
        return this.startEnd(() -> setState(state), () -> setState(State.STOP))
            .withName(state.name() + " Until Interrupted");
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
    public Command holdStateUntilInterruptedAndExtend(State state) {
        return this.startEnd(
            () -> {
                setState(state);
                intakeLinearIO.runCurrent(Amps.of(INTAKE_CURRENT.get()));
            },
            () -> setState(State.STOP))
            .withName(state.name() + " Until Interrupted And Extend");
    }

    /**
     * Creates a command to extend the intake.
     *
     * @return A command that extends the intake
     */
    public Command extend() {
        return this.runOnce(() -> intakeLinearIO.runCurrent(Amps.of(INTAKE_CURRENT.get())));
    }

    /**
     * Creates a command to retract the intake.
     *
     * @return A command that retracts the intake
     */
    public Command retract() {
        return this.runOnce(() -> intakeLinearIO.runCurrent(Amps.of(-INTAKE_CURRENT.get())));
    }

    /**
     * Creates a command that repeatedly cycles the intake between extended and retracted positions.
     * Each cycle extends the intake, waits until stopped, retracts, then waits until stopped again.
     *
     * @return A repeating command that cycles the intake
     */
    public Command cycle() {
        return Commands.repeatingSequence(
            this.extend(),
            Commands.deadline(Commands.waitSeconds(1), Commands.waitUntil(linearStopped)),
            this.retract(),
            Commands.deadline(Commands.waitSeconds(1), Commands.waitUntil(linearStopped)));
    }

    /**
     * Creates a command to stop the intake linear mechanism.
     *
     * @return A command that stops the linear motion
     */
    public Command stopIntake() {
        return this.runOnce(() -> intakeLinearIO.runBrake());
    }

    /**
     * Creates a command to stop the intake linear mechanism.
     *
     * @return A command that stops the linear motion
     */
    public Command stopRoller() {
        return this.runOnce(() -> intakeRollerIO.runBrake())
            .andThen(this.runOnce(() -> this.state = State.STOP)).withName("STOP");
    }

    /**
     * Checks if the intake roller velocity is near the specified state's setpoint.
     *
     * @param state The state whose setpoint to check against
     * @return true if the roller is within tolerance of the state's setpoint, false otherwise
     */
    public boolean nearSetpoint(State state) {
        return MathUtil.isNear(
            state.angularVelocity.get().in(RotationsPerSecond),
            intakeRollerIO.getVelocity().in(RotationsPerSecond),
            IntakeRollerConstants.TOLERANCE.in(RotationsPerSecond));
    }

    /**
     * Gets the current velocity of the intake roller motor.
     *
     * @return The current angular velocity
     */
    public AngularVelocity getVelocity() {
        return intakeRollerIO.getVelocity();
    }

    /**
     * Coast intake linear for pit testing.
     *
     * @return a command to set the intake linear mechanism to coast mode.
     */
    public Command coast() {
        return this.runOnce(() -> intakeLinearIO.runCoast());
    }

    private boolean isLinearStopped() {
        return Math.abs(intakeLinearIO.getVelocity().in(RotationsPerSecond)) < 2.0;
    }

    /**
     * Gets the linear extension of the subsystem by converting the motor's rotation
     *
     * @return The estimated linear extension of the subsystem
     */
    public Distance getExtension() {
        return IntakeLinearConstants.CONVERTER.toDistance(intakeLinearIO.getPosition());
    }

    @Override
    public void periodic() {
        Logger.recordOutput(this.getName() + "/State",
            this.isExtended.getAsBoolean() ? "EXTENDED"
                : this.isRetracted.getAsBoolean() ? "RETRACTED" : "MOVING");

        Logger.recordOutput(IntakeRollerConstants.NAME + "/State", state.name());
        intakeLinearIO.periodic();

        intakeRollerIO.periodic();
    }


    @Override
    public void close() {
        intakeRollerIO.close();
        intakeLinearIO.close();
    }
}
