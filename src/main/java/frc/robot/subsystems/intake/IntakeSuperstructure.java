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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.mechanisms.linear.LinearMechanism;
import frc.lib.util.LoggedTrigger;
import frc.lib.util.LoggedTunableMeasure;

public class IntakeSuperstructure extends SubsystemBase implements AutoCloseable {
    private static final LoggedTunableMeasure<AngularVelocity> ROLLER_INTAKE_SETPOINT =
        new LoggedTunableMeasure<>(IntakeRollerConstants.NAME + "/Roller Intake",
            RotationsPerSecond,
            10.0);
    private static final LoggedTunableMeasure<AngularVelocity> ROLLER_EJECT_SETPOINT =
        new LoggedTunableMeasure<>(IntakeRollerConstants.NAME + "/Roller Eject", RotationsPerSecond,
            -10.0);
    private static final LoggedTunableMeasure<Current> LINEAR_CURRENT =
        new LoggedTunableMeasure<>(IntakeLinearConstants.NAME + "/Linear Setpoint", Amps, 60.0);

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
        this.isExtended = new LoggedTrigger(IntakeLinearConstants.NAME + "/isLinearExtended",
            () -> this.intakeLinearIO.getTorqueCurrent().gte(LINEAR_CURRENT.get().times(0.8)))
                .and(this.linearStopped);
        this.isRetracted = new LoggedTrigger(IntakeLinearConstants.NAME + "/isLinearRetracted",
            () -> this.intakeLinearIO.getTorqueCurrent()
                .lte(LINEAR_CURRENT.get().unaryMinus().times(0.8)))
                    .and(this.linearStopped);
    }

    /**
     * Gets the linear extension of the subsystem by converting the motor's rotation.
     *
     * @return The estimated linear extension of the subsystem
     */
    public Distance getExtension() {
        return IntakeLinearConstants.CONVERTER.toDistance(intakeLinearIO.getPosition());
    }

    /**
     * Checks if the linear mechanism has stopped moving.
     *
     * @return true if the linear velocity is less than 2.0 rotations per second, false otherwise
     */
    private boolean isLinearStopped() {
        return Math.abs(intakeLinearIO.getVelocity().in(RotationsPerSecond)) < 2.0;
    }

    public boolean isIntaking() {
        return intakeRollerIO.getVelocity().in(RotationsPerSecond) > 1.0
            && isExtended.getAsBoolean();
    }

    /**
     * Creates a command to run the intake roller at a specified angular velocity.
     *
     * @param angularVelocity A supplier that provides the desired angular velocity for the roller
     * @return A command that sets the roller to the specified velocity
     */
    public Command runRoller(Supplier<AngularVelocity> angularVelocity) {
        return Commands.runOnce(() -> intakeRollerIO.runVelocity(
            angularVelocity.get(),
            IntakeRollerConstants.MAX_ACCELERATION,
            PIDSlot.SLOT_0));
    }

    /**
     * Creates a command to extend the intake linear mechanism by applying positive current.
     *
     * @return A command that extends the linear mechanism
     */
    public Command extendLinear() {
        return this.runOnce(() -> intakeLinearIO.runCurrent(LINEAR_CURRENT.get()));
    }

    /**
     * Creates a command to extend the intake linear mechanism by applying negative current.
     *
     * @return A command that applies negative current to the linear mechanism
     */
    public Command retractLinear() {
        return this.runOnce(() -> intakeLinearIO.runCurrent(LINEAR_CURRENT.get().unaryMinus()));
    }

    /**
     * Creates a command sequence to retract the intake. Starts the roller, applies negative current
     * to retract, waits until fully retracted, then stops the roller.
     *
     * @return A command sequence that retracts the intake
     */
    public Command retractIntake() {
        return Commands.sequence(
            runRoller(() -> ROLLER_INTAKE_SETPOINT.get()),
            retractLinear(),
            Commands.waitUntil(isRetracted),
            stopRoller());
    }

    /**
     * Creates a command sequence to extend the intake. Starts the roller, applies positive current
     * to extend, then waits until extended. - may need verification.
     *
     * @return A command sequence that extends the intake
     */
    public Command extendIntake() {
        return Commands.sequence(
            runRoller(() -> ROLLER_INTAKE_SETPOINT.get()),
            extendLinear(),
            Commands.waitUntil(isExtended));
    }

    /**
     * Creates a command that repeatedly cycles the intake linear mechanism. Alternates between
     * extending and retracting with 3 second timeouts for each operation.
     *
     * @return A repeating command that cycles the linear mechanism
     */
    public Command cycle() {
        return Commands.repeatingSequence(
            extendLinear().withTimeout(3.0),
            retractLinear().withTimeout(3.0));
    }

    /**
     * Creates a command to stop the intake linear mechanism.
     *
     * @return A command that stops the linear motion
     */
    public Command stopLinear() {
        return this.runOnce(() -> intakeLinearIO.runBrake());
    }

    /**
     * Creates a command to stop the intake roller flywheel by applying brake. Use
     * {@link #stopLinear()} to stop the linear mechanism.
     *
     * @return A command that stops the roller motion
     */
    public Command stopRoller() {
        return this.runOnce(() -> intakeRollerIO.runBrake());
    }

    /**
     * Creates a command to coast the intake linear mechanism for pit testing.
     *
     * @return A command that sets the intake linear mechanism to coast mode
     */
    public Command linearCoast() {
        return this.runOnce(() -> intakeLinearIO.runCoast());
    }

    /**
     * Creates a command to run the roller in reverse to eject game pieces.
     *
     * @return A command that runs the roller at negative velocity for ejection
     */
    public Command ejectRoller() {
        return Commands.startEnd(
            () -> runRoller(() -> ROLLER_EJECT_SETPOINT.get()),
            this::stopRoller);
    }

    @Override
    public void periodic() {
        intakeLinearIO.periodic();
        intakeRollerIO.periodic();
    }


    @Override
    public void close() {
        intakeRollerIO.close();
        intakeLinearIO.close();
    }
}
