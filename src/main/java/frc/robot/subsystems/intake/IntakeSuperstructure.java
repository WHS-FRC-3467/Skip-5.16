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

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.DistanceControlledMechanism;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.mechanisms.linear.LinearMechanism;
import frc.lib.util.LoggedTrigger;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggerHelper;
import java.util.function.Supplier;

public class IntakeSuperstructure extends SubsystemBase implements AutoCloseable {
    private static final LoggedTunableNumber ROLLER_INTAKE_RPS =
            new LoggedTunableNumber(IntakeRollerConstants.NAME + "/IntakeRPS", 30.0);
    private static final LoggedTunableNumber ROLLER_EJECT_RPS =
            new LoggedTunableNumber(IntakeRollerConstants.NAME + "/EjectRPS", -30.0);

    private final DistanceControlledMechanism<LinearMechanism<?>> intakeLinearIO;
    private final FlywheelMechanism<?> intakeRollerIO;

    private final LoggedTrigger isExtended;
    private final LoggedTrigger isRetracted;

    public IntakeSuperstructure(
            DistanceControlledMechanism<LinearMechanism<?>> intakeLinearIO,
            FlywheelMechanism<?> intakeRollerIO) {
        this.intakeLinearIO = intakeLinearIO;
        this.intakeRollerIO = intakeRollerIO;

        isExtended =
                new LoggedTrigger(
                        "IntakeSuperstructure/IsExtended",
                        () ->
                                MathUtil.isNear(
                                        IntakeLinearConstants.MAX_DISTANCE.in(Meters),
                                        intakeLinearIO.getLinearPosition().in(Meters),
                                        IntakeLinearConstants.TOLERANCE.in(Meters)));
        isRetracted =
                new LoggedTrigger(
                        "IntakeSuperstructure/IsRetracted",
                        () ->
                                MathUtil.isNear(
                                        IntakeLinearConstants.MIN_DISTANCE.in(Meters),
                                        intakeLinearIO.getLinearPosition().in(Meters),
                                        IntakeLinearConstants.TOLERANCE.in(Meters)));
    }

    /**
     * Gets the linear extension of the subsystem by converting the motor's rotation.
     *
     * @return The estimated linear extension of the subsystem
     */
    public Distance getExtension() {
        return intakeLinearIO.getLinearPosition();
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
    private Command runRoller(Supplier<AngularVelocity> angularVelocity) {
        return this.runOnce(
                        () ->
                                intakeRollerIO.runVelocity(
                                        angularVelocity.get(),
                                        IntakeRollerConstants.MAX_ACCELERATION,
                                        PIDSlot.SLOT_0))
                .withName("Run Roller");
    }

    /**
     * Creates a command to extend the intake linear mechanism by applying positive current.
     *
     * @return A command that extends the linear mechanism
     */
    private Command extendLinear() {
        return Commands.sequence(
                        this.runOnce(
                                () ->
                                        intakeLinearIO.runLinearPosition(
                                                IntakeLinearConstants.MAX_DISTANCE,
                                                PIDSlot.SLOT_0)),
                        Commands.waitUntil(isExtended))
                .withName("Extend Linear");
    }

    /**
     * Creates a command to retract the intake linear mechanism by applying negative current.
     *
     * @return A command that retracts the linear mechanism
     */
    private Command retractLinear() {
        return Commands.parallel(
                        Commands.runOnce(
                                () ->
                                        intakeRollerIO.runVelocity(
                                                RotationsPerSecond.of(
                                                        ROLLER_INTAKE_RPS.getAsDouble()),
                                                IntakeRollerConstants.MAX_ACCELERATION,
                                                PIDSlot.SLOT_0)),
                        Commands.sequence(
                                this.runOnce(
                                        () ->
                                                intakeLinearIO.runLinearPosition(
                                                        IntakeLinearConstants.MIN_DISTANCE,
                                                        PIDSlot.SLOT_0))),
                        Commands.waitUntil(isRetracted).withTimeout(2.0))
                .withName("Retract Linear");
    }

    /**
     * Creates a command sequence to retract the intake. Starts the roller, applies negative current
     * to retract, waits until fully retracted, then stops the roller.
     *
     * @return A command sequence that retracts the intake
     */
    public Command retractIntake() {
        return Commands.sequence(
                        runRoller(() -> RotationsPerSecond.of(ROLLER_INTAKE_RPS.get())),
                        retractLinear(),
                        stopRoller())
                .withName("Retract Intake");
    }

    /**
     * Creates a command sequence to extend the intake. Starts the roller, applies positive current
     * to extend, then waits until extended. - may need verification.
     *
     * @return A command sequence that extends the intake
     */
    public Command intake() {
        return Commands.sequence(
                        runRoller(() -> RotationsPerSecond.of(ROLLER_INTAKE_RPS.get())),
                        extendLinear())
                .withName("Intake");
    }

    /**
     * Creates a command sequence to extend the intake without the roller moving.
     *
     * @return A command sequence that extends the intake
     */
    public Command extendIntake() {
        return Commands.sequence(runRoller(() -> RotationsPerSecond.zero()), extendLinear())
                .withName("Extend Intake");
    }

    /**
     * Creates a command that repeatedly cycles the intake linear mechanism. Alternates between
     * extending and retracting with 3 second timeouts for each operation.
     *
     * @return A repeating command that cycles the linear mechanism
     */
    public Command cycle() {
        return Commands.sequence(
                runRoller(() -> RotationsPerSecond.of(ROLLER_INTAKE_RPS.get())),
                Commands.repeatingSequence(
                        Commands.deadline(
                                Commands.waitSeconds(0.2)
                                        .andThen(
                                                Commands.waitUntil(
                                                        () ->
                                                                intakeLinearIO
                                                                        .getLinearVelocity()
                                                                        .lt(
                                                                                InchesPerSecond.of(
                                                                                        1.0)))),
                                this.runOnce(
                                        () ->
                                                intakeLinearIO.runLinearPosition(
                                                        IntakeLinearConstants.MIN_DISTANCE.plus(
                                                                Inches.of(1.0)),
                                                        PIDSlot.SLOT_0))),
                        Commands.sequence(
                                extendLinear(),
                                Commands.waitUntil(
                                        () ->
                                                intakeLinearIO
                                                        .getLinearPositionError()
                                                        .lte(IntakeLinearConstants.TOLERANCE)))));
    }

    /**
     * Creates a command to stop the intake roller flywheel by applying brake.
     *
     * @return A command that stops the roller motion
     */
    public Command stopRoller() {
        return this.runOnce(() -> intakeRollerIO.runBrake()).withName("Stop Roller");
    }

    /**
     * Creates a command to coast the intake linear mechanism for pit testing.
     *
     * @return A command that sets the intake linear mechanism to coast mode
     */
    public Command linearCoast() {
        return this.runOnce(() -> intakeLinearIO.runCoast()).withName("Linear Coast");
    }

    /**
     * Creates a command to run the roller in reverse to eject game pieces.
     *
     * @return A command that runs the roller at negative velocity for ejection
     */
    public Command ejectRoller() {
        return Commands.startEnd(
                        () -> runRoller(() -> RotationsPerSecond.of(-ROLLER_EJECT_RPS.get())),
                        this::stopRoller)
                .withName("Eject Roller");
    }

    public Command homeLinear() {
        return Commands.sequence(
                this.runOnce(
                        () ->
                                intakeLinearIO.runLinearVelocity(
                                        IntakeLinearConstants.CRUISE_VELOCITY.unaryMinus(),
                                        IntakeLinearConstants.MAX_ACCELERATION,
                                        PIDSlot.SLOT_0)),
                Commands.waitUntil(
                        () -> intakeLinearIO.getLinearVelocity().lt(InchesPerSecond.of(0.2))),
                this.runOnce(() -> intakeLinearIO.setEncoderPosition(Rotations.zero())));
    }

    @Override
    public void periodic() {
        LoggerHelper.recordCurrentCommand(this.getName(), this);
        intakeLinearIO.periodic();
        intakeRollerIO.periodic();
    }

    @Override
    public void close() {
        intakeRollerIO.close();
        intakeLinearIO.close();
    }
}
