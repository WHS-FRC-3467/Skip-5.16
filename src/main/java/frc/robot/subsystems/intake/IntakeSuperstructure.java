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

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.*;

import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.mechanisms.linear.LinearMechanism;
import frc.lib.util.LoggedTrigger;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggerHelper;

import java.util.function.Supplier;

public class IntakeSuperstructure extends SubsystemBase implements AutoCloseable {

    private static final LoggedTunableNumber ROLLER_INTAKE_RPS =
            new LoggedTunableNumber(IntakeRollerConstants.NAME + "/IntakeRPS", 35.0);

    private static final LoggedTunableNumber ROLLER_EJECT_RPS =
            new LoggedTunableNumber(IntakeRollerConstants.NAME + "/EjectRPS", -35.0);

    private static final LoggedTunableNumber SLOW_MPS =
            new LoggedTunableNumber(IntakeLinearConstants.NAME + "/SlowMPS", 0.25);

    private final LinearMechanism<?> intakeLinearIO;
    private final FlywheelMechanism<?> intakeRollerIO;

    private final LoggedTrigger isExtended;
    private final LoggedTrigger isRetracted;
    private final LoggedTrigger isCycleComplete;

    private final LinearVelocity shuffleVelocity = MetersPerSecond.of(0.8);
    private final Distance retractDistance = IntakeLinearConstants.MIN_DISTANCE;
    private final Distance cycleCompleteTolerance =
            IntakeLinearConstants.MIN_DISTANCE.plus(Inches.of(1.5));

    public IntakeSuperstructure(
            LinearMechanism<?> intakeLinearIO, FlywheelMechanism<?> intakeRollerIO) {

        this.intakeLinearIO = intakeLinearIO;
        this.intakeRollerIO = intakeRollerIO;

        intakeLinearIO.runLinearPosition(
                IntakeLinearConstants.MIN_DISTANCE,
                PIDSlot.SLOT_0,
                IntakeLinearConstants.CRUISE_VELOCITY,
                IntakeLinearConstants.MAX_ACCELERATION);

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
                                        retractDistance.in(Meters),
                                        intakeLinearIO.getLinearPosition().in(Meters),
                                        IntakeLinearConstants.TOLERANCE.in(Meters)));
        isCycleComplete =
                new LoggedTrigger(
                        "IntakeSuperstructure/IsCycleComplete",
                        () ->
                                MathUtil.isNear(
                                        cycleCompleteTolerance.in(Meters),
                                        intakeLinearIO.getLinearPosition().in(Meters),
                                        Inches.of(2.0).in(Meters)));
    }

    /** Returns true if the intake roller is running and the intake is extended. */
    public boolean isIntaking() {
        return intakeRollerIO.getVelocity().in(RotationsPerSecond) > 1.0
                && isExtended.getAsBoolean();
    }

    /**
     * Moves the intake to a goal distance using Motion Magic with the given cruise velocity and
     * acceleration. The command completes immediately after issuing the control request. Use {@link
     * Commands#waitUntil} to block until the goal is reached.
     */
    private Command moveToPosition(
            Distance goal,
            LinearVelocity cruiseVelocity,
            LinearAcceleration acceleration,
            String name) {
        Distance clampedGoal = clampDistance(goal);
        return this.runOnce(
                        () ->
                                intakeLinearIO.runLinearPosition(
                                        clampedGoal, PIDSlot.SLOT_0, cruiseVelocity, acceleration))
                .withName(name);
    }

    private Distance clampDistance(Distance goal) {
        return Meters.of(
                MathUtil.clamp(
                        goal.in(Meters),
                        IntakeLinearConstants.MIN_DISTANCE.in(Meters),
                        IntakeLinearConstants.MAX_DISTANCE.in(Meters)));
    }

    /**
     * Moves the intake by a delta using Motion Magic with the given cruise velocity and
     * acceleration, optionally running the roller at a scaled speed, and waits until within
     * toleranceMeters of the goal.
     */
    private Command moveByDistance(
            Distance distance,
            LinearVelocity cruiseVelocity,
            LinearAcceleration acceleration,
            boolean runRoller,
            double rollerScale,
            double toleranceMeters,
            String name) {
        return moveToDistance(
                () ->
                        Meters.of(
                                intakeLinearIO.getLinearPosition().in(Meters)
                                        + distance.in(Meters)),
                cruiseVelocity,
                acceleration,
                runRoller,
                rollerScale,
                toleranceMeters,
                name);
    }

    private Command moveToDistance(
            Supplier<Distance> goalSupplier,
            LinearVelocity cruiseVelocity,
            LinearAcceleration acceleration,
            boolean runRoller,
            double rollerScale,
            double toleranceMeters,
            String name) {
        return Commands.sequence(
                        this.runOnce(
                                () ->
                                        intakeLinearIO.runLinearPosition(
                                                clampDistance(goalSupplier.get()),
                                                PIDSlot.SLOT_0,
                                                cruiseVelocity,
                                                acceleration)),
                        runRoller ? runRoller(1.0) : Commands.none(),
                        Commands.waitUntil(
                                () ->
                                        MathUtil.isNear(
                                                intakeLinearIO.getGoalLinearPosition().in(Meters),
                                                intakeLinearIO.getLinearPosition().in(Meters),
                                                toleranceMeters)))
                .withName(name);
    }

    private Command runRoller(double dutyCycle) {
        return this.runOnce(() -> intakeRollerIO.runDutyCycle(dutyCycle, false))
                .withName("Run Roller");
    }

    public Command stopRoller() {
        return this.runOnce(intakeRollerIO::runBrake).withName("Stop Roller");
    }

    public Command ejectRoller() {
        return this.startEnd(
                        () ->
                                intakeRollerIO.runVelocity(
                                        RotationsPerSecond.of(ROLLER_EJECT_RPS.get()),
                                        PIDSlot.SLOT_0),
                        intakeRollerIO::runBrake)
                .withName("Eject Roller");
    }

    public Command intake() {
        return extendWithRoller(() -> RotationsPerSecond.of(ROLLER_INTAKE_RPS.get()), "Intake");
    }

    public Command extendIntake() {
        return extendWithRoller(RotationsPerSecond::zero, "Extend Intake");
    }

    private Command extendWithRoller(Supplier<AngularVelocity> rollerVelocity, String name) {
        return Commands.sequence(
                        runRoller(1.0),
                        moveToPosition(
                                IntakeLinearConstants.MAX_DISTANCE,
                                IntakeLinearConstants.CRUISE_VELOCITY,
                                IntakeLinearConstants.MAX_ACCELERATION,
                                "Extend Linear"))
                .withName(name);
    }

    public Command retractIntake() {
        return retractWithSpeed(IntakeLinearConstants.CRUISE_VELOCITY, 1.0, "Retract Intake");
    }

    public Command slowRetract(LinearVelocity retractSpeed) {
        return retractWithSpeed(retractSpeed, 0.6, "Slow Retract");
    }

    public Command slowRetract() {
        return slowRetract(MetersPerSecond.of(SLOW_MPS.get()));
    }

    private Command retractWithSpeed(LinearVelocity retractSpeed, double rollerScale, String name) {
        return Commands.sequence(
                        runRoller(1.0),
                        moveToPosition(
                                retractDistance,
                                retractSpeed,
                                IntakeLinearConstants.MAX_ACCELERATION,
                                "Retract Linear"),
                        Commands.waitUntil(isRetracted),
                        stopRoller())
                .withName(name);
    }

    /**
     * Creates an experimental command to shuffle the intake. If the intake is already retracted,
     * extend it. Then, retract 6 inches, and then extend 3 inches. If the intake is at the end of
     * the shuffle cycle when this command is called, extend intake before shuffling. Each cycle
     * increases the speed.
     *
     * @return a Command that is a "step" in the intake shuffle.
     */
    public Command shuffleStep() {
        return Commands.sequence(
                        // Extend intake if there is no more space to retract
                        Commands.either(
                                Commands.sequence(extendIntake(), Commands.waitUntil(isExtended)),
                                Commands.none(),
                                isCycleComplete),
                        moveByDistance(
                                        Inches.of(-6),
                                        shuffleVelocity,
                                        IntakeLinearConstants.MAX_ACCELERATION,
                                        true,
                                        0.6,
                                        Inches.of(0.5).in(Meters),
                                        "Shuffle Retract")
                                .withTimeout(0.5),
                        moveByDistance(
                                        Inches.of(3.0),
                                        shuffleVelocity,
                                        IntakeLinearConstants.MAX_ACCELERATION,
                                        false,
                                        0.0,
                                        Inches.of(0.5).in(Meters),
                                        "Shuffle Extend")
                                .withTimeout(0.5),
                        stopRoller())
                .withName("Intake Shuffle Step");
    }

    /**
     * Create a command to shuffle the intake between its retracted position and a partially
     * extended position aligned with the front buumper edge.
     *
     * @return a Command that is a step in this intake shuffle for up-against-hub shooting.
     */
    public Command hubShuffleStep() {
        return Commands.sequence(
                        // Ensure intake is retracted
                        moveToDistance(
                                        () -> IntakeLinearConstants.MIN_DISTANCE,
                                        shuffleVelocity,
                                        IntakeLinearConstants.MAX_ACCELERATION,
                                        true,
                                        0.6,
                                        Inches.of(0.5).in(Meters),
                                        "Hub Shuffle Pre Retract")
                                .withTimeout(1),
                        moveByDistance(
                                        Inches.of(3.15),
                                        shuffleVelocity,
                                        IntakeLinearConstants.MAX_ACCELERATION,
                                        false,
                                        0.0,
                                        Inches.of(0.5).in(Meters),
                                        "Hub Shuffle Extend")
                                .withTimeout(1),
                        stopRoller())
                .withName("Intake Hub Shuffle Step");
    }

    public Command linearCoast() {
        return this.runOnce(intakeLinearIO::runCoast).withName("Linear Coast");
    }

    public Command homeLinear() {
        return Commands.sequence(
                        this.runOnce(() -> intakeLinearIO.runDutyCycle(0.25, true)), this.idle())
                .finallyDo(() -> intakeLinearIO.setEncoderPosition(Rotations.of(3.7)))
                .withName("Home Linear");
    }

    @Override
    public void periodic() {
        LoggerHelper.recordCurrentCommand(this.getName(), this);
        intakeLinearIO.periodic();
        intakeRollerIO.periodic();
    }

    /**
     * Gets the linear extension of the subsystem by converting the motor's rotation.
     *
     * @return The estimated linear extension of the subsystem
     */
    public Distance getExtension() {
        return intakeLinearIO.getLinearPosition();
    }

    @Override
    public void close() {
        intakeRollerIO.close();
        intakeLinearIO.close();
    }
}
