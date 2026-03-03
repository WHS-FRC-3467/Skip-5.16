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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
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
            new LoggedTunableNumber(IntakeRollerConstants.NAME + "/IntakeRPS", 35.0);

    private static final LoggedTunableNumber ROLLER_EJECT_RPS =
            new LoggedTunableNumber(IntakeRollerConstants.NAME + "/EjectRPS", -35.0);

    private static final LoggedTunableNumber SLOW_MPS =
            new LoggedTunableNumber(IntakeLinearConstants.NAME + "/SlowMPS", 0.05);

    private final DistanceControlledMechanism<LinearMechanism<?>> intakeLinearIO;
    private final FlywheelMechanism<?> intakeRollerIO;

    private final LoggedTrigger isExtended;
    private final LoggedTrigger isRetracted;

    private final TrapezoidProfile fastMotionProfiler =
            new TrapezoidProfile(
                    new Constraints(
                            IntakeLinearConstants.CRUISE_VELOCITY.in(MetersPerSecond),
                            IntakeLinearConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond)));

    private TrapezoidProfile slowMotionProfiler =
            new TrapezoidProfile(
                    new Constraints(
                            SLOW_MPS.get(),
                            IntakeLinearConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond)));

    private State setpointState = new State();
    private State goalState = new State();

    private TrapezoidProfile activeProfiler = fastMotionProfiler;

    private double lastTimestamp = Timer.getTimestamp();
    private boolean runProfile = true;

    public IntakeSuperstructure(
            DistanceControlledMechanism<LinearMechanism<?>> intakeLinearIO,
            FlywheelMechanism<?> intakeRollerIO) {

        this.intakeLinearIO = intakeLinearIO;
        this.intakeRollerIO = intakeRollerIO;

        intakeLinearIO.runLinearPosition(IntakeLinearConstants.MIN_DISTANCE, PIDSlot.SLOT_0);

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
                                        IntakeLinearConstants.MIN_DISTANCE.in(Meters)
                                                + Inches.of(2.0).in(Meters),
                                        intakeLinearIO.getLinearPosition().in(Meters),
                                        IntakeLinearConstants.TOLERANCE.in(Meters)));
    }

    private void resetSetpoint() {
        double current = intakeLinearIO.getLinearPosition().in(Meters);
        setpointState = new State(current, 0.0);
    }

    private TrapezoidProfile createProfiler(double maxVelocityMetersPerSecond) {
        return new TrapezoidProfile(
                new Constraints(
                        maxVelocityMetersPerSecond,
                        IntakeLinearConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond)));
    }

    private void startProfile(TrapezoidProfile profiler, double goalMeters) {
        resetSetpoint();

        goalState.position = goalMeters;
        goalState.velocity = 0.0;

        activeProfiler = profiler;
    }

    /** Returns true if the intake roller is running and the intake is extended. */
    public boolean isIntaking() {
        return intakeRollerIO.getVelocity().in(RotationsPerSecond) > 1.0
                && isExtended.getAsBoolean();
    }

    private Command moveToPosition(TrapezoidProfile profiler, double goalMeters, String name) {
        return Commands.runOnce(() -> startProfile(profiler, goalMeters)).withName(name);
    }

    private Command moveByInches(
            double inches,
            TrapezoidProfile profiler,
            boolean runRoller,
            double rollerScale,
            String name) {

        return Commands.sequence(
                        Commands.runOnce(
                                () -> {
                                    double current = intakeLinearIO.getLinearPosition().in(Meters);
                                    double delta = Inches.of(inches).in(Meters);

                                    double goal =
                                            MathUtil.clamp(
                                                    current + delta,
                                                    IntakeLinearConstants.MIN_DISTANCE.in(Meters),
                                                    IntakeLinearConstants.MAX_DISTANCE.in(Meters));

                                    startProfile(profiler, goal);
                                }),
                        runRoller
                                ? runRoller(
                                        () ->
                                                RotationsPerSecond.of(ROLLER_INTAKE_RPS.get())
                                                        .times(rollerScale))
                                : Commands.none(),
                        Commands.waitUntil(
                                () ->
                                        MathUtil.isNear(
                                                goalState.position,
                                                intakeLinearIO.getLinearPosition().in(Meters),
                                                IntakeLinearConstants.TOLERANCE.in(Meters))))
                .withName(name);
    }

    private Command runRoller(Supplier<AngularVelocity> angularVelocity) {
        return this.runOnce(
                        () ->
                                intakeRollerIO.runVelocity(
                                        angularVelocity.get(),
                                        IntakeRollerConstants.MAX_ACCELERATION,
                                        PIDSlot.SLOT_0))
                .withName("Run Roller");
    }

    public Command stopRoller() {
        return this.runOnce(intakeRollerIO::runBrake).withName("Stop Roller");
    }

    public Command ejectRoller() {
        return Commands.startEnd(
                        () ->
                                intakeRollerIO.runVelocity(
                                        RotationsPerSecond.of(ROLLER_EJECT_RPS.get()),
                                        IntakeRollerConstants.MAX_ACCELERATION,
                                        PIDSlot.SLOT_0),
                        intakeRollerIO::runBrake)
                .withName("Eject Roller");
    }

    public Command intake() {
        return Commands.sequence(
                runRoller(() -> RotationsPerSecond.of(ROLLER_INTAKE_RPS.get())),
                moveToPosition(
                        fastMotionProfiler,
                        IntakeLinearConstants.MAX_DISTANCE.in(Meters),
                        "Extend Linear"));
    }

    public Command extendIntake() {
        return Commands.sequence(
                runRoller(RotationsPerSecond::zero),
                moveToPosition(
                        fastMotionProfiler,
                        IntakeLinearConstants.MAX_DISTANCE.in(Meters),
                        "Extend Linear"));
    }

    public Command retractIntake() {
        return Commands.sequence(
                        runRoller(() -> RotationsPerSecond.of(ROLLER_INTAKE_RPS.get())),
                        moveToPosition(
                                fastMotionProfiler,
                                IntakeLinearConstants.MIN_DISTANCE.in(Meters)
                                        + Inches.of(2.0).in(Meters),
                                "Retract Linear"),
                        Commands.waitUntil(isRetracted),
                        stopRoller())
                .withName("Retract Intake");
    }

    public Command slowRetract(LinearVelocity retractSpeed) {
        return Commands.sequence(
                Commands.runOnce(
                        () ->
                                slowMotionProfiler =
                                        createProfiler(retractSpeed.in(MetersPerSecond))),
                runRoller(() -> RotationsPerSecond.of(ROLLER_INTAKE_RPS.get()).times(0.6)),
                moveToPosition(
                        slowMotionProfiler,
                        IntakeLinearConstants.MIN_DISTANCE.in(Meters) + Inches.of(2.0).in(Meters),
                        "Slow Retract"),
                Commands.waitUntil(isRetracted),
                stopRoller());
    }

    public Command slowRetract() {
        return slowRetract(MetersPerSecond.of(SLOW_MPS.get()));
    }

    public Command shuffleStep() {
        return Commands.sequence(
                        Commands.either(extendIntake(), Commands.none(), isRetracted),
                        moveByInches(-6, slowMotionProfiler, true, 0.6, "Shuffle Retract"),
                        moveByInches(3, slowMotionProfiler, false, 0.0, "Shuffle Extend"))
                .withName("The Brendan Shuffle");
    }

    public Command linearCoast() {
        return this.runOnce(intakeLinearIO::runCoast).withName("Linear Coast");
    }

    public Command homeLinear() {
        return Commands.sequence(
                        Commands.runOnce(() -> runProfile = false),
                        this.runOnce(() -> intakeLinearIO.runDutyCycle(0.25, true)),
                        this.idle())
                .finallyDo(
                        () -> {
                            intakeLinearIO.setEncoderPosition(Rotations.of(3.56));
                            runProfile = true;
                        });
    }

    @Override
    public void periodic() {

        if (SLOW_MPS.hasChanged(hashCode())) {
            slowMotionProfiler =
                    new TrapezoidProfile(
                            new Constraints(
                                    SLOW_MPS.get(),
                                    IntakeLinearConstants.MAX_ACCELERATION.in(
                                            MetersPerSecondPerSecond)));
        }

        double now = Timer.getTimestamp();
        double delta = now - lastTimestamp;
        lastTimestamp = now;

        setpointState = activeProfiler.calculate(delta, setpointState, goalState);

        if (runProfile) {
            intakeLinearIO.runUnprofiledLinearPosition(
                    Meters.of(setpointState.position), PIDSlot.SLOT_0);
        }

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
