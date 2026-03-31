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
package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.util.LoggedTrigger;
import frc.lib.util.LoggedTunableBoolean;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggerHelper;

import java.util.Set;

/**
 * Subsystem that controls the indexer floor and indexer centering mechanism for moving game pieces
 * within the robot. The indexer can pull game pieces in, expel them, or stop. Uses a flywheel
 * mechanism for velocity control.
 */
public class Indexer extends SubsystemBase {
    private final FlywheelMechanism<?> io;

    private static final LoggedTunableNumber SHOOT_RPS =
            new LoggedTunableNumber(
                    IndexerConstants.NAME + "/ShootRPS",
                    IndexerConstants.MAX_VELOCITY.in(RotationsPerSecond));

    private static final LoggedTunableNumber EJECT_RPS =
            new LoggedTunableNumber(
                    IndexerConstants.NAME + "/EjectRPS",
                    -IndexerConstants.MAX_VELOCITY.in(RotationsPerSecond));

    private static final LoggedTunableNumber FEED_RPS =
            new LoggedTunableNumber(
                    IndexerConstants.NAME + "/FeedRPS",
                    IndexerConstants.MAX_VELOCITY.in(RotationsPerSecond));

    private final Trigger tuningModeEnabled =
            new Trigger(new LoggedTunableBoolean(getName() + "/Tuning/Enable", false));
    private final LoggedTunableNumber tuningModeRPS =
            new LoggedTunableNumber(getName() + "/Tuning/SpeedRPS", 0.0);

    // From logs
    private final LoggedTunableNumber jamDetectionVelocityError =
            new LoggedTunableNumber(getName() + "/JamDetection/VelocityErrorThresholdRPS", 15.0);
    private final LoggedTunableNumber jamDetectionTorqueCurrent =
            new LoggedTunableNumber(getName() + "/JamDetection/TorqueCurrentThreshold", 65.0);
    private final LoggedTunableNumber jamDetectionTorqueCurrentResponse =
            new LoggedTunableNumber(getName() + "/JamDetection/TorqueCurrentResponse", 100.0);
    private final LoggedTunableNumber jamDetectionResponseLengthSeconds =
            new LoggedTunableNumber(getName() + "/JamDetection/ResponseLengthSeconds", 0.2);
    private final LoggedTrigger isJammed;

    private final Command tuningModeCommand =
            Commands.sequence(
                            Commands.runOnce(this::cancelCurrentCommandIfAny),
                            createTuningRunCommand())
                    .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                    .finallyDo(() -> runVelocity(RotationsPerSecond.zero()));

    private void cancelCurrentCommandIfAny() {
        var currentCommand = this.getCurrentCommand();
        if (currentCommand == null) return;
        CommandScheduler.getInstance().cancel(currentCommand);
    }

    private Command createTuningRunCommand() {
        return this.run(() -> runVelocity(RotationsPerSecond.of(tuningModeRPS.getAsDouble())))
                .asProxy();
    }

    /**
     * Constructs an Indexer subsystem.
     *
     * @param io The flywheel mechanism for controlling the indexer floor motors
     */
    public Indexer(FlywheelMechanism<?> io) {
        this.io = io;
        tuningModeEnabled.whileTrue(tuningModeCommand);

        isJammed =
                new LoggedTrigger(
                                getName() + "/IsJammed",
                                () -> {
                                    boolean velocityTripped =
                                            io.getVelocityError()
                                                    .gt(
                                                            RotationsPerSecond.of(
                                                                    jamDetectionVelocityError
                                                                            .get()));
                                    boolean currentTripped =
                                            io.getTorqueCurrent()
                                                    .gt(Amps.of(jamDetectionTorqueCurrent.get()));
                                    return velocityTripped && currentTripped;
                                })
                        .debounce(0.1);
    }

    @Override
    public void periodic() {
        LoggerHelper.recordCurrentCommand(this.getName(), this);
        io.periodic();
    }

    private void runVelocity(AngularVelocity velocity) {
        io.runVelocity(velocity, PIDSlot.SLOT_0);
    }

    /**
     * Creates a command to stop the indexer by applying coast mode.
     *
     * @return a command that stops the indexer
     */
    public Command stopCommand() {
        return this.runOnce(this::stop).withName("Stop Indexer");
    }

    private void stop() {
        io.runCoast();
    }

    /**
     * Enables tuning mode by cancelling the currently running command (if any), scheduling a
     * non-interruptible idle command, and applying the current tuning position setpoint.
     */
    void enableTuningMode() {
        CommandScheduler.getInstance().schedule(tuningModeCommand);
    }

    /**
     * Disables tuning mode by cancelling the tuning idle command and restoring the previously
     * active command when one was captured.
     */
    void disableTuningMode() {
        CommandScheduler.getInstance().cancel(tuningModeCommand);
    }

    /**
     * Run the indexer at the foundtain velocity (5RPS)
     *
     * @return a command to fountain
     */
    public Command fountain() {
        return this.runOnce(() -> runVelocity(RotationsPerSecond.of(5.0)));
    }

    /**
     * Creates a command to run the indexer at shooting velocities. The indexer will stop when the
     * command is interrupted or cancelled.
     *
     * @return a command that runs the indexer at shooting speed
     */
    public Command shoot() {
        return Commands.repeatingSequence(
                        this.runOnce(() -> runVelocity(RotationsPerSecond.of(SHOOT_RPS.get()))),
                        Commands.waitUntil(isJammed),
                        this.runOnce(
                                () ->
                                        io.runCurrent(
                                                Amps.of(
                                                        Math.copySign(
                                                                jamDetectionTorqueCurrentResponse
                                                                        .get(),
                                                                -SHOOT_RPS.get())))),
                        Commands.defer(
                                () -> Commands.waitSeconds(jamDetectionResponseLengthSeconds.get()),
                                Set.of()))
                .finallyDo(this::stop)
                .withName("Shoot");
    }

    /**
     * Creates a command to run the indexer at feeding velocities to move game pieces through the
     * robot. The indexer will stop when the command is interrupted or cancelled.
     *
     * @return a command that runs the indexer at feeding speed
     */
    public Command feed() {
        return this.startEnd(() -> runVelocity(RotationsPerSecond.of(FEED_RPS.get())), () -> stop())
                .withName("Feed");
    }

    /**
     * Creates a command to run the indexer in reverse to eject game pieces. The indexer will stop
     * when the command is interrupted or cancelled.
     *
     * @return a command that runs the indexer in reverse
     */
    public Command eject() {
        return this.startEnd(
                        () -> runVelocity(RotationsPerSecond.of(EJECT_RPS.get())), () -> stop())
                .withName("Eject");
    }

    /**
     * Checks if the indexer velocity is near the current state's setpoint.
     *
     * @return true if the indexer is within tolerance of the setpoint, false otherwise
     */
    public boolean nearSetpoint() {
        return io.getVelocityError().lte(IndexerConstants.TOLERANCE);
    }

    /**
     * Gets the current velocity of the indexer floor motors.
     *
     * @return The velocity in rotations per second
     */
    public double getFloorSpeed() {
        return io.getVelocity().in(RotationsPerSecond);
    }

    /**
     * Gets the current linear velocity of the indexer motors.
     *
     * @return The linear velocity in meters per second.
     */
    public LinearVelocity getFloorLinearVelocity() {
        return io.getLinearVelocity();
    }

    /**
     * Sets the current linear velocities of the indexer motors.
     *
     * @param velocity the desired linear velocity for the mechanism (meters per second)
     */
    public void setLinearVelocity(LinearVelocity velocity) {
        io.runLinearVelocity(velocity, PIDSlot.SLOT_0);
    }

    /** Closes the indexer mechanism and releases resources. */
    public void close() {
        io.close();
    }
}
