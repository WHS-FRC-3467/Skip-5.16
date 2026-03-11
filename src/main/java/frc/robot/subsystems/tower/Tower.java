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

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.DistanceControlledMechanism;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.util.LoggedTunableBoolean;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggerHelper;

/**
 * Subsystem that controls the tower mechanism that transfers game pieces from the indexer to the
 * shooter. The tower can stop, idle at a slow speed to hold game pieces, or shoot at full speed.
 * Uses a flywheel mechanism for velocity control.
 */
public class Tower extends SubsystemBase {

    private static final LoggedTunableNumber SHOOT_RPS =
            new LoggedTunableNumber(
                    TowerConstants.NAME + "/ShootRPS",
                    TowerConstants.MAX_VELOCITY.in(RotationsPerSecond));

    private static final LoggedTunableNumber EJECT_RPS =
            new LoggedTunableNumber(TowerConstants.NAME + "/EjectRPS", -0.5);

    private static final LoggedTunableNumber FEED_RPS =
            new LoggedTunableNumber(
                    TowerConstants.NAME + "/FeedRPS",
                    TowerConstants.MAX_VELOCITY.in(RotationsPerSecond));

    // Tuning Mode
    private static final Trigger TUNING_MODE_ENABLED =
            new Trigger(new LoggedTunableBoolean(TowerConstants.NAME + "/Tuning/Enable", false));
    private static final LoggedTunableNumber TUNING_MODE_SPEED_RPS =
            new LoggedTunableNumber(TowerConstants.NAME + "/Tuning/SpeedRPS", 0.0);

    private final DistanceControlledMechanism<FlywheelMechanism<?>> io;

    private final Command tuningModeCommand =
            Commands.sequence(
                            Commands.runOnce(this::cancelCurrentCommandIfAny),
                            createTuningRunCommand())
                    // Prevent interruptions
                    .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                    // Stop after tuning mode is disabled for safety/convenience
                    .finallyDo(() -> runVelocity(RotationsPerSecond.zero()));

    private void cancelCurrentCommandIfAny() {
        var currentCommand = this.getCurrentCommand();
        if (currentCommand == null) return;
        CommandScheduler.getInstance().cancel(currentCommand);
    }

    private Command createTuningRunCommand() {
        // Proxy prevents the sequence from requiring this subsystem.
        return this.run(
                        () ->
                                runVelocity(
                                        RotationsPerSecond.of(TUNING_MODE_SPEED_RPS.getAsDouble())))
                .asProxy();
    }

    /**
     * Constructs a new Tower subsystem with the specified flywheel mechanism.
     *
     * @param io the flywheel mechanism IO implementation for the tower
     */
    public Tower(DistanceControlledMechanism<FlywheelMechanism<?>> io) {
        this.io = io;
        TUNING_MODE_ENABLED.whileTrue(tuningModeCommand);
    }

    @Override
    public void periodic() {
        LoggerHelper.recordCurrentCommand(this.getName(), this);
        io.periodic();
    }

    /**
     * Checks if the tower is near its current velocity setpoint.
     *
     * @return true if the tower velocity is within tolerance of the setpoint
     */
    public boolean nearSetpoint() {
        return io.getVelocityError().lte(TowerConstants.TOLERANCE);
    }

    private void runVelocity(AngularVelocity velocity) {
        io.runVelocity(velocity, TowerConstants.MAX_ACCELERATION, PIDSlot.SLOT_0);
    }

    /**
     * Gets the current velocity of the tower motor.
     *
     * @return The velocity in rotations per second
     */
    public double getSpeed() {
        return io.getVelocity().in(RotationsPerSecond);
    }

    public Command fountain() {
        return this.runOnce(() -> runVelocity(RotationsPerSecond.of(5.0)));
    }

    /**
     * Creates a command to run the tower at shooting velocity to transfer game pieces to the
     * shooter. The tower will stop when the command is interrupted or cancelled.
     *
     * @return a command that runs the tower at shooting speed
     */
    public Command shoot() {
        return this.startEnd(
                        () -> runVelocity(RotationsPerSecond.of(SHOOT_RPS.get())), () -> stop())
                .withName("Shoot");
    }

    /**
     * Creates a command to run the tower at feeding velocity to move game pieces through the
     * mechanism. The tower will stop when the command is interrupted or cancelled.
     *
     * @return a command that runs the tower at feeding speed
     */
    public Command feed() {
        return this.startEnd(() -> runVelocity(RotationsPerSecond.of(FEED_RPS.get())), () -> stop())
                .withName("Feed");
    }

    /**
     * Creates a command to run the tower in reverse to eject game pieces. The tower will stop when
     * the command is interrupted or cancelled.
     *
     * @return a command that runs the tower in reverse
     */
    public Command eject() {
        return this.startEnd(
                        () -> runVelocity(RotationsPerSecond.of(EJECT_RPS.get())), () -> stop())
                .withName("Eject");
    }

    /**
     * Creates a command to stop the tower by applying brake mode.
     *
     * @return a command that stops the tower
     */
    public Command stopCommand() {
        return this.runOnce(() -> io.runBrake()).withName("Stop");
    }

    private void stop() {
        io.runBrake();
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

    /** Closes the underlying flywheel mechanism and releases resources. */
    public void close() {
        io.close();
    }
}
