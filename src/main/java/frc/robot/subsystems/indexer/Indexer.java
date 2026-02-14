// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.util.LoggedTunableNumber;

/**
 * Subsystem that controls the indexer mechanism for moving game pieces within the robot. The
 * indexer can pull game pieces in, expel them, or stop. Uses a flywheel mechanism for velocity
 * control.
 */
public class Indexer extends SubsystemBase {
    private final FlywheelMechanism<?> io;

    private static final LoggedTunableNumber SHOOT_RPS =
        new LoggedTunableNumber(IndexerConstants.NAME + "/ShootRPS",
            IndexerConstants.MAX_VELOCITY.in(RotationsPerSecond));

    private static final LoggedTunableNumber Eject_RPS =
        new LoggedTunableNumber(IndexerConstants.NAME + "/EjectRPS", -0.5);

    private static final LoggedTunableNumber FEED_RPS =
        new LoggedTunableNumber(IndexerConstants.NAME + "/FeedRPS",
            IndexerConstants.MAX_VELOCITY.in(RotationsPerSecond));

    /**
     * Constructs an Indexer subsystem.
     *
     * @param io The flywheel mechanism for controlling the indexer motor
     */
    public Indexer(FlywheelMechanism<?> io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.periodic();
    }

    private void runVelocity(AngularVelocity velocity) {
        io.runVelocity(velocity, IndexerConstants.MAX_ACCELERATION, PIDSlot.SLOT_0);
    }

    /**
     * Creates a command to stop the indexer by applying brake mode.
     *
     * @return a command that stops the indexer
     */
    public Command stopCommand() {
        return Commands.runOnce(() -> io.runBrake(), this);
    }

    private void stop() {
        io.runBrake();
    }

    /**
     * Creates a command to run the indexer at shooting velocity. The indexer will stop when the
     * command is interrupted or cancelled.
     *
     * @return a command that runs the indexer at shooting speed
     */
    public Command shoot() {
        return this.startEnd(
            () -> runVelocity(RotationsPerSecond.of(SHOOT_RPS.get())),
            () -> stop());
    }

    /**
     * Creates a command to run the indexer at feeding velocity to move game pieces through the
     * robot. The indexer will stop when the command is interrupted or cancelled.
     *
     * @return a command that runs the indexer at feeding speed
     */
    public Command feed() {
        return Commands.startEnd(
            () -> runVelocity(RotationsPerSecond.of(FEED_RPS.get())),
            () -> stop());
    }

    /**
     * Creates a command to run the indexer in reverse to eject game pieces. The indexer will stop
     * when the command is interrupted or cancelled.
     *
     * @return a command that runs the indexer in reverse
     */
    public Command eject() {
        return Commands.startEnd(
            () -> runVelocity(RotationsPerSecond.of(Eject_RPS.get())),
            () -> stop());
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
     * Gets the current velocity of the indexer motor.
     *
     * @return The velocity in rotations per second
     */
    public double getSpeed() {
        return io.getVelocity().in(RotationsPerSecond);
    }

    /**
     * Gets the current linear velocity of the indexer motor.
     *
     * @return The velocity in rotations per second multiplied by the radius
     */
    public double getLinearVelocity() {
        return io.getVelocity().in(RadiansPerSecond)
            * IndexerConstants.RADIUS.in(Meters);
    }

    /**
     * Sets the current linear velocity of the indexer motor.
     */
    public void setLinearVelocity(LinearVelocity velocity) {
        io.runVelocity(RadiansPerSecond.of(velocity.in(MetersPerSecond)),
            IndexerConstants.MAX_ACCELERATION, PIDSlot.SLOT_0);
    }

    /**
     * Gets the current linear position of the indexer motor.
     *
     * @return The Position in radians multiplied by 2_PIr
     */
    public double getLinearPosition() {
        return io.getPosition().in(Radians)
            * (2 * Math.PI * IndexerConstants.RADIUS.in(Meters));
    }

    /**
     * Sets the current linear position of the indexer motor.
     */
    public void setLinearPosition(Distance position) {
        io.runPosition(Radians.of(
            position.in(Meters)
                / IndexerConstants.RADIUS.in(Meters)),
            PIDSlot.SLOT_0);
    }

    /**
     * Closes the indexer mechanism and releases resources.
     */
    public void close() {
        io.close();
    }
}
