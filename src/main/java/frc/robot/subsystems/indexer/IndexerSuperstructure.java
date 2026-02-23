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

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggerHelper;

/**
 * Subsystem that controls the indexer floor and indexer centering mechanism for moving game pieces
 * within the robot. The indexer can pull game pieces in, expel them, or stop. Uses a flywheel
 * mechanism for velocity control.
 */
public class IndexerSuperstructure extends SubsystemBase {
    private final FlywheelMechanism<?> floorIO;
    private final FlywheelMechanism<?> centerIO;

    private static final LoggedTunableNumber FLOOR_SHOOT_RPS =
            new LoggedTunableNumber(
                    IndexerFloorConstants.NAME + "/ShootRPS",
                    IndexerFloorConstants.MAX_VELOCITY.in(RotationsPerSecond));

    private static final LoggedTunableNumber FLOOR_EJECT_RPS =
            new LoggedTunableNumber(IndexerFloorConstants.NAME + "/EjectRPS", -0.5);

    private static final LoggedTunableNumber FLOOR_FEED_RPS =
            new LoggedTunableNumber(
                    IndexerFloorConstants.NAME + "/FeedRPS",
                    IndexerFloorConstants.MAX_VELOCITY.in(RotationsPerSecond));

    private static final LoggedTunableNumber CENTER_SHOOT_RPS =
            new LoggedTunableNumber(
                    IndexerCenterConstants.NAME + "/ShootRPS",
                    IndexerCenterConstants.MAX_VELOCITY.in(RotationsPerSecond));

    private static final LoggedTunableNumber CENTER_EJECT_RPS =
            new LoggedTunableNumber(IndexerCenterConstants.NAME + "/EjectRPS", -0.5);

    private static final LoggedTunableNumber CENTER_FEED_RPS =
            new LoggedTunableNumber(
                    IndexerCenterConstants.NAME + "/FeedRPS",
                    IndexerCenterConstants.MAX_VELOCITY.in(RotationsPerSecond));

    /**
     * Constructs an IndexerSuperstructure subsystem.
     *
     * @param floorIO The flywheel mechanism for controlling the indexer floor motors
     * @param centerIO The flywheel mechanism for controlling the indexer centering motors
     */
    public IndexerSuperstructure(FlywheelMechanism<?> floorIO, FlywheelMechanism<?> centerIO) {
        this.floorIO = floorIO;
        this.centerIO = centerIO;
    }

    @Override
    public void periodic() {
        LoggerHelper.recordCurrentCommand(this.getName(), this);
        floorIO.periodic();
        centerIO.periodic();
    }

    private void runVelocity(AngularVelocity floorVelocity, AngularVelocity centeringVelocity) {
        floorIO.runVelocity(floorVelocity, IndexerFloorConstants.MAX_ACCELERATION, PIDSlot.SLOT_0);
        centerIO.runVelocity(
                centeringVelocity, IndexerCenterConstants.MAX_ACCELERATION, PIDSlot.SLOT_0);
    }

    /**
     * Creates a command to stop the indexer by applying brake mode.
     *
     * @return a command that stops the indexer
     */
    public Command stopCommand() {
        return this.runOnce(
                () -> {
                    floorIO.runBrake();
                    centerIO.runBrake();
                });
    }

    private void stop() {
        floorIO.runBrake();
        centerIO.runBrake();
    }

    /**
     * Creates a command to run the indexer at shooting velocities. The indexer will stop when the
     * command is interrupted or cancelled.
     *
     * @return a command that runs the indexer at shooting speed
     */
    public Command shoot() {
        return this.startEnd(
                        () ->
                                runVelocity(
                                        RotationsPerSecond.of(FLOOR_SHOOT_RPS.get()),
                                        RotationsPerSecond.of(CENTER_SHOOT_RPS.get())),
                        () -> stop())
                .withName("Shoot");
    }

    /**
     * Creates a command to run the indexer at feeding velocities to move game pieces through the
     * robot. The indexer will stop when the command is interrupted or cancelled.
     *
     * @return a command that runs the indexer at feeding speed
     */
    public Command feed() {
        return this.startEnd(
                        () ->
                                runVelocity(
                                        RotationsPerSecond.of(FLOOR_FEED_RPS.get()),
                                        RotationsPerSecond.of(CENTER_FEED_RPS.get())),
                        () -> stop())
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
                        () ->
                                runVelocity(
                                        RotationsPerSecond.of(FLOOR_EJECT_RPS.get()),
                                        RotationsPerSecond.of(CENTER_EJECT_RPS.get())),
                        () -> stop())
                .withName("Eject");
    }

    /**
     * Checks if the indexer velocity is near the current state's setpoint.
     *
     * @return true if the indexer is within tolerance of the setpoint, false otherwise
     */
    public boolean nearSetpoint() {
        return floorIO.getVelocityError().lte(IndexerFloorConstants.TOLERANCE)
                && centerIO.getVelocityError().lte(IndexerCenterConstants.TOLERANCE);
    }

    /**
     * Gets the current velocity of the indexer floor motors.
     *
     * @return The velocity in rotations per second
     */
    public double getFloorSpeed() {
        return floorIO.getVelocity().in(RotationsPerSecond);
    }

    /**
     * Gets the current velocity of the indexer centering motor.
     *
     * @return The velocity in rotations per second
     */
    public double getCenteringSpeed() {
        return centerIO.getVelocity().in(RotationsPerSecond);
    }

    /**
     * Gets the current linear velocity of the indexer floor motors.
     *
     * @return The linear velocity in meters per second.
     */
    public LinearVelocity getFloorLinearVelocity() {
        return MetersPerSecond.of(
                floorIO.getVelocity().in(RadiansPerSecond)
                        * IndexerFloorConstants.RADIUS.in(Meters));
    }

    /**
     * Gets the current linear velocity of the indexer centering motor.
     *
     * @return The linear velocity in meters per second.
     */
    public LinearVelocity getCenteringLinearVelocity() {
        return MetersPerSecond.of(
                centerIO.getVelocity().in(RadiansPerSecond)
                        * IndexerCenterConstants.RADIUS.in(Meters));
    }

    /**
     * Sets the current linear velocities of the indexer motors.
     *
     * @param floorVelocity the desired linear velocity for the floor mechanism (meters per second)
     * @param centeringVelocity the desired linear velocity for the centering mechanism (meters per
     *     second)
     */
    public void setLinearVelocity(LinearVelocity floorVelocity, LinearVelocity centeringVelocity) {
        floorIO.runVelocity(
                RadiansPerSecond.of(floorVelocity.in(MetersPerSecond)),
                IndexerFloorConstants.MAX_ACCELERATION,
                PIDSlot.SLOT_0);
        centerIO.runVelocity(
                RadiansPerSecond.of(centeringVelocity.in(MetersPerSecond)),
                IndexerCenterConstants.MAX_ACCELERATION,
                PIDSlot.SLOT_0);
    }

    /** Closes the indexer mechanism and releases resources. */
    public void close() {
        floorIO.close();
        centerIO.close();
    }
}
