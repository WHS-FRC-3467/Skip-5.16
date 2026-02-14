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
import java.util.function.BooleanSupplier;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.devices.DistanceSensor;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.util.LoggedTrigger;
import frc.lib.util.LoggedTunableNumber;

/**
 * Subsystem that controls the tower mechanism that transfers game pieces from the indexer to the
 * shooter. The tower can stop, idle at a slow speed to hold game pieces, or shoot at full speed.
 * Uses a flywheel mechanism for velocity control.
 */
public class Tower extends SubsystemBase {

    private static final LoggedTunableNumber SHOOT_RPS =
        new LoggedTunableNumber(TowerConstants.NAME + "/ShootRPS",
            TowerConstants.MAX_VELOCITY.in(RotationsPerSecond));

    private static final LoggedTunableNumber Eject_RPS =
        new LoggedTunableNumber(TowerConstants.NAME + "/EjectRPS", -0.5);

    private static final LoggedTunableNumber FEED_RPS =
        new LoggedTunableNumber(TowerConstants.NAME + "/FeedRPS",
            TowerConstants.MAX_VELOCITY.in(RotationsPerSecond));

    private final FlywheelMechanism<?> io;

    private final DistanceSensor laserCAN1 = TowerConstants.getLaserCAN1();
    private final DistanceSensor laserCAN2 = TowerConstants.getLaserCAN2();

    public final LoggedTrigger laserCAN1Tripped =
        new LoggedTrigger(TowerConstants.LASERCAN1_NAME,
            () -> laserCAN1.betweenDistance(TowerConstants.MINIMUM_TRIP_DISTANCE,
                TowerConstants.MAXIMUM_TRIP_DISTANCE));
    public final LoggedTrigger laserCAN2Tripped =
        new LoggedTrigger(TowerConstants.LASERCAN2_NAME,
            () -> laserCAN2.betweenDistance(TowerConstants.MINIMUM_TRIP_DISTANCE,
                TowerConstants.MAXIMUM_TRIP_DISTANCE));

    public final LoggedTrigger isStaged = new LoggedTrigger(TowerConstants.NAME + "/IsStaged",
        laserCAN1Tripped.or(laserCAN2Tripped));

    /**
     * Constructs a new Tower subsystem with the specified flywheel mechanism.
     *
     * @param io the flywheel mechanism IO implementation for the tower
     */
    public Tower(FlywheelMechanism<?> io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.periodic();
        laserCAN1.periodic();
        laserCAN2.periodic();
    }

    /**
     * Checks if the tower is near its current velocity setpoint.
     *
     * @return true if the tower velocity is within tolerance of the setpoint
     */
    public boolean nearSetpoint() {
        return io.getVelocityError().lte(TowerConstants.TOLERANCE);
    }

    /**
     * Determines whether FUEL is fully staged in the tower. Fully staged FUEL likely represents a
     * full hopper.
     *
     * @return BooleanSupplier for use in commands indicating whether FUEL is fully staged in the
     *         tower.
     */
    public BooleanSupplier isStaged() {
        return laserCAN1Tripped.or(laserCAN2Tripped);
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

    /**
     * Creates a command to run the tower at shooting velocity to transfer game pieces to the
     * shooter. The tower will stop when the command is interrupted or cancelled.
     *
     * @return a command that runs the tower at shooting speed
     */
    public Command shoot() {
        return Commands.startEnd(
            () -> runVelocity(RotationsPerSecond.of(SHOOT_RPS.get())),
            () -> stop());
    }

    /**
     * Creates a command to run the tower at feeding velocity to move game pieces through the
     * mechanism. The tower will stop when the command is interrupted or cancelled.
     *
     * @return a command that runs the tower at feeding speed
     */
    public Command feed() {
        return Commands.startEnd(
            () -> runVelocity(RotationsPerSecond.of(FEED_RPS.get())),
            () -> stop());
    }

    /**
     * Creates a command to run the tower in reverse to eject game pieces. The tower will stop when
     * the command is interrupted or cancelled.
     *
     * @return a command that runs the tower in reverse
     */
    public Command eject() {
        return Commands.startEnd(
            () -> runVelocity(RotationsPerSecond.of(Eject_RPS.get())),
            () -> stop());
    }

    /**
     * Creates a command to stop the tower by applying brake mode.
     *
     * @return a command that stops the tower
     */
    public Command stopCommand() {
        return Commands.runOnce(() -> io.runBrake(), this);
    }

    private void stop() {
        io.runBrake();
    }

    /**
     * Closes the underlying flywheel mechanism and releases resources.
     */
    public void close() {
        io.close();
    }

}
