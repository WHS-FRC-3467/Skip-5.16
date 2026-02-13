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
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.devices.DistanceSensor;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.util.LoggedTrigger;
import frc.lib.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/**
 * Subsystem that controls the tower mechanism that transfers game pieces from the indexer to the
 * shooter. The tower can stop, idle at a slow speed to hold game pieces, or shoot at full speed.
 * Uses a flywheel mechanism for velocity control.
 */
public class Tower extends SubsystemBase {
    private static final LoggedTunableNumber STOP_RPS =
        new LoggedTunableNumber(TowerConstants.NAME + "/StopRPS", 0.0);
    private static final LoggedTunableNumber IDLE_RPS =
        new LoggedTunableNumber(TowerConstants.NAME + "/IdleRPS", -0.1);
    private static final LoggedTunableNumber Eject_RPS =
        new LoggedTunableNumber(TowerConstants.NAME + "/EjectRPS", -0.5);
    private static final LoggedTunableNumber SHOOT_RPS =
        new LoggedTunableNumber(TowerConstants.NAME + "/ShootRPS",
            TowerConstants.MAX_VELOCITY.in(RotationsPerSecond));

    private final FlywheelMechanism<?> io;
    private State state = State.STOP;

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

    @RequiredArgsConstructor
    @SuppressWarnings("Immutable")
    @Getter
    public enum State {
        STOP(() -> RotationsPerSecond.of(STOP_RPS.get())),
        IDLE(
            () -> RotationsPerSecond.of(IDLE_RPS.get())),
        EJECT(
            () -> RotationsPerSecond.of(Eject_RPS.get())),
        SHOOT(
            () -> RotationsPerSecond.of(SHOOT_RPS.get()));

        private final Supplier<AngularVelocity> stateVelocity;
    }

    /**
     * Constructs a new Tower subsystem with the specified flywheel mechanism.
     * 
     * @param io the flywheel mechanism IO implementation for the tower
     */
    public Tower(FlywheelMechanism<?> io)
    {
        this.io = io;
    }

    @Override
    public void periodic()
    {
        Logger.recordOutput(TowerConstants.NAME + "/State", this.state.name());
        io.periodic();
        laserCAN1.periodic();
        laserCAN2.periodic();
    }

    private void setState(State state)
    {
        this.state = state;
        io.runVelocity(state.stateVelocity.get(),
            TowerConstants.MAX_ACCELERATION, PIDSlot.SLOT_0);
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
    public Command setStateCommand(State state)
    {
        return this.runOnce(() -> setState(state))
            .withName(state.name());
    }

    /**
     * Holds a state until the command is interrupted. Once the command is interrupted, its state
     * will automatically be set to {@link State#IDLE}
     * 
     * In a sequence, this command is blocking and requires this subsystem
     * 
     * @param state The state to hold
     * @return The command sequence
     */
    public Command holdStateUntilInterrupted(State state)
    {
        return this.startEnd(() -> setState(state), () -> setState(State.IDLE))
            .withName(state.name() + " Until Interrupted");
    }

    /**
     * Checks if the tower is near its current velocity setpoint.
     * 
     * @return true if the tower velocity is within tolerance of the setpoint
     */
    public boolean nearSetpoint()
    {
        return MathUtil.isNear(
            state.stateVelocity.get().in(RotationsPerSecond),
            io.getVelocity().in(RotationsPerSecond),
            TowerConstants.TOLERANCE.in(RotationsPerSecond));
    }

    /**
     * Closes the underlying flywheel mechanism and releases resources.
     */
    public void close()
    {
        io.close();
    }

    /**
     * Gets the current speed of the tower in rotations per second.
     * 
     * @return the tower's current velocity in rotations per second
     */
    public double getSpeed()
    {
        return io.getVelocity().in(RotationsPerSecond);
    }

    /**
     * Determines whether FUEL is fully staged in the tower. Fully staged FUEL likely represents a
     * full hopper.
     * 
     * @return BooleanSupplier for use in commands indicating whether FUEL is fully staged in the
     *         tower.
     */
    public BooleanSupplier isStaged()
    {
        return laserCAN1Tripped.or(laserCAN2Tripped);
    }

}
