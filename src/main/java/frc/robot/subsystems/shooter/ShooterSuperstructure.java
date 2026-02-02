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

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.mechanisms.rotary.RotaryMechanism;
import frc.lib.util.LoggedTrigger;
import frc.robot.RobotState;

public class ShooterSuperstructure extends SubsystemBase implements AutoCloseable {

    /** Distance from goal in meters -> hood angle in degrees */
    private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
    static {
        hoodAngleMap.put(2.604, 5.00); // Lowest
        hoodAngleMap.put(3.42, 15.00); // Highest
    }

    /** Distance from goal in meters -> flywheel speed in rotations per second */
    private static final InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();
    static {
        flywheelMap.put(2.6, 20.5); // Lowest
        flywheelMap.put(3.42, 22.0); // Highest
    }

    private final RobotState robotState = RobotState.getInstance();

    private final RotaryMechanism<?, ?> hoodIO;
    private final FlywheelMechanism<?> leftFlywheelIO;
    private final FlywheelMechanism<?> rightFlywheelIO;

    public final LoggedTrigger readyToShoot =
        new LoggedTrigger(this.getName() + "/readyToShoot", () -> {
            double dist = robotState
                .getDistanceToTarget(robotState.getEstimatedPose()).in(Meters);
            return isFlywheelAt(RotationsPerSecond.of(flywheelMap.get(dist)))
                && isHoodAt(Degrees.of(hoodAngleMap.get(dist)));
        });

    /**
     * Constructs a new ShooterSuperstructure subsystem with the specified hood and flywheel
     * mechanisms.
     * 
     * @param hoodIO the hood mechanism for adjusting shot angle
     * @param leftFlywheelIO the left flywheel mechanism for spinning up shots
     * @param rightFlywheelIO the right flywheel mechanism for spinning up shots
     */
    public ShooterSuperstructure(
        RotaryMechanism<?, ?> hoodIO,
        FlywheelMechanism<?> leftFlywheelIO,
        FlywheelMechanism<?> rightFlywheelIO)
    {
        this.hoodIO = hoodIO;
        this.leftFlywheelIO = leftFlywheelIO;
        this.rightFlywheelIO = rightFlywheelIO;
    }

    private void spinFlywheel(AngularVelocity velocity)
    {
        leftFlywheelIO.runVelocity(velocity, FlywheelConstants.MAX_ACCELERATION, PIDSlot.SLOT_0);
        rightFlywheelIO.runVelocity(velocity, FlywheelConstants.MAX_ACCELERATION, PIDSlot.SLOT_0);
    }

    private boolean isFlywheelAt(AngularVelocity velocity)
    {
        return MathUtil.isNear(
            velocity.in(RotationsPerSecond),
            leftFlywheelIO.getVelocity().in(RotationsPerSecond),
            FlywheelConstants.TOLERANCE.in(RotationsPerSecond)) &&
            MathUtil.isNear(
                velocity.in(RotationsPerSecond),
                rightFlywheelIO.getVelocity().in(RotationsPerSecond),
                FlywheelConstants.TOLERANCE.in(RotationsPerSecond));
    }

    // Hood

    private void setHoodPosition(Angle angle)
    {
        hoodIO.runPosition(angle, PIDSlot.SLOT_0);
    }

    private boolean isHoodAt(Angle angle)
    {
        return hoodIO.nearGoal(angle, HoodConstants.TOLERANCE);
    }

    /**
     * Gets the current angle of the hood.
     * 
     * @return the hood's current position angle
     */
    public Angle getHoodAngle()
    {
        return hoodIO.getPosition();
    }

    /**
     * Gets the average angular velocity of both flywheels.
     * 
     * @return the average velocity of left and right flywheels
     */
    public AngularVelocity getAverageFlywheelVelocity()
    {
        return RotationsPerSecond.of(
            (leftFlywheelIO.getVelocity().in(RotationsPerSecond) +
                rightFlywheelIO.getVelocity().in(RotationsPerSecond)) / 2.0);
    }

    /**
     * Gets the average linear velocity at the edge of both flywheels. Converts angular velocity to
     * linear velocity using the flywheel radius.
     * 
     * @return the average linear velocity at the flywheel edge in meters per second
     */
    public LinearVelocity getAverageLinearVelocity()
    {
        return MetersPerSecond.of(
            getAverageFlywheelVelocity().in(RotationsPerSecond) * 2.0 * Math.PI
                * FlywheelConstants.FLYWHEEL_RADIUS.in(Meters));
    }

    /**
     * Spins the flywheel and actuates the hood to the proper values given field-relative robot
     * pose. Perpetual command -- never spins down. Therefore, to end, this should be interrupted by
     * a parent command group or timed-out. Primarily for use in autos.
     * 
     * @return Shooter spin-up command.
     */
    public Command spinUpShooter()
    {
        Supplier<AngularVelocity> desiredFlywheelVelocitySupplier =
            () -> RotationsPerSecond.of(flywheelMap
                .get(robotState
                    .getDistanceToTarget(robotState.getEstimatedPose()).in(Meters)));
        Supplier<Angle> desiredHoodPositionSupplier = () -> Degrees.of(hoodAngleMap
            .get(robotState
                .getDistanceToTarget(robotState.getEstimatedPose()).in(Meters)));

        return Commands.run(() -> {
            spinFlywheel(desiredFlywheelVelocitySupplier.get());
            setHoodPosition(desiredHoodPositionSupplier.get());
        }, this).withName("Spin-Up Shooter");
    }

    /**
     * Prepares the subsystem to shoot, and runs a command while it is ready
     * 
     * @param whileAtPosition A command that runs while the shooter is ready to shoot. If the
     *        shooter's setpoint changes or otherwise is no longer at position, the command will be
     *        canceled, and started again once it has caught up. This is usually used for starting
     *        and stopping the indexer to ensure balls are not shot unless we are confident we will
     *        make the shot.
     * @return The command sequence
     */
    public Command prepareShot(Command whileAtPosition)
    {
        Supplier<AngularVelocity> desiredFlywheelVelocitySupplier =
            () -> RotationsPerSecond.of(flywheelMap.get(robotState
                .getDistanceToTarget(robotState.getEstimatedPose()).in(Meters)));

        Supplier<Angle> desiredHoodPositionSupplier =
            () -> Degrees.of(hoodAngleMap.get(robotState
                .getDistanceToTarget(robotState.getEstimatedPose()).in(Meters)));

        return Commands.sequence(
            Commands.parallel(
                Commands.run(() -> spinFlywheel(desiredFlywheelVelocitySupplier.get())),
                Commands.run(() -> setHoodPosition(desiredHoodPositionSupplier.get())),
                // Require this subsystem
                Commands.idle(this),

                Commands.repeatingSequence(
                    Commands.waitUntil(readyToShoot),
                    whileAtPosition.until(readyToShoot.negate()))),
            this.runOnce(() -> spinFlywheel(RotationsPerSecond.zero())));
    }

    /**
     * Creates a command to set the hood to a specific angle.
     * 
     * @param angle the target angle for the hood
     * @return command that sets the hood angle
     */
    public Command setHoodAngle(Angle angle)
    {
        return Commands.runOnce(() -> setHoodPosition(angle));
    }

    /**
     * Creates a command to set the flywheel velocity.
     * 
     * @param velocity the target angular velocity for both flywheels
     * @return command that sets the flywheel speed
     */
    public Command setFlywheelSpeed(AngularVelocity velocity)
    {
        return Commands.runOnce(() -> spinFlywheel(velocity));
    }

    /**
     * Creates a command to set the flywheel velocity (alternate spelling).
     * 
     * @param velocity the target angular velocity for both flywheels
     * @return command that sets the flywheel speed
     */
    public Command setFlyWheelSpeed(AngularVelocity velocity)
    {
        return setFlywheelSpeed(velocity);
    }

    @Override
    public void periodic()
    {
        leftFlywheelIO.periodic();
        rightFlywheelIO.periodic();
        hoodIO.periodic();
    }

    /**
     * Closes all underlying mechanisms and releases resources.
     */
    @Override
    public void close()
    {
        leftFlywheelIO.close();
        rightFlywheelIO.close();
        hoodIO.close();
    }
}
