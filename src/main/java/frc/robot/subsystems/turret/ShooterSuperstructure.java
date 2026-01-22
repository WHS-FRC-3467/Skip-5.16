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

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.mechanisms.rotary.RotaryMechanism;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.RobotState;

public class ShooterSuperstructure extends SubsystemBase implements AutoCloseable {

    /** Distance from goal in meters -> hood angle in degrees */
    private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
    static {
        hoodAngleMap.put(0.00, 0.00); // Lowest
        hoodAngleMap.put(0.50, 10.00);
        hoodAngleMap.put(1.00, 20.00);
        hoodAngleMap.put(1.50, 30.00);
        hoodAngleMap.put(2.00, 40.00);
        hoodAngleMap.put(2.50, 50.00);
        hoodAngleMap.put(3.00, 60.50);
        hoodAngleMap.put(3.50, 70.00);
        hoodAngleMap.put(4.00, 80.00);
        hoodAngleMap.put(4.50, 90.00); // Highest
    }

    /** Distance from goal in meters -> flywheel speed in radians per second */
    private static final InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();
    static {
        flywheelMap.put(1.01, 43.00); // Lowest
        flywheelMap.put(2.15, 27.00);
        flywheelMap.put(2.56, 23.00);
        flywheelMap.put(3.0, 21.00);
        flywheelMap.put(3.5, 17.00);
        flywheelMap.put(4.02, 15.00);
        flywheelMap.put(4.6, 11.50);
        flywheelMap.put(4.95, 10.00);
        flywheelMap.put(5.5, 9.00);
        flywheelMap.put(6.08, 8.00); // Highest
    }

    private static final Pose2d SHOOT_GOAL = Pose2d.kZero;

    private LoggedTunableNumber timeToBeReady = new LoggedTunableNumber("TimeToBeReady", 0.5);

    private final RobotState robotState = RobotState.getInstance();

    private final RotaryMechanism hoodIO;
    private final RotaryMechanism indexerIO;
    private final FlywheelMechanism flywheelIO;

    public ShooterSuperstructure(RotaryMechanism hoodIO,
        RotaryMechanism indexerIO,
        FlywheelMechanism flywheelIO)
    {
        this.hoodIO = hoodIO;
        this.indexerIO = indexerIO;
        this.flywheelIO = flywheelIO;
    }

    private void spinFlywheel(AngularVelocity velocity)
    {
        flywheelIO.runVelocity(velocity, FlywheelConstants.MAX_ACCELERATION, PIDSlot.SLOT_0);
    }

    private boolean flywheelIsAt(AngularVelocity velocity)
    {
        return MathUtil.isNear(
            velocity.in(RotationsPerSecond),
            flywheelIO.getVelocity().in(RotationsPerSecond),
            FlywheelConstants.TOLERANCE.in(RotationsPerSecond));
    }

    // Hood

    private void setHoodPosition(Angle angle)
    {
        hoodIO.runPosition(angle, PIDSlot.SLOT_0);
    }

    private boolean hoodIsAt(Angle angle)
    {
        return hoodIO.nearGoal(angle, HoodConstants.TOLERANCE);
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
        Supplier<Pose2d> futurePoseSupplier = () -> robotState.getEstimatedPose()
            .exp(robotState.getFieldRelativeVelocity().toTwist2d(timeToBeReady.get()));
        Supplier<AngularVelocity> desiredFlywheelVelocitySupplier = () -> RadiansPerSecond.of(
            flywheelMap
                .get(SHOOT_GOAL.minus(futurePoseSupplier.get()).getTranslation().getNorm()));
        Supplier<Angle> desiredHoodPositionSupplier = () -> Degrees.of(hoodAngleMap
            .get(SHOOT_GOAL.minus(futurePoseSupplier.get()).getTranslation().getNorm()));

        Trigger ready = new Trigger(() -> flywheelIsAt(desiredFlywheelVelocitySupplier.get())
            && hoodIsAt(desiredHoodPositionSupplier.get()));

        return Commands.sequence(
            Commands.parallel(
                Commands.run(() -> spinFlywheel(desiredFlywheelVelocitySupplier.get())),
                Commands.run(() -> setHoodPosition(desiredHoodPositionSupplier.get())),
                // Require this subsystem
                Commands.idle(this),

                Commands.repeatingSequence(
                    Commands.waitUntil(ready),
                    whileAtPosition.until(ready.negate()))),
            this.runOnce(() -> spinFlywheel(RotationsPerSecond.zero())));
    }

    // Create commands for simple shooter tasks.
    // Set Hood Position
    public Command setHoodAngle(Angle angle)
    {
        return Commands.runOnce(() -> setHoodPosition(angle));
    }

    // Set Shooter Speed
    public Command setFlyWheelSpeed(AngularVelocity velocity)
    {
        return Commands.runOnce(() -> spinFlywheel(velocity));
    }



    @Override
    public void periodic()
    {
        flywheelIO.periodic();
        indexerIO.periodic();
        hoodIO.periodic();
    }

    @Override
    public void close()
    {
        flywheelIO.close();
        indexerIO.close();
        hoodIO.close();
    }
}
