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

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.devices.BeamBreak;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.mechanisms.rotary.RotaryMechanism;
import frc.lib.util.FallingEdge;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.RisingEdge;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;

public class TurretSuperstructure extends SubsystemBase implements AutoCloseable {

    private static final Pose2d SHOOT_GOAL = Pose2d.kZero;

    private static final InterpolatingDoubleTreeMap magnitudeMap =
        new InterpolatingDoubleTreeMap();
    static {
        // Pre-defined lookup table values
        magnitudeMap.put(1.0, 325.0); // Use rad/s for continuity with MotorIO logging
        magnitudeMap.put(1.5, 325.0);
        magnitudeMap.put(2.0, 325.0);
        magnitudeMap.put(2.5, 325.0);
        magnitudeMap.put(3.0, 325.0);
        magnitudeMap.put(3.5, 400.0);
        magnitudeMap.put(4.0, 400.0);
        magnitudeMap.put(4.5, 400.0);
        magnitudeMap.put(5.0, 400.0);
    }

    private LoggedTunableNumber timeToBeReady = new LoggedTunableNumber("TimeToBeReady", 0.5);

    private final RobotState robotState = RobotState.getInstance();

    private final BeamBreak indexerBeamBreak;

    private final RotaryMechanism rotaryIO;
    private final RotaryMechanism indexerIO;
    private final FlywheelMechanism flywheelIO;

    public TurretSuperstructure(RotaryMechanism rotaryIO, RotaryMechanism indexerIO,
        FlywheelMechanism flywheelIO,
        BeamBreak indexerBeamBreak)
    {
        this.rotaryIO = rotaryIO;
        this.indexerIO = indexerIO;
        this.flywheelIO = flywheelIO;
        this.indexerBeamBreak = indexerBeamBreak;
    }

    private void runIndexer()
    {
        indexerIO.runVelocity(IndexerConstants.CRUISE_VELOCITY, IndexerConstants.ACCELERATION,
            PIDSlot.SLOT_0);
    }

    private void stopIndexer()
    {
        indexerIO.runVelocity(RotationsPerSecond.zero(), IndexerConstants.ACCELERATION,
            PIDSlot.SLOT_0);
    }

    @SuppressWarnings("unused")
    private Command indexerIntake()
    {
        return Commands.startEnd(this::runIndexer, this::stopIndexer)
            .until(RisingEdge.of(indexerBeamBreak::isBroken));
    }

    private Command indexerShoot()
    {
        return Commands.startEnd(this::runIndexer, this::stopIndexer)
            .until(FallingEdge.of(indexerBeamBreak::isBroken));
    }

    private void setTurretPosition(Angle angle)
    {
        rotaryIO.runPosition(angle, PIDSlot.SLOT_0);
    }

    private boolean turretIsAt(Angle angle)
    {
        return rotaryIO.nearGoal(angle, TurretConstants.TOLERANCE);
    }

    private boolean turretIsAt(Rotation2d angle)
    {
        return turretIsAt(Radians.of(angle.getRadians()));
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

    public Command moveTurretRobotRelative(Supplier<Rotation2d> robotRelativeHeading)
    {
        return Commands
            .run(() -> setTurretPosition(Radians.of(robotRelativeHeading.get().getRadians())));
    }

    private Command moveTurretFieldRelative(
        Drive drive,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        Supplier<Rotation2d> fieldRelativeHeadingSupplier)
    {
        return Commands.parallel(
            DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier,
                fieldRelativeHeadingSupplier),
            moveTurretRobotRelative(
                () -> fieldRelativeHeadingSupplier.get()
                    .plus(robotState.getRotation().unaryMinus())));
    }

    public Command shoot(
        Drive drive,
        DoubleSupplier joystickXSupplier,
        DoubleSupplier joystickYSupplier)
    {
        Supplier<Pose2d> futurePoseSupplier = () -> robotState.getEstimatedPose()
            .exp(robotState.getFieldRelativeVelocity().toTwist2d(timeToBeReady.get()));
        Supplier<AngularVelocity> flywheelVelocitySupplier = () -> RadiansPerSecond.of(
            magnitudeMap
                .get(SHOOT_GOAL.minus(futurePoseSupplier.get()).getTranslation().getNorm()));

        return Commands.sequence(
            Commands.parallel(
                moveTurretFieldRelative(
                    drive,
                    joystickXSupplier,
                    joystickYSupplier,
                    () -> SHOOT_GOAL.minus(futurePoseSupplier.get()).getRotation()),
                Commands.run(() -> spinFlywheel(flywheelVelocitySupplier.get())),
                // Wait until at goal positions and make sequences depend on this
                Commands.idle(this))
                .until(() -> flywheelIsAt(flywheelVelocitySupplier.get())
                    && turretIsAt(
                        SHOOT_GOAL.minus(futurePoseSupplier.get()).getRotation()
                            .plus(robotState.getRotation().unaryMinus()))),
            // Shoot
            indexerShoot(),
            // Spin down flywheel
            Commands.runOnce(() -> spinFlywheel(RotationsPerSecond.zero())));
    }

    @Override
    public void periodic()
    {
        rotaryIO.periodic();
        flywheelIO.periodic();
        indexerIO.periodic();
        indexerBeamBreak.periodic();

        var currentRobotHeading = robotState.getEstimatedPose().getRotation();

        // Robot relative
        var currentTurretHeading = Rotation2d.fromRadians(rotaryIO.getPosition().in(Radians));
        Logger.recordOutput("Turret/Orientation", currentTurretHeading.plus(currentRobotHeading));
    }

    @Override
    public void close()
    {
        rotaryIO.close();
        flywheelIO.close();
        indexerIO.close();
    }
}
