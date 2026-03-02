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
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.DistanceControlledMechanism;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.mechanisms.rotary.RotaryMechanism;
import frc.lib.util.LoggedTrigger;
import frc.lib.util.LoggedTunableBoolean;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggerHelper;
import frc.robot.Constants;
import frc.robot.FieldConstants.Hub;
import frc.robot.RobotState;
import frc.robot.RobotState.Target;
import org.littletonrobotics.junction.Logger;

public class ShooterSuperstructure extends SubsystemBase implements AutoCloseable {

    /** Distance from hub in meters -> hood angle in degrees */
    private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    static {
        hoodAngleMap.put(1.30, 0.0);
        hoodAngleMap.put(1.72, 5.0);
        hoodAngleMap.put(2.1, 5.0);
        hoodAngleMap.put(3.05, 6.0);
        hoodAngleMap.put(3.54, 8.0);
        hoodAngleMap.put(4.6, 16.0);
    }

    /** Distance from hub in meters -> flywheel speed in rotations per second */
    private static final InterpolatingDoubleTreeMap hubFlywheelMap =
            new InterpolatingDoubleTreeMap();

    static {
        hubFlywheelMap.put(1.30, 40.6);
        hubFlywheelMap.put(1.72, 42.6);
        hubFlywheelMap.put(2.1, 44.6);
        hubFlywheelMap.put(3.05, 48.6);
        hubFlywheelMap.put(3.54, 50.6);
        hubFlywheelMap.put(4.6, 52.1);
    }

    /** Distance from feed pose in meters -> flywheel speed in rotations per second */
    private static final InterpolatingDoubleTreeMap feedFlywheelMap =
            new InterpolatingDoubleTreeMap();

    static {
        feedFlywheelMap.put(3.0, 17.0);
        feedFlywheelMap.put(4.0, 18.0);
        feedFlywheelMap.put(5.0, 20.0);
        feedFlywheelMap.put(6.0, 22.0);
        feedFlywheelMap.put(7.0, 25.0);
        feedFlywheelMap.put(8.0, 27.0);
        feedFlywheelMap.put(9.0, 29.0);
        feedFlywheelMap.put(10.0, 30.5);
        feedFlywheelMap.put(11.0, 32.0);
        feedFlywheelMap.put(12.0, 34.0);
        feedFlywheelMap.put(13.0, 35.0);
    }

    private final RobotState robotState = RobotState.getInstance();

    private final RotaryMechanism<?, ?> hoodIO;
    private final DistanceControlledMechanism<FlywheelMechanism<?>> leftFlywheelIO;
    private final DistanceControlledMechanism<FlywheelMechanism<?>> rightFlywheelIO;

    private final Debouncer readyToShootDebounder = new Debouncer(0.1, DebounceType.kFalling);

    public final LoggedTrigger shooterWithinTolerance =
            new LoggedTrigger(
                    this.getName() + "/shooterWithinTolerance",
                    () ->
                            isFlywheelAt(getDesiredFlywheelVelocity())
                                    && isHoodAt(getDesiredHoodAngle()));

    public final LoggedTrigger readyToShoot =
            new LoggedTrigger(
                    this.getName() + "/readyToShoot",
                    () -> readyToShootDebounder.calculate(shooterWithinTolerance.getAsBoolean()));

    public final LoggedTrigger atHubSetpoints =
            new LoggedTrigger(
                    this.getName() + "/atHubSetpoints",
                    () -> {
                        // Distance between robot and hub centers
                        // Assume the Robot is pressed against the Hub. Hardcoded as part of no
                        // vision fallback.
                        double dist = (Hub.WIDTH + Constants.FULL_ROBOT_LENGTH.in(Meters)) / 2.0;
                        return isFlywheelAt(RotationsPerSecond.of(hubFlywheelMap.get(dist)))
                                && isHoodAt(Degrees.of(hoodAngleMap.get(dist)));
                    });

    private final LoggedTunableBoolean tuningMode =
            new LoggedTunableBoolean(getName() + "/Tuning/Enable", false);
    private final LoggedTunableNumber tuningFlywheelSpeedRPS =
            new LoggedTunableNumber(getName() + "/Tuning/FlywheelSpeedRPS", 0.0);
    private final LoggedTunableNumber tuningHoodAngleDegrees =
            new LoggedTunableNumber(getName() + "/Tuning/HoodAngleDegrees", 0.0);

    private final LoggedTunableNumber flywheelTrimDefaultRPS =
            new LoggedTunableNumber(getName() + "/FlywheelTrimDefaultRPS", 0.0);

    private AngularVelocity flywheelTrimRPS = RotationsPerSecond.of(flywheelTrimDefaultRPS.get());
    private static final double MAX_TRIM_RPS = 6.0;
    private static final double MAX_TRIM_RPS_PER_SEC = 2;

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
            DistanceControlledMechanism<FlywheelMechanism<?>> leftFlywheelIO,
            DistanceControlledMechanism<FlywheelMechanism<?>> rightFlywheelIO) {
        this.hoodIO = hoodIO;
        this.leftFlywheelIO = leftFlywheelIO;
        this.rightFlywheelIO = rightFlywheelIO;
    }

    private void spinFlywheel(AngularVelocity velocity) {
        leftFlywheelIO.runVelocity(
                velocity.plus(flywheelTrimRPS), FlywheelConstants.MAX_ACCELERATION, PIDSlot.SLOT_0);
        rightFlywheelIO.runVelocity(
                velocity.plus(flywheelTrimRPS), FlywheelConstants.MAX_ACCELERATION, PIDSlot.SLOT_0);
    }

    private boolean isFlywheelAt(AngularVelocity velocity) {
        return MathUtil.isNear(
                        velocity.in(RotationsPerSecond),
                        leftFlywheelIO.getVelocity().in(RotationsPerSecond),
                        FlywheelConstants.TOLERANCE.in(RotationsPerSecond))
                && MathUtil.isNear(
                        velocity.in(RotationsPerSecond),
                        rightFlywheelIO.getVelocity().in(RotationsPerSecond),
                        FlywheelConstants.TOLERANCE.in(RotationsPerSecond));
    }

    // Hood
    private void setHoodPosition(Angle angle) {
        hoodIO.runPosition(angle, PIDSlot.SLOT_0);
    }

    private boolean isHoodAt(Angle angle) {
        return hoodIO.nearGoal(angle, HoodConstants.TOLERANCE);
    }

    /**
     * Gets the current angle of the hood.
     *
     * @return the hood's current position angle
     */
    public Angle getHoodAngle() {
        return hoodIO.getPosition();
    }

    /**
     * Gets the average angular velocity of both flywheels.
     *
     * @return the average velocity of left and right flywheels
     */
    public AngularVelocity getAverageFlywheelVelocity() {
        return RotationsPerSecond.of(
                (leftFlywheelIO.getVelocity().in(RotationsPerSecond)
                                + rightFlywheelIO.getVelocity().in(RotationsPerSecond))
                        / 2.0);
    }

    /**
     * Gets the average linear velocity at the edge of both flywheels. Converts angular velocity to
     * linear velocity using the flywheel radius.
     *
     * @return the average linear velocity at the flywheel edge in meters per second
     */
    public LinearVelocity getAverageLinearVelocity() {
        return MetersPerSecond.of(
                getAverageFlywheelVelocity().in(RotationsPerSecond)
                        * 2.0
                        * Math.PI
                        * FlywheelConstants.FLYWHEEL_RADIUS.in(Meters));
    }

    private AngularVelocity getDesiredFlywheelVelocity() {
        // InterpolatingDoubleTreeMap flywheelMap =
        //         switch (robotState.getTarget()) {
        //             case HUB -> hubFlywheelMap;
        //             case FEED_LEFT, FEED_RIGHT -> feedFlywheelMap;
        //         };

        if (robotState.getTarget() != Target.HUB) {
            return RotationsPerSecond.of(50.0);
        }

        return RotationsPerSecond.of(
                hubFlywheelMap.get(robotState.getDistanceToTarget().in(Meters)));
    }

    private Angle getDesiredHoodAngle() {
        if (robotState.getTarget() == Target.HUB) {
            return Degrees.of(hoodAngleMap.get(robotState.getDistanceToTarget().in(Meters)));
        }

        return Degrees.of(24.0);
    }

    // Gets ball trajectory exit angle relative to horizontal, accounting for hood angle and
    // physical offset of the hood from horizontal
    public Angle getExitAngle() {
        return Degrees.of(90).minus(HoodConstants.MIN_ANGLE_OFFSET).minus(hoodIO.getPosition());
    }

    /**
     * Statically spins the flywheel and actuates the hood to the proper values for a HUB SHOT given
     * a provided distance. ONLY valid for HUB shots in the CURRENT ALLIANCE ZONE. If called in the
     * trench or neutral zone, will spin flywheel to proper speed but keep the hood low to prevent
     * collision. Perpetual command -- never spins down. Therefore, to end, this should be
     * interrupted by a parent command group or timed-out. Primarily for use in autos.
     *
     * @param distance the distance from the desired robot shot position to the HUB.
     * @return Static non-updating HUB only shooter spin-up command.
     */
    public Command spinUpShooterToHubDistance(Distance distance) {
        return Commands.run(
                        () -> {
                            spinFlywheel(
                                    RotationsPerSecond.of(hubFlywheelMap.get(distance.in(Meters))));
                            if (robotState.hoodSafe.getAsBoolean()) {
                                setHoodPosition(Degrees.of(hoodAngleMap.get(distance.in(Meters))));
                            } else {
                                setHoodPosition(Degrees.zero());
                            }
                        },
                        this)
                .withName("Spin-Up Shooter to Distance");
    }

    /**
     * Dynamically spins the flywheel and actuates the hood to the proper values for ANY target shot
     * given current field-relative robot pose. Valid for ANY target. Perpetual command -- never
     * spins down. Therefore, to end, this should be interrupted by a parent command group or
     * timed-out.
     *
     * @return Dynamically-updating ALL TARGET shooter spin-up command.
     */
    public Command spinUpShooter() {
        return Commands.run(
                        () -> {
                            spinFlywheel(getDesiredFlywheelVelocity());
                            setHoodPosition(getDesiredHoodAngle());
                        },
                        this)
                .withName("Spin-Up Shooter");
    }

    /**
     * Prepares the subsystem to shoot at the HUB, and runs a command while it is ready
     *
     * @param whileAtPosition A command that runs while the shooter is ready to shoot at the HUB. If
     *     shooting is disrupted because shooter readiness drops, attempt a flywheel/hood adjustment
     *     and, if successful, re-commence shooting. Only valid for HUB shots. This is usually used
     *     for starting and stopping the indexer to ensure balls are not shot unless we are
     *     confident we will make the shot. Shooter remains spun-up at the end of this command.
     * @return The command sequence
     */
    public Command shootFuel(Command whileAtPosition) {
        return Commands.sequence(
                        Commands.parallel(
                                spinUpShooter(),
                                Commands.repeatingSequence(
                                        Commands.waitUntil(readyToShoot),
                                        whileAtPosition.until(readyToShoot.negate()))))
                .withName("Shoot Fuel");
    }

    /**
     * Creates a command to set the hood to a specific angle.
     *
     * @param angle the target angle for the hood
     * @return command that sets the hood angle
     */
    public Command setHoodAngle(Angle angle) {
        return Commands.runOnce(() -> setHoodPosition(angle)).withName("Set Hood Angle");
    }

    /**
     * Creates a command to force the hood to a specific angle.
     *
     * @param angle the target angle for the hood
     * @return command that sets the hood angle
     */
    public Command forceHoodAngle(Angle angle) {
        return this.run(() -> setHoodPosition(angle)).withName("Force Hood Angle");
    }

    /**
     * Creates a command to set the flywheel velocity (alternate spelling).
     *
     * @param velocity the target angular velocity for both flywheels
     * @return command that sets the flywheel speed
     */
    public Command setFlywheelSpeed(AngularVelocity velocity) {
        return Commands.runOnce(() -> spinFlywheel(velocity)).withName("Set Flywheel Speed");
    }

    /**
     * Creates a command to coast the flywheel. Use after shooting in auto.
     *
     * @return command that coasts the flywheel.
     */
    public Command coastFlywheel() {
        return Commands.runOnce(
                () -> {
                    leftFlywheelIO.runCoast();
                    rightFlywheelIO.runCoast();
                });
    }

    public Command stopFlywheels() {
        return this.runOnce(
                () -> {
                    leftFlywheelIO.runCoast();
                    rightFlywheelIO.runCoast();
                });
    }

    public Command stopAndStow() {
        return Commands.sequence(stopFlywheels(), setHoodAngle(Rotations.zero()));
    }

    public Command homeHood() {
        return Commands.sequence(this.runOnce(() -> hoodIO.runDutyCycle(-0.1, true)), this.idle())
                .finallyDo(
                        () -> {
                            hoodIO.setEncoderPosition(Rotations.zero());
                            hoodIO.runBrake();
                        });
    }

    /**
     * A blocking command that lowers the hood and waits until it is zeroed. This should be included
     * with a timeout to compensate for possible sensor error. Primarily for use in autos.
     */
    public Command retractHood() {
        return setHoodAngle(Rotations.zero())
                .andThen(
                        Commands.waitUntil(
                                () -> hoodIO.getPosition().lte(HoodConstants.TOLERANCE)));
    }

    public Command trimFlywheelSpeedUp() {
        return Commands.run(
                        () -> {
                            double dt = 0.02;
                            double deltaRPS = MAX_TRIM_RPS_PER_SEC * dt;
                            double updated = flywheelTrimRPS.in(RotationsPerSecond) + deltaRPS;
                            flywheelTrimRPS =
                                    RotationsPerSecond.of(
                                            MathUtil.clamp(updated, -MAX_TRIM_RPS, MAX_TRIM_RPS));
                        })
                .withName("Trim Flywheel Speed Up");
    }

    public Command trimFlywheelSpeedDown() {
        return Commands.run(
                        () -> {
                            double dt = 0.02;
                            double deltaRPS = -MAX_TRIM_RPS_PER_SEC * dt;
                            double updated = flywheelTrimRPS.in(RotationsPerSecond) + deltaRPS;
                            flywheelTrimRPS =
                                    RotationsPerSecond.of(
                                            MathUtil.clamp(updated, -MAX_TRIM_RPS, MAX_TRIM_RPS));
                        })
                .withName("Trim Flywheel Speed Down");
    }

    @Override
    public void periodic() {
        // Dashboard change overwrites driver trim adjustments but trims will persist across
        // disable/enable cycles
        if (flywheelTrimDefaultRPS.hasChanged(hashCode())) {
            flywheelTrimRPS = RotationsPerSecond.of(flywheelTrimDefaultRPS.get());
        }

        if (tuningMode.get()) {
            if (tuningMode.hasChanged(hashCode())
                    || tuningFlywheelSpeedRPS.hasChanged(hashCode())
                    || tuningHoodAngleDegrees.hasChanged(hashCode())) {
                spinFlywheel(RotationsPerSecond.of(tuningFlywheelSpeedRPS.get()));
                setHoodPosition(Degrees.of(tuningHoodAngleDegrees.get()));
            }

            Logger.recordOutput(
                    getName() + "/Tuning/DistanceToTargetMeters",
                    robotState.getDistanceToTarget().in(Meters));
        }
        LoggerHelper.recordCurrentCommand(this.getName(), this);

        leftFlywheelIO.periodic();
        rightFlywheelIO.periodic();
        hoodIO.periodic();

        Logger.recordOutput(
                getName() + "/VelocityErrorDifference",
                leftFlywheelIO.getVelocityError().minus(rightFlywheelIO.getVelocityError()));

        Logger.recordOutput(
                getName() + "/TotalDrawWatts",
                leftFlywheelIO.getAppliedVoltage().times(leftFlywheelIO.getSupplyCurrent()));

        Logger.recordOutput(getName() + "/FlywheelTrimRPS", flywheelTrimRPS.in(RotationsPerSecond));
    }

    /** Closes all underlying mechanisms and releases resources. */
    @Override
    public void close() {
        leftFlywheelIO.close();
        rightFlywheelIO.close();
        hoodIO.close();
    }
}
