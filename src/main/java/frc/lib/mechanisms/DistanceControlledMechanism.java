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
package frc.lib.mechanisms;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.lib.io.motor.MotorIO.PIDSlot;
import lombok.experimental.Delegate;

/**
 * Wrapper around a rotational {@link Mechanism} that exposes linear (distance-based) control and
 * feedback. This is useful for mechanisms such as elevators, winches, or drivetrains where linear
 * motion is derived from a rotating element with a known radius.
 *
 * <p>
 * The class transparently converts between linear and angular units when issuing commands or
 * reading sensor feedback. All motor control is delegated to the underlying mechanism.
 * </p>
 *
 * @param <T> The mechanism type being wrapped
 */
public class DistanceControlledMechanism<T extends Mechanism<?>> {

    /** Underlying rotational mechanism. */
    @Delegate
    private final T mechanism;

    /** Radius used to convert between angular and linear motion (meters). */
    private final double radiusMeters;

    /**
     * Creates a distance-controlled wrapper around a rotational mechanism.
     *
     * @param mechanism The mechanism to wrap
     * @param radius The effective radius used for linear-to-angular conversion
     */
    public DistanceControlledMechanism(T mechanism, Distance radius) {
        this.mechanism = mechanism;
        this.radiusMeters = radius.in(Meters);
    }

    /**
     * Runs the mechanism to a target linear position.
     *
     * @param position Desired linear position
     * @param slot PID slot to use
     */
    public void runLinearPosition(Distance position, PIDSlot slot) {
        Angle angle = Radians.of(position.in(Meters) / radiusMeters);
        mechanism.runPosition(angle, slot);
    }

    /**
     * Runs the mechanism at a target linear velocity with acceleration limiting.
     *
     * @param velocity Desired linear velocity
     * @param acceleration Maximum linear acceleration
     * @param slot PID slot to use
     */
    public void runLinearVelocity(
        LinearVelocity velocity,
        LinearAcceleration acceleration,
        PIDSlot slot) {

        AngularVelocity angularVelocity =
            RadiansPerSecond.of(velocity.in(MetersPerSecond) / radiusMeters);

        AngularAcceleration angularAcceleration =
            RadiansPerSecondPerSecond.of(
                acceleration.in(MetersPerSecondPerSecond) / radiusMeters);

        mechanism.runVelocity(angularVelocity, angularAcceleration, slot);
    }

    /**
     * Gets the current linear position of the mechanism.
     *
     * @return Linear position
     */
    public Distance getLinearPosition() {
        double angularPositionRadians = mechanism.getPosition().in(Radians);
        return Meters.of(angularPositionRadians * radiusMeters);
    }

    /**
     * Gets the error between the target and current linear position.
     *
     * @return Linear position error
     */
    public Distance getLinearPositionError() {
        double angularPositionErrorRadians =
            mechanism.getPositionError().in(Radians);
        return Meters.of(angularPositionErrorRadians * radiusMeters);
    }

    /**
     * Gets the current linear velocity of the mechanism.
     *
     * @return Linear velocity
     */
    public LinearVelocity getLinearVelocity() {
        double angularVelocityRadiansPerSecond =
            mechanism.getVelocity().in(RadiansPerSecond);
        return MetersPerSecond.of(
            angularVelocityRadiansPerSecond * radiusMeters);
    }

    /**
     * Gets the error between the target and current linear velocity.
     *
     * @return Linear velocity error
     */
    public LinearVelocity getLinearVelocityError() {
        double angularVelocityErrorRadiansPerSecond =
            mechanism.getVelocityError().in(RadiansPerSecond);
        return MetersPerSecond.of(
            angularVelocityErrorRadiansPerSecond * radiusMeters);
    }
}
