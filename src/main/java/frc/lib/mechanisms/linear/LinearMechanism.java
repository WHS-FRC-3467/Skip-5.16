// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.mechanisms.linear;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import java.util.Optional;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorIO.ControlType;
import frc.lib.mechanisms.Mechanism;

/**
 * Abstract class for linear mechanisms, which are mechanisms that move in a straight line. This
 * class extends Mechanism and provides characteristics specific to linear mechanisms. Supports
 * mechanisms at any orientation angle, including dynamic angle updates for pivoting mechanisms.
 */
public abstract class LinearMechanism<T extends MotorIO> extends Mechanism<T> {

    /**
     * Characteristics for a linear mechanism.
     *
     * @param minDistance Minimum extension distance
     * @param maxDistance Maximum extension distance
     * @param startingDistance Starting extension distance
     * @param drumRadius The radius of the drum causing linear motion
     * @param orientation The 3D orientation of the linear mechanism's axis of motion. The mechanism
     *        extends along the positive X-axis in its local frame, then rotated by this
     *        orientation. Uses WPILib's counter-clockwise positive convention around Y-axis:
     *        <ul>
     *        <li>Pitch of 0° = horizontal mechanism extending forward</li>
     *        <li>Pitch of -90° (-π/2 radians) = vertical mechanism extending upward (elevator)</li>
     *        <li>Pitch of 90° (π/2 radians) = vertical mechanism extending downward</li>
     *        </ul>
     */
    public record LinearMechCharacteristics(
        Distance minDistance,
        Distance maxDistance,
        Distance startingDistance,
        Distance drumRadius,
        Rotation3d orientation) {}

    private final double drumRadiusMeters;
    private final LinearMechanismVisualizer visualizer;

    public LinearMechanism(String name, LinearMechCharacteristics characteristics, T io) {
        super(name, io);
        visualizer = new LinearMechanismVisualizer(name, characteristics);
        drumRadiusMeters = characteristics.drumRadius().in(Meters);
    }

    protected Distance toDistance(Angle angle) {
        return Meters.of(angle.in(Radians) * drumRadiusMeters);
    }

    protected Angle toAngle(Distance distance) {
        return Radians.of(distance.in(Meters) / drumRadiusMeters);
    }

    private Optional<Distance> getTrajectoryDistance() {
        if (inputs.controlType != ControlType.POSITION || inputs.positionError == null) {
            return Optional.empty();
        }

        return Optional.of(toDistance(inputs.activeTrajectoryPosition));
    }

    private Optional<Distance> getGoalDistance() {
        if (inputs.controlType != ControlType.POSITION || inputs.positionError == null) {
            return Optional.empty();
        }

        return Optional.of(toDistance(inputs.goalPosition));
    }

    // Checks if mechanism is near a goal position within a specified tolerance
    public boolean nearGoal(Distance goalPosition, Distance tolerance) {
        return MathUtil.isNear(
            toDistance(getPosition()).in(BaseUnits.DistanceUnit),
            goalPosition.in(BaseUnits.DistanceUnit),
            tolerance.in(BaseUnits.DistanceUnit));
    }

    @Override
    public void periodic() {
        super.periodic();

        visualizer.setMeasuredDistance(toDistance(inputs.position));
        visualizer.setTrajectoryDistance(getTrajectoryDistance());
        visualizer.setGoalDistance(getGoalDistance());
    }

    /**
     * Updates the orientation of the linear mechanism. This is useful for pivoting linear
     * mechanisms where the angle changes dynamically.
     *
     * @param orientation The new orientation of the mechanism
     */
    public void setOrientation(Rotation3d orientation) {
        visualizer.setOrientation(orientation);
    }

    /**
     * Gets the current orientation of the linear mechanism.
     *
     * @return The current orientation
     */
    public Rotation3d getOrientation() {
        return visualizer.getOrientation();
    }
}
