// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.mechanisms.linear;

import static edu.wpi.first.units.Units.Meters;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.mechanisms.linear.LinearMechanism.LinearMechCharacteristics;

/**
 * A visualizer for linear mechanisms that displays the current distance, trajectory, and goal
 * distance using a LoggedMechanism2d. Supports mechanisms at any orientation angle.
 *
 * <p>
 * The orientation uses WPILib's Rotation3d convention (counter-clockwise positive around Y-axis):
 * <ul>
 * <li>The mechanism extends along the positive X-axis in its local frame</li>
 * <li>Pitch (Y-axis rotation) determines the angle from horizontal for 2D visualization</li>
 * <li>A pitch of 0° represents a horizontal mechanism extending forward</li>
 * <li>A pitch of -90° (-π/2 radians) represents a vertical mechanism extending upward</li>
 * <li>A pitch of 90° (π/2 radians) represents a vertical mechanism extending downward</li>
 * </ul>
 *
 * <p>
 * For 3D pose calculation, the distance is projected along the orientation direction by rotating a
 * vector [distance, 0, 0] by the orientation Rotation3d.
 */
public class LinearMechanismVisualizer {

    private static final double ARM_LENGTH = 0.25;

    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismLigament2d measured;
    private final LoggedMechanismLigament2d measuredArm;
    private final LoggedMechanismLigament2d trajectory;
    private final LoggedMechanismLigament2d trajectoryArm;
    private final LoggedMechanismLigament2d goal;
    private final LoggedMechanismLigament2d goalArm;
    private final LoggedMechanismLigament2d lowerBound;
    private final LoggedMechanismLigament2d lowerBoundArm;
    private final LoggedMechanismLigament2d upperBound;
    private final LoggedMechanismLigament2d upperBoundArm;
    private final String name;

    private Rotation3d orientation;
    private Pose3d currentPose = new Pose3d();

    public LinearMechanismVisualizer(String name, LinearMechCharacteristics characteristics)
    {
        this.name = name;
        this.orientation = characteristics.orientation();

        // Calculate the 2D angle for mechanism visualization (using pitch as the primary angle)
        // WPILib uses counter-clockwise positive, so pitch directly maps to visualization angle
        double visualAngleDegrees = Math.toDegrees(-orientation.getY());

        mechanism = new LoggedMechanism2d(3.0, 3.0, new Color8Bit(Color.kBlack));
        LoggedMechanismRoot2d root = mechanism.getRoot(name + " root", 1.5, 1.5);

        lowerBound =
            new LoggedMechanismLigament2d(name + "lowerBound",
                characteristics.minDistance().in(Meters), visualAngleDegrees, 3,
                new Color8Bit(Color.kWhite));

        lowerBoundArm = new LoggedMechanismLigament2d(name + "lowerBoundArm", ARM_LENGTH, -90, 3,
            new Color8Bit(Color.kWhite));


        upperBound =
            new LoggedMechanismLigament2d(name + "upperBound",
                characteristics.maxDistance().in(Meters), visualAngleDegrees, 3,
                new Color8Bit(Color.kWhite));

        upperBoundArm =
            new LoggedMechanismLigament2d(name + "upperBoundArm", ARM_LENGTH, -90.0, 3,
                new Color8Bit(Color.kWhite));

        measured =
            new LoggedMechanismLigament2d(name + "measured",
                characteristics.startingDistance().in(Meters), visualAngleDegrees,
                3,
                new Color8Bit(Color.kGreen));

        measuredArm =
            new LoggedMechanismLigament2d(name + "measuredArm", ARM_LENGTH, -90, 3,
                new Color8Bit(Color.kGreen));

        trajectory =
            new LoggedMechanismLigament2d(name + "trajectory",
                characteristics.startingDistance().in(Meters), visualAngleDegrees,
                3,
                new Color8Bit(Color.kYellow));

        trajectoryArm =
            new LoggedMechanismLigament2d(name + "trajectoryArm", ARM_LENGTH, -90, 3,
                new Color8Bit(Color.kYellow));

        goal = new LoggedMechanismLigament2d(name + "goal",
            characteristics.startingDistance().in(Meters),
            visualAngleDegrees, 3,
            new Color8Bit(Color.kRed));

        goalArm =
            new LoggedMechanismLigament2d(name + "goalArm", ARM_LENGTH, -90, 3,
                new Color8Bit(Color.kRed));

        root.append(lowerBound);
        lowerBound.append(lowerBoundArm);
        root.append(upperBound);
        upperBound.append(upperBoundArm);
        root.append(measured);
        measured.append(measuredArm);
        root.append(trajectory);
        trajectory.append(trajectoryArm);
        root.append(goal);
        goal.append(goalArm);
    }

    /**
     * Updates the 2D visualization angle based on the current orientation.
     */
    private void updateVisualizationAngle()
    {
        // Convert the pitch (Y rotation) to a 2D visualization angle
        // WPILib uses counter-clockwise positive, so pitch directly maps to visualization angle
        double visualAngleDegrees = Math.toDegrees(-orientation.getY());

        lowerBound.setAngle(visualAngleDegrees);
        upperBound.setAngle(visualAngleDegrees);
        measured.setAngle(visualAngleDegrees);
        trajectory.setAngle(visualAngleDegrees);
        goal.setAngle(visualAngleDegrees);
    }

    private void update()
    {
        // Calculate the 3D position based on measured distance and orientation
        // The orientation defines the direction of linear motion
        double distance = measured.getLength();

        // Create a unit vector in the direction of motion (along X-axis in local frame)
        // Then rotate it by the orientation to get the world-space direction
        Translation3d direction = new Translation3d(distance, 0, 0).rotateBy(orientation);

        // Apply offset and calculate final pose
        currentPose = new Pose3d(direction, orientation);

        SmartDashboard.putData(name + " Visualizer", mechanism);
        Logger.recordOutput(name + "Pose3d", currentPose);
    }

    public void setMeasuredDistance(Distance distance)
    {
        measured.setLength(distance.in(Meters));

        update();
    }

    public void setTrajectoryDistance(Optional<Distance> distance)
    {
        if (distance.isEmpty()) {
            trajectoryArm.setLength(0.0);
        }

        distance.ifPresent(d -> {
            trajectoryArm.setLength(ARM_LENGTH);
            trajectory.setLength(d.in(Meters));
        });

        update();
    }

    public void setGoalDistance(Optional<Distance> distance)
    {
        if (distance.isEmpty()) {
            goalArm.setLength(0.0);
        }

        distance.ifPresent(d -> {
            goalArm.setLength(ARM_LENGTH);
            goal.setLength(d.in(Meters));
        });

        update();
    }

    /**
     * Sets the orientation of the linear mechanism. This allows dynamic updates for pivoting
     * mechanisms.
     *
     * @param orientation The new orientation of the mechanism
     */
    public void setOrientation(Rotation3d orientation)
    {
        this.orientation = orientation;
        updateVisualizationAngle();
        update();
    }

    /**
     * Gets the current orientation of the linear mechanism.
     *
     * @return The current orientation
     */
    public Rotation3d getOrientation()
    {
        return orientation;
    }

    public Supplier<Pose3d> getPoseSupplier()
    {
        return () -> currentPose;
    }
}
