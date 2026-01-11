// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// https://github.com/Mechanical-Advantage/RobotCode2024Public/blob/main/src/main/java/org/littletonrobotics/frc2024/subsystems/superstructure/arm/ArmVisualizer.java

package frc.lib.mechanisms.rotary;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.mechanisms.rotary.RotaryMechanism.RotaryMechCharacteristics;

/**
 * A visualizer for rotary mechanisms that displays the current angle, trajectory, and goal angle
 * using a LoggedMechanism2d.
 */
public class RotaryVisualizer {

    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismLigament2d measured;
    private final LoggedMechanismLigament2d trajectory;
    private final LoggedMechanismLigament2d goal;
    private final LoggedMechanismLigament2d lowerBound;
    private final LoggedMechanismLigament2d upperBound;
    private final String name;

    private final double armLength;

    private Pose3d currentPose = new Pose3d();

    private final RotaryMechCharacteristics constants;

    public RotaryVisualizer(String name, RotaryMechCharacteristics constants)
    {
        this.name = name;
        this.constants = constants;
        mechanism = new LoggedMechanism2d(3.0, 3.0, new Color8Bit(Color.kBlack));
        LoggedMechanismRoot2d root = mechanism.getRoot(name + " root", 1.5, 1.5);

        armLength = constants.armLength().in(Meters);

        if (constants.maxAngle().minus(constants.minAngle()).in(Rotations) < 1) {
            lowerBound =
                new LoggedMechanismLigament2d(name + "Lower Bound",
                    constants.armLength().in(Meters),
                    constants.minAngle().in(Degrees), 3,
                    new Color8Bit(Color.kWhite));

            upperBound =
                new LoggedMechanismLigament2d(name + "Upper Bound",
                    constants.armLength().in(Meters),
                    constants.maxAngle().in(Degrees), 3,
                    new Color8Bit(Color.kWhite));
        } else {
            lowerBound =
                new LoggedMechanismLigament2d(name + "Lower Bound", 0.0,
                    constants.minAngle().in(Degrees), 3,
                    new Color8Bit(Color.kWhite));

            upperBound =
                new LoggedMechanismLigament2d(name + "Upper Bound", 0.0,
                    constants.maxAngle().in(Degrees), 3,
                    new Color8Bit(Color.kWhite));
        }

        measured =
            new LoggedMechanismLigament2d(name + "Measured", armLength,
                constants.startingAngle().in(Radians), 3,
                new Color8Bit(Color.kGreen));

        trajectory =
            new LoggedMechanismLigament2d(name + "Trajectory", armLength,
                constants.startingAngle().in(Radians), 3,
                new Color8Bit(Color.kYellow));

        goal = new LoggedMechanismLigament2d(name + "Goal", armLength,
            constants.startingAngle().in(Radians), 3,
            new Color8Bit(Color.kRed));

        root.append(lowerBound);
        root.append(upperBound);
        root.append(measured);
        root.append(trajectory);
        root.append(goal);
    }

    private void update()
    {
        switch (constants.axis()) {
            case ROLL:
                currentPose = Pose3d.kZero.rotateBy(
                    new Rotation3d(Degrees.of(measured.getAngle()), Degrees.zero(),
                        Degrees.zero()));
                break;
            case PITCH:
                currentPose = Pose3d.kZero.rotateBy(
                    new Rotation3d(Degrees.zero(), Degrees.of(measured.getAngle()),
                        Degrees.zero()));
                break;
            case YAW:
                currentPose = Pose3d.kZero.rotateBy(
                    new Rotation3d(Degrees.zero(), Degrees.zero(),
                        Degrees.of(measured.getAngle())));
                break;

            default:
                break;
        }

        SmartDashboard.putData(name + " Visualizer", mechanism);
        Logger.recordOutput(name + "Pose3d", currentPose);
    }

    public void setCurrentAngle(Angle angle)
    {
        measured.setAngle(Rotation2d.fromRadians(angle.in(Radians)));

        update();
    }

    public void setTrajectoryAngle(Optional<Angle> angle)
    {
        if (angle.isEmpty()) {
            trajectory.setLength(0.0);
        }

        angle.ifPresent((a) -> {
            trajectory.setLength(armLength);
            trajectory.setAngle(a.in(Degrees));
        });

        update();
    }

    public void setGoalAngle(Optional<Angle> angle)
    {
        if (angle.isEmpty()) {
            goal.setLength(0.0);
        }

        angle.ifPresent((a) -> {
            goal.setLength(armLength);
            goal.setAngle(a.in(Degrees));
        });

        update();
    }

    public Supplier<Pose3d> getPoseSupplier()
    {
        return () -> currentPose;
    }
}
