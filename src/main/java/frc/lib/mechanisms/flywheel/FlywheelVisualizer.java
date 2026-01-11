// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.mechanisms.flywheel;

import static edu.wpi.first.units.Units.Degrees;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * The {@code FlywheelVisualizer} class is responsible for visualizing a flywheel mechanism using
 * the Mechanism2d library. It creates a graphical representation of the flywheel and its
 * components, which can be displayed on the SmartDashboard.
 * 
 * <p>
 * This class initializes a {@code LoggedMechanism2d} object with specified dimensions and
 * constructs the flywheel mechanism using {@code LoggedMechanismLigament2d} objects. The flywheel
 * is represented as a series of connected ligaments forming a circular structure.
 * 
 * <p>
 * Example Usage:
 * 
 * <pre>
 * FlywheelVisualizer visualizer = new FlywheelVisualizer("Flywheel");
 * </pre>
 * 
 */
public class FlywheelVisualizer {

    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismLigament2d roller;
    private final String name;

    private static double HEIGHT = .50; // Controls the height of the mech2d SmartDashboard
    private static double WIDTH = .50; // Controls width of the mech2d SmartDashboard

    public FlywheelVisualizer(String name)
    {
        this.name = name;
        mechanism = new LoggedMechanism2d(WIDTH, HEIGHT);

        roller = mechanism
            .getRoot("pivotPoint_", WIDTH / 2.0, HEIGHT / 2.0)
            .append(
                new LoggedMechanismLigament2d(
                    "roller ", .2, 0, 0, new Color8Bit(Color.kAliceBlue)));

        LoggedMechanismLigament2d side1 =
            roller.append(
                new LoggedMechanismLigament2d(
                    "side1 ", 0.15307, 112.5, 6, new Color8Bit(Color.kAliceBlue)));
        LoggedMechanismLigament2d side2 =
            side1.append(
                new LoggedMechanismLigament2d(
                    "side2 ", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
        LoggedMechanismLigament2d side3 =
            side2.append(
                new LoggedMechanismLigament2d(
                    "side3 ", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
        LoggedMechanismLigament2d side4 =
            side3.append(
                new LoggedMechanismLigament2d(
                    "side4 ", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
        LoggedMechanismLigament2d side5 =
            side4.append(
                new LoggedMechanismLigament2d(
                    "side5 ", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
        LoggedMechanismLigament2d side6 =
            side5.append(
                new LoggedMechanismLigament2d(
                    "side6 ", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
        LoggedMechanismLigament2d side7 =
            side6.append(
                new LoggedMechanismLigament2d(
                    "side7 ", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
        side7.append(
            new LoggedMechanismLigament2d("side8 ", 0.15307, 45, 6,
                new Color8Bit(Color.kAliceBlue)));
    }

    public void setAngle(Angle angle)
    {
        roller.setAngle(angle.in(Degrees));
        SmartDashboard.putData(name + " Visualizer", mechanism); // Creates mech2d in SmartDashboard
    }

    public void setColor(Color color)
    {
        mechanism.setBackgroundColor(new Color8Bit(color));
    }
}
