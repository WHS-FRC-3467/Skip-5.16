// Copyright (C) 2026 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import java.util.List;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

/**
 * Contains various field dimensions and useful reference points. All poses
 * have a blue alliance origin.
 */
public class FieldConstants {
    public static final AprilTagFieldLayout APRILTAG_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public static final Distance FIELD_LENGTH = Meters.of(APRILTAG_LAYOUT.getFieldLength());
    public static final Distance FIELD_WIDTH = Meters.of(APRILTAG_LAYOUT.getFieldWidth());

    public static final Distance STARTING_LINE_X = Inches.of(157.61);

    public static final Translation2d FIELD_CENTER = new Translation2d(FIELD_LENGTH.in(Meters) / 2,
            FIELD_WIDTH.in(Meters) / 2);

    public static final List<Translation2d> ALLIANCE_ZONE = List.of(
            new Translation2d(0, 0),
            new Translation2d(0, FIELD_WIDTH.in(Meters)),
            new Translation2d(Inches.of(158.6), FIELD_WIDTH),
            new Translation2d(Inches.of(158.6), Inches.of(0)));

    public static final Distance ROBOT_STARTING_LINE = Inches.of(158.6);

    public static final List<Translation2d> NEUTRAL_ZONE = List.of(
            new Translation2d(FIELD_LENGTH.div(2).minus(Inches.of(143.50)).in(Meters), Inches.of(0).in(Meters)),
            new Translation2d(FIELD_LENGTH.div(2).minus(Inches.of(143.50)).in(Meters), FIELD_WIDTH.in(Meters)),
            new Translation2d(FIELD_LENGTH.div(2).plus(Inches.of(143.50)).in(Meters), FIELD_WIDTH.in(Meters)),
            new Translation2d(FIELD_LENGTH.div(2).plus(Inches.of(143.50)).in(Meters), Inches.of(0).in(Meters)));

    public static final List<Distance> TUNNEL_CENTER_Y = List.of(
            Inches.of(24.97), FIELD_WIDTH.minus(Inches.of(24.97)));
    public static final Translation2d HUB_CENTER = new Translation2d(
            Inches.of(158.6),
            FIELD_WIDTH.div(2));

    //MJW - PR 30: Shooter Lookup Table
    public static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
    static {
        hoodAngleMap.put(0.00, 0.00); //Lowest
        hoodAngleMap.put(0.50, 10.00);
        hoodAngleMap.put(1.00, 20.00);
        hoodAngleMap.put(1.50, 30.00);
        hoodAngleMap.put(2.00, 40.00);
        hoodAngleMap.put(2.50, 50.00);
        hoodAngleMap.put(3.00, 60.50);
        hoodAngleMap.put(3.50, 70.00);
        hoodAngleMap.put(4.00, 80.00);
        hoodAngleMap.put(4.50, 90.00); //Highest
    }

    public static final InterpolatingDoubleTreeMap flyWheelMap = new InterpolatingDoubleTreeMap();
    static {
        flyWheelMap.put(1.01, 43.00); //Lowest
        flyWheelMap.put(2.15, 27.00);
        flyWheelMap.put(2.56, 23.00);
        flyWheelMap.put(3.0, 21.00);
        flyWheelMap.put(3.5, 17.00);
        flyWheelMap.put(4.02, 15.00);
        flyWheelMap.put(4.6, 11.50);
        flyWheelMap.put(4.95, 10.00);
        flyWheelMap.put(5.5,9.00);
        flyWheelMap.put(6.08,8.00); //Highest
    }
    //MJW - PR 30: Shooter Lookup Table

    public static final Distance FUEL_DIAMETER = Inches.of(5.91);
    public static final Mass FUEL_WEIGHT = Pounds.of(0.474);

}
