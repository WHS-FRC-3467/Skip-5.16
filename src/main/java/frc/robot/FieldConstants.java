package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import java.util.List;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
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
            new Translation2d(FIELD_LENGTH.div(2).minus(Inches.of(143.50)), Inches.of(0)),
            new Translation2d(FIELD_LENGTH.div(2).minus(Inches.of(143.50)), FIELD_WIDTH),
            new Translation2d(FIELD_LENGTH.div(2).plus(Inches.of(143.50)), FIELD_WIDTH),
            new Translation2d(FIELD_LENGTH.div(2).plus(Inches.of(143.50)), Inches.of(0)));

    public static final List<Distance> TUNNEL_CENTER_Y = List.of(
            Inches.of(24.97),
            FIELD_WIDTH.minus(Inches.of(24.97)));

    public static final Translation2d HUB_CENTER = new Translation2d(
            Inches.of(158.6),
            FIELD_WIDTH.div(2));

    public static final Distance FUEL_DIAMETER = Inches.of(5.91);
    public static final Mass FUEL_WEIGHT = Pounds.of(0.474);

    public static final Pose3d DEPOT_FIRST_BALL = new Pose3d(
        FUEL_DIAMETER.div(2),
        FIELD_WIDTH.div(2).plus(Inches.of(75.93)).minus(FUEL_DIAMETER.times(2.5)),
        FUEL_DIAMETER.div(2),
        new Rotation3d());

    public static final Pose3d NEUTRAL_ZONE_FIRST_FUEL = new Pose3d(
        FIELD_CENTER.getMeasureX().minus(Inches.of(35.95)).plus(FUEL_DIAMETER.div(2)),
        FIELD_CENTER.getMeasureY().minus(Inches.of(90.95)).plus(FUEL_DIAMETER.div(2)),
        FUEL_DIAMETER.div(2),
        new Rotation3d());


}
