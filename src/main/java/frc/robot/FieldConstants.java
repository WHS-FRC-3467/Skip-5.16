// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import java.util.List;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Distance;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
    public static final AprilTagFieldLayout APRILTAG_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static final Distance FIELD_LENGTH = Meters.of(APRILTAG_LAYOUT.getFieldLength());
    public static final Distance FIELD_WIDTH = Meters.of(APRILTAG_LAYOUT.getFieldWidth());

    public static final Distance STARTING_LINE_X = Inches.of(299.438);

    public static final Translation2d FIELD_CENTER =
        new Translation2d(FIELD_LENGTH.in(Meters) / 2, FIELD_WIDTH.in(Meters) / 2);

    public static final Distance ALGAE_DIAMETER = Meters.of(.41);

    public static final List<Translation2d> ALLIANCE_STATION_POLYGON =
        List.of(
            new Translation2d(0, 0),
            new Translation2d(0, FIELD_WIDTH.in(Meters)),
            new Translation2d(1.5, FIELD_WIDTH.in(Meters)),
            new Translation2d(1.5, 0));

}
