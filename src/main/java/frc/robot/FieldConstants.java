// Copyright (C) 2026 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import frc.lib.util.FieldUtil;

/**
 * Contains information for location of field element and other useful reference points.
 *
 * <p>
 * NOTE: All constants are defined relative to the field coordinate system, and from the perspective
 * of the blue alliance station
 */
public class FieldConstants {
    public static final FieldType FIELD_TYPE = FieldType.ANDYMARK;

    public static final AprilTagLayoutType DEFAULT_APRIL_TAG_TYPE = AprilTagLayoutType.OFFICIAL;

    // Field dimensions
    public static final double FIELD_LENGTH =
        AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
    public static final double FIELD_WIDTH =
        AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();

    public static final Translation2d FIELD_CENTER = new Translation2d(FIELD_LENGTH / 2,
        FIELD_WIDTH / 2);

    public static final Translation3d FIELD_CENTER3D =
        new Translation3d(FIELD_LENGTH / 2, FIELD_WIDTH / 2, 0.00);

    /**
     * Officially defined and relevant vertical lines found on the field (defined by X-axis offset)
     */
    public static class LinesVertical {
        public static final double CENTER = FIELD_LENGTH / 2.0;
        public static final double STARTING =
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX();
        public static final double ALLIANCE_ZONE = STARTING;
        public static final double HUB_CENTER =
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + Hub.WIDTH / 2.0;
        public static final double NEUTRAL_ZONE_NEAR = CENTER - Units.inchesToMeters(120);
        public static final double NEUTRAL_ZONE_FAR = CENTER + Units.inchesToMeters(120);
    }

    /**
     * Officially defined and relevant horizontal lines found on the field (defined by Y-axis
     * offset)
     *
     * <p>
     * NOTE: The field element start and end are always left to right from the perspective of the
     * alliance station
     */
    public static class LinesHorizontal {

        public static final double CENTER = FIELD_WIDTH / 2.0;

        // Right of hub
        public static final double RIGHT_BUMP_START = Hub.NEAR_RIGHT_CORNER.getY();
        public static final double RIGHT_BUMP_END = RIGHT_BUMP_START - RightBump.WIDTH;
        public static final double RIGHT_TRENCH_OPEN_START =
            RIGHT_BUMP_END - Units.inchesToMeters(12.0);
        public static final double RIGHT_TRENCH_OPEN_END = 0;

        // Left of hub
        public static final double LEFT_BUMP_END = Hub.NEAR_LEFT_CORNER.getY();
        public static final double LEFT_BUMP_START = LEFT_BUMP_END + LeftBump.WIDTH;
        public static final double LEFT_TRENCH_OPEN_END =
            LEFT_BUMP_START + Units.inchesToMeters(12.0);
        public static final double LEFT_TRENCH_OPEN_START = FIELD_WIDTH;
    }

    /** Hub related constants */
    public static class Hub {

        // Dimensions
        public static final double WIDTH = Units.inchesToMeters(47.0);
        public static final double HEIGHT =
            Units.inchesToMeters(72.0); // includes the catcher at the top
        public static final double INNER_WIDTH = Units.inchesToMeters(41.7);
        public static final double INNER_HEIGHT = Units.inchesToMeters(56.5);

        // 2D Locations


        // Relevant reference points on alliance side
        public static final Translation3d TOP_CENTER_POINT =
            new Translation3d(
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + WIDTH / 2.0,
                FIELD_WIDTH / 2.0,
                HEIGHT);

        public static final Translation3d INNER_CENTER_POINT =
            new Translation3d(
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + WIDTH / 2.0,
                FIELD_WIDTH / 2.0,
                INNER_HEIGHT);

        public static final Translation2d NEAR_LEFT_CORNER =
            new Translation2d(TOP_CENTER_POINT.getX() - WIDTH / 2.0,
                FIELD_WIDTH / 2.0 + WIDTH / 2.0);
        public static final Translation2d NEAR_RIGHT_CORNER =
            new Translation2d(TOP_CENTER_POINT.getX() - WIDTH / 2.0,
                FIELD_WIDTH / 2.0 - WIDTH / 2.0);
        public static final Translation2d FAR_LEFT_CORNER =
            new Translation2d(TOP_CENTER_POINT.getX() + WIDTH / 2.0,
                FIELD_WIDTH / 2.0 + WIDTH / 2.0);
        public static final Translation2d FAR_RIGHT_CORNER =
            new Translation2d(TOP_CENTER_POINT.getX() + WIDTH / 2.0,
                FIELD_WIDTH / 2.0 - WIDTH / 2.0);

        // Hub faces
        public static final Pose2d NEAR_FACE =
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().toPose2d();
        public static final Pose2d FAR_FACE =
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(20).get().toPose2d();
        public static final Pose2d RIGHT_FACE =
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(18).get().toPose2d();
        public static final Pose2d LEFT_FACE =
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(21).get().toPose2d();
    }

    /** Left Bump related constants */
    public static class LeftBump {

        // Dimensions
        public static final double WIDTH = Units.inchesToMeters(73.0);
        public static final double HEIGHT = Units.inchesToMeters(6.513);
        public static final double DEPTH = Units.inchesToMeters(44.4);

        // Relevant reference points on alliance side
        public static final Translation2d NEAR_LEFT_CORNER =
            new Translation2d(LinesVertical.HUB_CENTER - WIDTH / 2, Units.inchesToMeters(255));
        public static final Translation2d NEAR_RIGHT_CORNER = Hub.NEAR_LEFT_CORNER;
        public static final Translation2d FAR_LEFT_CORNER =
            new Translation2d(LinesVertical.HUB_CENTER + WIDTH / 2, Units.inchesToMeters(255));
        public static final Translation2d FAR_RIGHT_CORNER = Hub.FAR_LEFT_CORNER;
    }

    /** Right Bump related constants */
    public static class RightBump {
        // Dimensions
        public static final double WIDTH = Units.inchesToMeters(73.0);
        public static final double HEIGHT = Units.inchesToMeters(6.513);
        public static final double DEPTH = Units.inchesToMeters(44.4);

        // Relevant reference points on alliance side
        public static final Translation2d NEAR_LEFT_CORNER =
            new Translation2d(LinesVertical.HUB_CENTER + WIDTH / 2, Units.inchesToMeters(255));
        public static final Translation2d NEAR_RIGHT_CORNER = Hub.NEAR_LEFT_CORNER;
        public static final Translation2d FAR_LEFT_CORNER =
            new Translation2d(LinesVertical.HUB_CENTER - WIDTH / 2, Units.inchesToMeters(255));
        public static final Translation2d FAR_RIGHT_CORNER = Hub.FAR_LEFT_CORNER;
    }

    /** Left Trench related constants */
    public static class LeftTrench {
        // Dimensions
        public static final double WIDTH = Units.inchesToMeters(65.65);
        public static final double DEPTH = Units.inchesToMeters(47.0);
        public static final double HEIGHT = Units.inchesToMeters(40.25);
        public static final double OPENING_WIDTH = Units.inchesToMeters(50.34);
        public static final double OPENING_HEIGHT = Units.inchesToMeters(22.25);

        // Relevant reference points on alliance side
        public static final Translation3d OPENING_TOP_LEFT =
            new Translation3d(LinesVertical.HUB_CENTER, FIELD_WIDTH, OPENING_HEIGHT);
        public static final Translation3d OPENING_TOP_RIGHT =
            new Translation3d(LinesVertical.HUB_CENTER, FIELD_WIDTH - OPENING_WIDTH,
                OPENING_HEIGHT);
    }

    public static class RightTrench {

        // Dimensions
        public static final double WIDTH = Units.inchesToMeters(65.65);
        public static final double DEPTH = Units.inchesToMeters(47.0);
        public static final double HEIGHT = Units.inchesToMeters(40.25);
        public static final double OPENING_WIDTH = Units.inchesToMeters(50.34);
        public static final double OPENING_HEIGHT = Units.inchesToMeters(22.25);

        // Relevant reference points on alliance side
        public static final Translation3d OPENING_TOP_LEFT =
            new Translation3d(LinesVertical.HUB_CENTER, OPENING_WIDTH, OPENING_HEIGHT);
        public static final Translation3d OPENING_TOP_RIGHT =
            new Translation3d(LinesVertical.HUB_CENTER, 0, OPENING_HEIGHT);
    }

    /** Tower related constants */
    public static class Tower {
        // Dimensions
        public static final double WIDTH = Units.inchesToMeters(49.25);
        public static final double DEPTH = Units.inchesToMeters(45.0);
        public static final double HEIGHT = Units.inchesToMeters(78.25);
        public static final double INNDER_OPENING_WIDTH = Units.inchesToMeters(32.250);
        public static final double FRONT_FACE_X = Units.inchesToMeters(43.51);

        public static final double UPRIGHT_HEIGHT = Units.inchesToMeters(72.1);

        // Rung heights from the floor
        public static final double LOW_RUNG_HEIGHT = Units.inchesToMeters(27.0);
        public static final double MID_RUNG_HEIGHT = Units.inchesToMeters(45.0);
        public static final double HIGH_RUNG_HEIGHT = Units.inchesToMeters(63.0);

        // Relevant reference points on alliance side
        public static final Translation2d CENTER_POINT =
            new Translation2d(
                FRONT_FACE_X, AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY());
        public static final Translation2d LEFT_UPRIGHT =
            new Translation2d(
                FRONT_FACE_X,
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY()
                    + INNDER_OPENING_WIDTH / 2
                    + Units.inchesToMeters(0.75));
        public static final Translation2d RIGHT_UPRIGHT =
            new Translation2d(
                FRONT_FACE_X,
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY()
                    - INNDER_OPENING_WIDTH / 2
                    - Units.inchesToMeters(0.75));
    }

    public static class Depot {
        // Dimensions
        public static final double WIDTH = Units.inchesToMeters(42.0);
        public static final double DEPTH = Units.inchesToMeters(27.0);
        public static final double HEIGHT = Units.inchesToMeters(1.125);
        public static final double DISTANCE_FROM_CENTER_Y = Units.inchesToMeters(75.93);

        // Relevant reference points on alliance side
        public static final Translation3d depotCenter =
            new Translation3d(DEPTH, (FIELD_WIDTH / 2) + DISTANCE_FROM_CENTER_Y, HEIGHT);
        public static final Translation3d leftCorner =
            new Translation3d(DEPTH, (FIELD_WIDTH / 2) + DISTANCE_FROM_CENTER_Y + (WIDTH / 2),
                HEIGHT);
        public static final Translation3d rightCorner =
            new Translation3d(DEPTH, (FIELD_WIDTH / 2) + DISTANCE_FROM_CENTER_Y - (WIDTH / 2),
                HEIGHT);
    }

    public static class Outpost {
        // Dimensions
        public static final double WIDTH = Units.inchesToMeters(31.8);
        public static final double OPENING_DISTANCE_FROM_FLOOR = Units.inchesToMeters(28.1);
        public static final double HEIGHT = Units.inchesToMeters(7.0);

        // Relevant reference points on alliance side
        public static final Translation2d CENTER_POINT =
            new Translation2d(0,
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(29).get().getY());
    }

    @RequiredArgsConstructor
    public enum FieldType {
        ANDYMARK("andymark"),
        WELDED("welded");

        @Getter
        private final String jsonFolder;
    }

    @SuppressWarnings("ImmutableEnumChecker")
    public enum AprilTagLayoutType {
        OFFICIAL("2026-official"),
        NONE("2026-none");

        private final String name;
        private volatile AprilTagFieldLayout layout;
        private volatile String layoutString;

        AprilTagLayoutType(String name)
        {
            this.name = name;
        }

        public AprilTagFieldLayout getLayout()
        {
            if (layout == null) {
                synchronized (this) {
                    if (layout == null) {
                        try {
                            Path p =
                                Constants.disableHAL
                                    ? Path.of(
                                        "src",
                                        "main",
                                        "deploy",
                                        "apriltags",
                                        FIELD_TYPE.getJsonFolder(),
                                        name + ".json")
                                    : Path.of(
                                        Filesystem.getDeployDirectory().getPath(),
                                        "apriltags",
                                        FIELD_TYPE.getJsonFolder(),
                                        name + ".json");
                            layout = new AprilTagFieldLayout(p);
                            layoutString = new ObjectMapper().writeValueAsString(layout);
                        } catch (IOException e) {
                            throw new RuntimeException(e);
                        }
                    }
                }
            }
            return layout;
        }

        public String getLayoutString()
        {
            if (layoutString == null) {
                getLayout();
            }
            return layoutString;
        }
    }

    public static final Distance FUEL_DIAMETER = Inches.of(5.91);
    public static final Mass FUEL_WEIGHT = Pounds.of(0.474);
}
