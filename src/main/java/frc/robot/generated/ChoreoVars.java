package frc.robot.generated;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

/**
 * Generated file containing variables defined in Choreo.
 * DO NOT MODIFY THIS FILE YOURSELF; instead, change these values
 * in the Choreo GUI.
 */
public final class ChoreoVars {
    public static final Distance Neutral2ML_Safe_X1 = Units.Meters.of(5.95);
    public static final Distance Neutral2ML_Safe_X3 = Units.Meters.of(7.8);
    public static final Distance Neutral2ML_Safe_Y = Units.Meters.of(4.5);

    public static final class Poses {
        public static final Pose2d BumpEntrance = new Pose2d(5.665, 5.793, Rotation2d.fromRadians(6.283));
        public static final Pose2d BumpExit = new Pose2d(3.468, 5.793, Rotation2d.fromRadians(6.283));
        public static final Pose2d NeutralShoot = new Pose2d(3.125, 7.309, Rotation2d.fromRadians(-1.154));
        public static final Pose2d TunnelEntrance = new Pose2d(4, 7.399, Rotation2d.fromRadians(-1.571));
        public static final Pose2d TunnelExit = new Pose2d(5.65, 7.399, Rotation2d.fromRadians(-1.571));

        private Poses() {}
    }

    private ChoreoVars() {}
}