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

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

import java.util.Set;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static final boolean tuningMode = true;
    public static final boolean disableHAL = false;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,
        /** Running a physics simulator. */
        SIM,
        /** Replaying from a log file. */
        REPLAY
    }

    public static final class PathConstants {}

    public static final Distance STARTING_POSE_DRIVE_TOLERANCE = Inches.of(3.0); // For auto
    public static final Angle STARTING_POSE_ROT_TOLERANCE_DEGREES = Degrees.of(5.0);

    public static final Distance FULL_ROBOT_WIDTH = Inches.of(27.0 + 3.25);
    public static final Distance FULL_ROBOT_LENGTH = Inches.of(27.0 + 3.25);
    public static final Distance BUMPER_HEIGHT = Inches.of(4.0);
    // public static final Set<Integer> FILTERED_TAGS = Set.of(1, 12, 22, 23, 28, 17, 7, 6);
    public static final Set<Integer> FILTERED_TAGS = Set.of();
}
