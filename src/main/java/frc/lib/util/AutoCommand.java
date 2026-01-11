/*
 * Copyright (C) 2025 Windham Windup
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

// https://github.com/3015RangerRobotics/2024Public/blob/main/RobotCode2024/src/main/java/frc/robot/commands/auto/AutoCommand.java

package frc.lib.util;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;

public abstract class AutoCommand extends SequentialCommandGroup {
    public abstract List<Pose2d> getAllPathPoses();

    public abstract Pose2d getStartingPose();
}
