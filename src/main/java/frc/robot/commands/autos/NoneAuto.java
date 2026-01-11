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

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.AutoCommand;
import java.util.Collections;
import java.util.List;

public class NoneAuto extends AutoCommand {
    public NoneAuto()
    {
        addCommands(Commands.none());
    }

    @Override
    public List<Pose2d> getAllPathPoses()
    {
        return Collections.emptyList();
    }

    @Override
    public Pose2d getStartingPose()
    {
        return new Pose2d();
    }
}
