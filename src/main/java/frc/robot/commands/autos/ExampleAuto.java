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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.AutoCommand;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

public class ExampleAuto extends AutoCommand {

    PathPlannerPath path1 = null;
    PathPlannerPath path2 = null;

    public ExampleAuto(Drive drive)
    {
        try {
            path1 = PathPlannerPath.fromPathFile("path1");
            path2 = PathPlannerPath.fromPathFile("path2");

            if (Robot.isSimulation() && !Logger.hasReplaySource()) {
                addCommands(AutoCommands.resetOdom(drive, path1));
            }

            addCommands(
                Commands.sequence(
                    AutoBuilder.followPath(path1),
                    Commands.waitSeconds(1),
                    AutoBuilder.followPath(path2)));

        } catch (Exception e) {
            DriverStation.reportError(
                "Path Failed to Load in " + this.getName() + " " + e.getMessage(),
                e.getStackTrace());
            throw new RuntimeException("Path Failed to Load in " + this.getName());
        }

    }

    @Override
    public List<Pose2d> getAllPathPoses()
    {
        return Stream.of(path1.getPathPoses(), path2.getPathPoses())
            .flatMap(Collection::stream)
            .collect(Collectors.toList());
    }

    @Override
    public Pose2d getStartingPose()
    {
        return path1.getStartingHolonomicPose().get();
    }
}
