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

// https://github.com/3015RangerRobotics/2024Public/blob/main/RobotCode2024/src/main/java/frc/robot/commands/auto/AutoCommand.java

package frc.lib.util;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.ArrayList;
import java.util.List;

/**
 * Abstract base class for autonomous routines that use PathPlanner paths. Provides utilities for
 * loading paths, combining them into commands, and retrieving path poses for visualization and
 * starting position validation.
 */
public abstract class AutoRoutine extends SequentialCommandGroup {
    protected final List<PathPlannerPath> pathPlannerPaths = new ArrayList<>();

    /**
     * Loads all PathPlanner paths from the given path names.
     *
     * @param pathNames List of path file names to load
     * @param shouldMirror Whether to mirror the paths
     */
    public void loadAllPaths(List<String> pathNames, boolean shouldMirror) {
        // Load paths into a temporary list so we can initialize mirror flags to the
        // same size and keep things consistent even if loading fails.
        List<PathPlannerPath> loaded = pathNames.stream().map(name -> loadPath(name)).toList();
        if (shouldMirror) {
            for (var path : loaded) {
                path = path.mirrorPath();
            }
        }

        pathPlannerPaths.addAll(loaded);
    }

    private PathPlannerPath loadPath(String pathName) {
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            DriverStation.reportError(
                    "Failed to load PathPlanner path: " + pathName, e.getStackTrace());
            path = null;
        }

        return path;
    }

    /**
     * Adds commands to the sequential command group if all paths loaded successfully.
     *
     * @param commands Commands to add to the auto routine
     */
    public void loadCommands(Command... commands) {
        if (!pathPlannerPaths.contains(null)) {
            this.addCommands(commands);
        } else {
            DriverStation.reportWarning("Skipping auto due to missing path(s).", false);
        }
    }

    /**
     * Gets all poses from all loaded paths, mirroring from left to right side within an alliance,
     * as necessary.
     *
     * @return List of all path poses, or empty list if any paths failed to load
     */
    public List<Pose2d> getAllPathPoses() {
        // If any path failed to load, return an empty list to indicate problem to caller.
        if (pathPlannerPaths.contains(null)) {
            return List.of();
        }

        List<Pose2d> poses = new ArrayList<>();
        // Loop through each path and check for whether the mirror should be applied.
        for (int i = 0; i < pathPlannerPaths.size(); ++i) {
            PathPlannerPath path = pathPlannerPaths.get(i);

            poses.addAll(path.getPathPoses());
        }

        return List.copyOf(poses);
    }

    /**
     * Gets the starting pose of the first path.
     *
     * @return Starting pose of the first path, or zero pose if paths failed to load
     */
    public Pose2d getStartingPose() {
        if (pathPlannerPaths.contains(null)) {
            return new Pose2d();
        } else {
            var firstPath = pathPlannerPaths.get(0);
            return new Pose2d(
                    firstPath.getPathPoses().get(0).getTranslation(),
                    firstPath.getIdealStartingState().rotation());
        }
    }
}
