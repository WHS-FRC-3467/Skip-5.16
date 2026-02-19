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
import frc.robot.commands.autos.StartPosition;
import java.util.ArrayList;
import java.util.List;

/**
 * Abstract base class for autonomous routines that use PathPlanner paths. Provides utilities for
 * loading paths, combining them into commands, and retrieving path poses for visualization and
 * starting position validation.
 */
public abstract class AutoRoutine extends SequentialCommandGroup {
    protected final List<PathPlannerPath> pathPlannerPaths = new ArrayList<>();
    // Per-path flags: when true, the corresponding path's poses will be mirrored
    // (useful for displaying the selected auto on the dashboard or for right-side starts).
    protected final List<Boolean> pathMirrorFlags = new ArrayList<>();

    /**
     * Loads all PathPlanner paths from the given path names.
     *
     * @param pathNames List of path file names to load
     */
    public void loadAllPaths(List<String> pathNames) {
        // Load paths into a temporary list so we can initialize mirror flags to the
        // same size and keep things consistent even if loading fails.
        List<PathPlannerPath> loaded = pathNames.stream().map(name -> loadPath(name)).toList();

        pathPlannerPaths.addAll(loaded);

        // Initialize mirror flags to false for each loaded path (default: no mirror).
        pathMirrorFlags.clear();
        for (int i = 0; i < loaded.size(); ++i) {
            pathMirrorFlags.add(false);
        }
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

        // Defensive: ensure mirror flags list matches the number of loaded paths. If it
        // doesn't, treat missing flags as `false` (no mirror).
        List<Pose2d> poses = new ArrayList<>();
        boolean mirror;
        // Loop through each path and check for whether the mirror should be applied.
        for (int i = 0; i < pathPlannerPaths.size(); ++i) {
            PathPlannerPath path = pathPlannerPaths.get(i);
            mirror = false;
            if (i < pathMirrorFlags.size() && Boolean.TRUE.equals(pathMirrorFlags.get(i))) {
                mirror = true;
            }

            // Apply mirror to poses in that path
            if (mirror) {
                poses.addAll(path.mirrorPath().getPathPoses());
            } else {
                poses.addAll(path.getPathPoses());
            }
        }

        return List.copyOf(poses);
    }

    /**
     * RIGHT side autos - Reset the mirror flags for applicable paths. If the provided list is
     * shorter than the number of loaded paths, remaining flags default to false. If longer, extra
     * values are ignored.
     *
     * @param mirrors a list of booleans indicating whether to mirror each corresponding path for
     *     RIGHT side autos.
     * @param start the starting side of the field - LEFT, CENTER, or RIGHT.
     */
    public void setMirrorFlags(List<Boolean> mirrors, StartPosition start) {
        if (mirrors == null) {
            return;
        }

        switch (start) {
            case RIGHT:
                pathMirrorFlags.clear();
                // Copy up to the number of loaded paths
                for (int i = 0; i < pathPlannerPaths.size(); ++i) {
                    if (i < mirrors.size()) {
                        pathMirrorFlags.add(mirrors.get(i));
                    } else {
                        pathMirrorFlags.add(false);
                    }
                }
                break;
            case CENTER:
                return;
            case LEFT:
                return;
            default:
                return;
        }
    }

    /**
     * Set a single path's mirror flag. If index is out of range this is a no-op.
     *
     * @param index path index
     * @param mirror whether to mirror that path
     */
    public void setShouldMirrorPath(int index, boolean mirror) {
        if (index < 0 || index >= pathPlannerPaths.size()) {
            return;
        }

        // Ensure the flags list is large enough
        while (pathMirrorFlags.size() < pathPlannerPaths.size()) {
            pathMirrorFlags.add(false);
        }

        pathMirrorFlags.set(index, mirror);
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
            return pathPlannerPaths.get(0).getPathPoses().get(0);
        }
    }
}
