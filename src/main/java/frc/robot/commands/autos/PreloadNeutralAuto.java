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

package frc.robot.commands.autos;

import java.util.ArrayList;
import java.util.List;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.util.AutoCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.ShooterSuperstructure;

// Auto routine that utilizes AutoSegment commands sequences to shoot a preload, collect pieces from
// the neutral zone, and then shoot them
public class PreloadNeutralAuto extends AutoCommand {
    private final List<String> pathNames;
    private final List<PathPlannerPath> pathsUsed = new ArrayList<>();

    public PreloadNeutralAuto(Drive drive, Intake intake, Indexer indexer,
        ShooterSuperstructure shooter, StartPosition start)
    {
        // Choose path names based on start position
        pathNames = switch (start) {
            case LEFT -> List.of("PreloadShoot-Left", "placeholder");
            case CENTER -> List.of("PreloadShoot-Center", "1SweepNeutral-Bump-Center");
            case RIGHT -> List.of("PreloadShoot-Right", "placeholder");
        };

        // Load PathPlanner paths based on path names
        loadPaths();

        // Execute auto if all paths successfully loaded. If not, do nothing.
        if (!pathsUsed.contains(null)) {
            // Chain AutoSegments here
            addCommands(
                AutoSegments.makePreloadShot(drive, indexer, shooter, pathNames.get(0))
            // Autosegments.makeNeutralRun()
            );
        } else {
            DriverStation.reportWarning("Skipping PreloadNeutralAuto due to missing path(s).",
                null);
        }

    }

    @Override
    public List<Pose2d> getAllPathPoses()
    {
        return pathsUsed.stream().flatMap(path -> path.getPathPoses().stream()).toList();
    }

    @Override
    public Pose2d getStartingPose()
    {
        return pathsUsed.get(0).getStartingHolonomicPose().get();
    }

    // Load the PathPlanner paths based on determined path names
    private void loadPaths()
    {
        for (String name : pathNames) {
            try {
                pathsUsed.add(PathPlannerPath.fromPathFile(name));
            } catch (Exception e) {
                DriverStation.reportError("Failed to load PathPlanner path: " + name,
                    e.getStackTrace());
                pathsUsed.add(null);
            }
        }
    }
}
