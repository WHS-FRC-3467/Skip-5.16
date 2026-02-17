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

import java.util.List;
import frc.lib.util.AutoRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.IndexerSuperstructure;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.tower.Tower;

/**
 * Auto routine that utilizes AutoSegment command sequences to shoot a preload, collect pieces from
 * the neutral zone, and then shoot them. Strategy layer.
 */
public class PreloadAuto extends AutoRoutine {

    public PreloadAuto(Drive drive, IntakeSuperstructure intake,
        IndexerSuperstructure indexer, Tower tower,
        ShooterSuperstructure shooter, StartPosition start) {
        // Choose path names based on start position
        List<String> expectedPaths;
        switch (start) {
            case LEFT -> expectedPaths =
                List.of("PreloadShoot-Left");
            case CENTER -> expectedPaths =
                List.of("PreloadShoot-Center");
            case RIGHT -> expectedPaths =
                List.of("PreloadShoot-Right");
            default -> expectedPaths = List.of();
        };

        // Load the named paths
        this.loadAllPaths(expectedPaths);

        // Mirror necessary paths when starting on RIGHT side so the dashboard shows correct poses
        this.setMirrorFlags(List.of(false), start);

        // Defensive check: ensure we loaded exactly the expected number of paths and none are null
        if (pathPlannerPaths.size() == expectedPaths.size() && !pathPlannerPaths.contains(null))
            loadCommands(
                // Reset odometry
                AutoCommands.resetSimOdom(drive, pathPlannerPaths.get(0)),
                // Initialize intake
                intake.retractIntake(),
                // Take preload shot
                AutoCommands.makePreloadShot(drive, indexer, tower, shooter,
                    pathPlannerPaths.get(0)));
    }
}
