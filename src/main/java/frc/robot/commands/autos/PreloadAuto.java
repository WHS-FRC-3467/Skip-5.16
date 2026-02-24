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

import static edu.wpi.first.units.Units.Degrees;

import frc.lib.util.AutoRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.IndexerSuperstructure;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.tower.Tower;
import java.util.List;

/** Auto routine that utilizes AutoCommands to shoot a preload. */
public class PreloadAuto extends AutoRoutine {

    public PreloadAuto(
            Drive drive,
            IntakeSuperstructure intake,
            IndexerSuperstructure indexer,
            Tower tower,
            ShooterSuperstructure shooter,
            StartPosition start) {
        // Choose path names based on start position
        List<String> expectedPaths;
        switch (start) {
            case LEFT -> expectedPaths = List.of("PreloadShoot-Left");
            case CENTER -> expectedPaths = List.of("PreloadShoot-Center");
            case RIGHT -> expectedPaths = List.of("PreloadShoot-Left");
            default -> expectedPaths = List.of();
        }

        // Load the named paths
        this.loadAllPaths(expectedPaths);

        // Mirror necessary paths when starting on RIGHT side so the dashboard shows correct poses
        this.setMirrorFlags(List.of(true), start);

        // Defensive check: ensure we loaded exactly the expected number of paths and none are null
        if (!pathPlannerPaths.isEmpty()
                && pathPlannerPaths.size() == expectedPaths.size()
                && !pathPlannerPaths.contains(null))
            loadCommands(
                    // Reset odometry
                    start == StartPosition.LEFT
                            ? AutoCommands.resetSimOdom(drive, pathPlannerPaths.get(0))
                            : AutoCommands.resetSimOdom(
                                    drive, pathPlannerPaths.get(0).mirrorPath()),
                    // Initialize intake & hood to starting positions
                    intake.retractIntake().withTimeout(1.25),
                    shooter.setHoodAngle(Degrees.zero()).withTimeout(1.25),
                    // Take preload shot
                    start == StartPosition.LEFT
                            ? AutoCommands.moveToShot(
                                    drive, intake, indexer, tower, shooter, pathPlannerPaths.get(0))
                            : AutoCommands.moveToShot(
                                    drive,
                                    intake,
                                    indexer,
                                    tower,
                                    shooter,
                                    pathPlannerPaths.get(0).mirrorPath()));
    }
}
