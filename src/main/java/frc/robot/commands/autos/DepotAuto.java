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

import frc.lib.util.AutoRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.tower.Tower;
import static edu.wpi.first.units.Units.Seconds;
import java.util.Collections;
import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Auto routine that utilizes AutoSegment command sequences to shoot a preload, collect FUEL from
 * the DEPOT, and then shoot them. Strategy layer.
 */
public class DepotAuto extends AutoRoutine {
    /**
     * Constructs a DepotAuto routine that shoots preload, collects from depot, and shoots collected
     * fuel. Path selection is based on the starting position (LEFT, CENTER, or RIGHT).
     *
     * @param drive the drive subsystem
     * @param intake the intake subsystem
     * @param indexer the indexer subsystem for managing fuel flow
     * @param tower the tower subsystem for moving fuel to shooter
     * @param shooter the shooter superstructure for launching fuel
     * @param start the starting position on the field
     */
    public DepotAuto(Drive drive, IntakeSuperstructure intake, Indexer indexer,
        Tower tower,
        ShooterSuperstructure shooter, StartPosition start)
    {
        // Choose path names based on start position
        List<String> expectedPaths;
        switch (start) {
            case LEFT -> expectedPaths =
                List.of("PreloadShoot-Left", "Left-Near-Depot", "Through-Depot",
                    "Depot-Shoot");
            case CENTER -> expectedPaths =
                List.of("PreloadShoot-Center", "Center-Near-Depot",
                    "Through-Depot", "Depot-Shoot");
            case RIGHT -> expectedPaths =
                List.of("PreloadShoot-Right", "Right-Near-Depot",
                    "Through-Depot", "Depot-Shoot");
            default -> expectedPaths = List.of();
        }

        // Load the named paths
        this.loadAllPaths(expectedPaths);

        // Mirror necessary paths when starting on RIGHT side so the dashboard shows correct poses
        this.setMirrorFlags(Collections.nCopies(expectedPaths.size(), false), start);

        // Defensive check: ensure we loaded exactly the expected number of paths and none are null
        if (pathPlannerPaths.size() == expectedPaths.size() && !pathPlannerPaths.contains(null)) {
            loadCommands(
                // Reset odometry
                AutoCommands.resetSimOdom(drive, pathPlannerPaths.get(0)),
                // Initialize intake
                AutoSegments.initializeIntake(intake),
                // Take preload shot
                AutoSegments.makePreloadShot(drive, indexer, tower, shooter,
                    pathPlannerPaths.get(0)),
                // Drive to depot
                AutoBuilder.followPath(pathPlannerPaths.get(1)),
                // Pre-deploy intake before driving through depot
                AutoCommands.extendIntake(intake),
                Commands.waitSeconds(0.25),
                // Run through depot while intaking FUEL
                AutoSegments.driveAndIntake(intake,
                    AutoBuilder.followPath(pathPlannerPaths.get(2)), Seconds.of(0.5)),
                // Drive to shooting location and shoot all FUEL
                AutoSegments.makeFullShot(drive, intake, indexer, tower, shooter,
                    pathPlannerPaths.get(3)));
        }
    }
}
