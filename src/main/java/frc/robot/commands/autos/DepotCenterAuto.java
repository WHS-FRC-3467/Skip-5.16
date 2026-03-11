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

import com.pathplanner.lib.auto.AutoBuilder;

import frc.lib.util.AutoRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.IndexerSuperstructure;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.tower.Tower;

import java.util.List;

/**
 * Auto routine that utilizes AutoSegment command sequences to drive to the DEPOT, collect FUEL from
 * the DEPOT, and then shoot them. Strategy layer.
 */
public class DepotCenterAuto extends AutoRoutine {
    /**
     * Constructs a DepotAuto routine that drive to depot, collects from depot, and shoots collected
     * fuel. Path selection is based on the starting position (LEFT, or CENTER).
     *
     * @param drive the drive subsystem
     * @param intake the intake subsystem
     * @param indexer the indexer subsystem for managing fuel flow
     * @param tower the tower subsystem for moving fuel to shooter
     * @param shooter the shooter superstructure for launching fuel
     */
    public DepotCenterAuto(
            Drive drive,
            IntakeSuperstructure intake,
            IndexerSuperstructure indexer,
            Tower tower,
            ShooterSuperstructure shooter) {
        // Choose path names based on start position
        List<String> expectedPaths = List.of("DepotCenter1");

        // Load the named paths
        this.loadAllPaths(expectedPaths, false, false);

        // Defensive check: ensure we loaded exactly the expected number of paths and none are null
        if (pathPlannerPaths.size() == expectedPaths.size() && !pathPlannerPaths.contains(null)) {
            loadCommands(
                    // Reset odometry
                    AutoCommands.resetOdom(drive, pathPlannerPaths.get(0)),
                    // Drive to depot and start intake, then run through depot while intaking FUEL
                    AutoBuilder.followPath(pathPlannerPaths.get(0)),
                    // // Drive to shooting location and shoot all FUEL
                    AutoCommands.shootCommand(drive, intake, indexer, tower, shooter, 10.0));
        }
    }
}
