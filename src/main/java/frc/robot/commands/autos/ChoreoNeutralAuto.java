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

public class ChoreoNeutralAuto extends AutoRoutine {

    public ChoreoNeutralAuto(
            Drive drive,
            IntakeSuperstructure intake,
            IndexerSuperstructure indexer,
            Tower tower,
            ShooterSuperstructure shooter,
            boolean shouldMirror,
            boolean isSafe) {
        // Choose path names based on start position
        List<String> expectedPaths;
        if (isSafe) {
            expectedPaths = List.of("NeutralSafe1", "Neutral2");
        } else {
            expectedPaths = List.of("Neutral1", "Neutral2");
        }

        // Load the named paths
        this.loadAllPaths(expectedPaths, shouldMirror, true);

        // Defensive check: ensure we loaded exactly the expected number of paths and none are null
        if (!pathPlannerPaths.isEmpty()
                && pathPlannerPaths.size() == expectedPaths.size()
                && !pathPlannerPaths.contains(null))
            loadCommands(
                    AutoCommands.resetSimOdom(drive, pathPlannerPaths.get(0)),
                    // Sweep neutral zone while intaking
                    AutoBuilder.followPath(pathPlannerPaths.get(0)),
                    // AutoCommands.shootCommand(
                    //         drive, intake, indexer, tower, shooter, MetersPerSecond.of(0.1),
                    // 3.5),
                    // Run back under the trench and shoot
                    // Initialize intake and hood to starting positions for teleop
                    // AutoCommands.stowHood(shooter),
                    // intake.retractIntake().asProxy().withTimeout(0.5),
                    // Drive to the neutral zone
                    AutoBuilder.followPath(pathPlannerPaths.get(1)));
        // AutoCommands.shootCommand(
        //         drive, intake, indexer, tower, shooter, MetersPerSecond.of(0.1), 10));
    }
}
