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
import frc.robot.subsystems.indexer.IndexerSuperstructure;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.tower.Tower;
import java.util.List;

public class WilksAuto extends AutoRoutine {

    public WilksAuto(
            Drive drive,
            IntakeSuperstructure intake,
            IndexerSuperstructure indexer,
            Tower tower,
            ShooterSuperstructure shooter,
            StartPosition start) {
        // Choose path names based on start position
        List<String> expectedPaths;
        switch (start) {
            case LEFT -> expectedPaths = List.of("StartWilkTest", "WilkTest");
            case CENTER ->
                    expectedPaths =
                            List.of(); // Currently no Neutral Zone auto strategy for the center
            case RIGHT -> expectedPaths = List.of("WilkTest");
            default -> expectedPaths = List.of();
        }

        // Load the named paths
        this.loadAllPaths(expectedPaths);

        // Keep track of which paths must be mirrored for RIGHT side autos
        this.setMirrorFlags(List.of(true), start);

        // Defensive check: ensure we loaded exactly the expected number of paths and none are null
        if (!pathPlannerPaths.isEmpty()
                && pathPlannerPaths.size() == expectedPaths.size()
                && !pathPlannerPaths.contains(null))
            loadCommands(
                    // Reset odometry
                    // start == StartPosition.LEFT
                    //         ? AutoCommands.resetSimOdom(drive, pathPlannerPaths.get(0))
                    //         : AutoCommands.resetSimOdom(
                    //                 drive, pathPlannerPaths.get(0).mirrorPath()),
                    // // Initialize intake and hood to starting positions for first sweep
                    // AutoCommands.stowHood(shooter),
                    // // Sweep neutral zone while intaking
                    // AutoCommands.driveAndIntake(
                    //         // Drive to the neutral zone
                    //         start == StartPosition.LEFT
                    //                 ? AutoBuilder.followPath(pathPlannerPaths.get(0))
                    //                 :
                    // AutoBuilder.followPath(pathPlannerPaths.get(0).mirrorPath()),
                    //         intake,
                    //         start == StartPosition.LEFT
                    //                 ? AutoBuilder.followPath(pathPlannerPaths.get(1))
                    //                 :
                    // AutoBuilder.followPath(pathPlannerPaths.get(1).mirrorPath()),
                    //         Seconds.of(0.0)),
                    AutoCommands.shootCommand(drive, intake, indexer, tower, shooter),
                    // Run back under the trench and shoot
                    // Initialize intake and hood to starting positions for teleop
                    AutoCommands.stowHood(shooter));
    }
}
