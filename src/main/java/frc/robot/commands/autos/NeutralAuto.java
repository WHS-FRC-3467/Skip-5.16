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

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import frc.lib.util.AutoRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.IndexerSuperstructure;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.tower.Tower;
import java.util.List;
import org.apache.commons.lang3.NotImplementedException;

public class NeutralAuto extends AutoRoutine {

    public NeutralAuto(
            Drive drive,
            IntakeSuperstructure intake,
            IndexerSuperstructure indexer,
            Tower tower,
            ShooterSuperstructure shooter,
            StartPosition start) {
        // Choose path names based on start position
        List<String> expectedPaths;
        switch (start) {
            case LEFT -> expectedPaths = List.of("NeutralStart", "Neutral1st", "Neutral2nd");
            case CENTER -> {
                throw new NotImplementedException("No Neutral Auto for Center Start");
            }
            case RIGHT -> expectedPaths = List.of("NeutralStart", "Neutral1st", "Neutral2nd");
            default -> {
                throw new IllegalArgumentException("Invalid Start Position");
            }
        }

        // Load the named paths
        this.loadAllPaths(expectedPaths);

        // Keep track of which paths must be mirrored for RIGHT side autos
        this.setMirrorFlags(List.of(true, true, true), start);

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
                    // Initialize intake and hood to starting positions for first sweep
                    AutoCommands.stowHood(shooter),
                    // Sweep neutral zone while intaking
                    AutoCommands.driveAndIntake(
                            // Drive to the neutral zone
                            start == StartPosition.LEFT
                                    ? AutoBuilder.followPath(pathPlannerPaths.get(0))
                                    : AutoBuilder.followPath(pathPlannerPaths.get(0).mirrorPath()),
                            intake,
                            start == StartPosition.LEFT
                                    ? AutoBuilder.followPath(pathPlannerPaths.get(1))
                                    : AutoBuilder.followPath(pathPlannerPaths.get(1).mirrorPath()),
                            Seconds.of(0.0)),
                    AutoCommands.shootCommand(
                            drive, intake, indexer, tower, shooter, MetersPerSecond.of(0.1)),
                    // Run back under the trench and shoot
                    // Initialize intake and hood to starting positions for teleop
                    AutoCommands.stowHood(shooter),
                    intake.retractIntake().asProxy().withTimeout(0.5),
                    // Drive to the neutral zone
                    start == StartPosition.LEFT
                            ? AutoBuilder.followPath(pathPlannerPaths.get(2))
                            : AutoBuilder.followPath(pathPlannerPaths.get(2).mirrorPath()),
                    AutoCommands.stowHood(shooter),
                    AutoCommands.shootCommand(
                            drive, intake, indexer, tower, shooter, MetersPerSecond.of(0.15)),
                    // Run back under the trench and shoot
                    // Initialize intake and hood to starting positions for teleop
                    AutoCommands.stowHood(shooter));
    }
}
