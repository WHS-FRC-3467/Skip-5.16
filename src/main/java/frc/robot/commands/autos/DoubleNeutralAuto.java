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
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.AutoRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.IndexerSuperstructure;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.tower.Tower;
import java.util.Collections;
import java.util.List;

/**
 * Auto routine that utilizes AutoSegment command sequences to drive to the NEUTRAL ZONE and collect
 * FUEL, shoots from by the trench, and returns to the NEUTRAL ZONE to collect balls that bunched
 * near the walls. Strategy layer.
 */
public class DoubleNeutralAuto extends AutoRoutine {

    public DoubleNeutralAuto(
            Drive drive,
            IntakeSuperstructure intake,
            IndexerSuperstructure indexer,
            Tower tower,
            ShooterSuperstructure shooter,
            StartPosition start) {
        // Choose path names based on start position
        List<String> expectedPaths;
        switch (start) {
            case LEFT ->
                    expectedPaths =
                            List.of(
                                    "DoubleNeutral-Start-Left1",
                                    // Tune this so balls bunch up near the trench/hub
                                    "SweepDoubleNeutral-Trench-Left",
                                    "DoubleNeutral-UnderTrench-Shoot-Left",
                                    "DoubleNeutral-Start-Left2",
                                    "DoubleNeutral-HubSweep-Left",
                                    "Hub-Shoot-Left",
                                    "UnderTrench-Run-Left");
            case CENTER ->
                    expectedPaths =
                            List.of(); // Currently no DoubleNeutral Zone auto strategy for the
            // center
            // position
            case RIGHT ->
                    expectedPaths =
                            List.of(
                                    "DoubleNeutral-Start-Left1",
                                    "SweepDoubleNeutral-Trench-Left",
                                    "DoubleNeutral-UnderTrench-Shoot-Left",
                                    "DoubleNeutral-Start-Left2",
                                    "DoubleNeutral-HubSweep-Left",
                                    "Hub-Shoot-Left",
                                    "UnderTrench-Run-Left");
            default -> expectedPaths = List.of();
        }

        // Load the named paths
        this.loadAllPaths(expectedPaths);

        // Mirror all paths for RIGHT side autos
        this.setMirrorFlags(Collections.nCopies(expectedPaths.size(), true), start);

        // Defensive check: ensure we loaded exactly the expected number of paths and none are null
        if (pathPlannerPaths.size() == expectedPaths.size() && !pathPlannerPaths.contains(null))
            loadCommands(
                    // Reset odometry
                    start == StartPosition.LEFT
                            ? AutoCommands.resetSimOdom(drive, pathPlannerPaths.get(0))
                            : AutoCommands.resetSimOdom(
                                    drive, pathPlannerPaths.get(0).mirrorPath()),
                    // Initialize intake & hood to starting positions
                    intake.retractIntake().withTimeout(1.25),
                    shooter.setHoodAngle(Degrees.zero()).withTimeout(1.25),
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
                    // Run back under the trench and shoot
                    AutoCommands.makeFullShot(
                            drive,
                            intake,
                            indexer,
                            tower,
                            shooter,
                            start == StartPosition.LEFT
                                    ? pathPlannerPaths.get(2)
                                    : pathPlannerPaths.get(2).mirrorPath()),
                    Commands.waitSeconds(0.2),
                    // Lower shooter hood before going back under the trench
                    shooter.setHoodAngle(Degrees.zero()).withTimeout(1.25),
                    // Sweep neutral zone while intaking
                    AutoCommands.driveAndIntake(
                            // Drive to the neutral zone
                            start == StartPosition.LEFT
                                    ? AutoBuilder.followPath(pathPlannerPaths.get(3))
                                    : AutoBuilder.followPath(pathPlannerPaths.get(3).mirrorPath()),
                            intake,
                            start == StartPosition.LEFT
                                    ? AutoBuilder.followPath(pathPlannerPaths.get(4))
                                    : AutoBuilder.followPath(pathPlannerPaths.get(4).mirrorPath()),
                            Seconds.of(0.0)),
                    // Run back under the trench and shoot
                    AutoCommands.makeFullShot(
                            drive,
                            intake,
                            indexer,
                            tower,
                            shooter,
                            start == StartPosition.LEFT
                                    ? pathPlannerPaths.get(5)
                                    : pathPlannerPaths.get(5).mirrorPath()),
                    // Re-initialize intake for tele-op
                    intake.retractIntake().withTimeout(1.25));
    }
}
