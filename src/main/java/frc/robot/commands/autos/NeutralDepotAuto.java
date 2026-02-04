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

import static edu.wpi.first.units.Units.Seconds;
import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import frc.lib.util.AutoRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intakeLinear.IntakeLinear;
import frc.robot.subsystems.intakeRoller.IntakeRoller;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.tower.Tower;

public class NeutralDepotAuto extends AutoRoutine {

    public NeutralDepotAuto(Drive drive, IntakeLinear intakeLinear, IntakeRoller intakeRoller,
        Indexer indexer, Tower tower, ShooterSuperstructure shooter, StartPosition start)
    {
        // Choose path names based on start position
        List<String> expectedPaths;
        switch (start) {
            case LEFT -> expectedPaths =
                List.of("DepotSweep-Left", "UnderTrench-Run-Left", "SweepNeutral-Trench-Left",
                    "UnderTrench-Shoot-Left",
                    "DepotSweep-From-Shot-Left", "Through-Depot", "Through-Depot-Reverse");
            case CENTER -> expectedPaths =
                List.of(); // Depot-center auto not yet implemented
            case RIGHT -> expectedPaths = List.of(); // Depot-right auto not yet implemented
            default -> expectedPaths = List.of();
        };

        // Load the named paths
        this.loadAllPaths(expectedPaths);

        // Defensive check: ensure we loaded exactly the expected number of paths and none are null
        if (pathPlannerPaths.size() == expectedPaths.size() && !pathPlannerPaths.contains(null))

            loadCommands(
                // Reset odometry
                AutoCommands.resetSimOdom(drive, pathPlannerPaths.get(0)),
                // Initialize intake to starting position
                AutoSegments.initializeIntake(intakeLinear, intakeRoller),
                // Drive to staging location and then to the neutral zone
                AutoBuilder.followPath(pathPlannerPaths.get(0))
                    .andThen(AutoBuilder.followPath(pathPlannerPaths.get(1))),
                // Sweep neutral zone while intaking
                AutoSegments.driveAndIntake(intakeLinear, intakeRoller,
                    AutoBuilder.followPath(pathPlannerPaths.get(2)), Seconds.of(0.0)),
                // Run back under the trench and shoot
                AutoSegments.makeFullShot(drive, intakeLinear, indexer, tower, shooter,
                    pathPlannerPaths.get(3)),
                // Re-initialize intake for depot run
                AutoSegments.initializeIntake(intakeLinear, intakeRoller),
                // Run to depot
                AutoBuilder.followPath(pathPlannerPaths.get(4)),
                // Sweep through depot while intaking
                AutoSegments.driveAndIntake(intakeLinear, intakeRoller,
                    AutoBuilder.followPath(pathPlannerPaths.get(5)), Seconds.of(0.0)),
                // Reverse back through depot to shooting location & shoot depot fuel
                AutoSegments.makeFullShot(drive, intakeLinear, indexer, tower, shooter,
                    pathPlannerPaths.get(6)),
                // Re-initialize intake for tele-op
                AutoSegments.initializeIntake(intakeLinear, intakeRoller));
    }
}


