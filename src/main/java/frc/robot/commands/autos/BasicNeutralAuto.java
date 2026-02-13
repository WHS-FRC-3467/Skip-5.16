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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.AutoRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.tower.Tower;
import frc.robot.util.RobotSim;

public class BasicNeutralAuto extends AutoRoutine {

    public BasicNeutralAuto(Drive drive, IntakeSuperstructure intake,
        Indexer indexer, Tower tower, ShooterSuperstructure shooter, StartPosition start)
    {
        // Choose path names based on start position
        List<String> expectedPaths;
        switch (start) {
            case LEFT -> expectedPaths =
                List.of("BasicNeutral-Start-Left", "SweepNeutral-Trench-Left",
                    "UnderTrench-Shoot-Left",
                    "DepotSweep-From-Shot-Left", "Through-Depot", "Through-Depot-Reverse",
                    "UnderTrench-Run-Left");
            case CENTER -> expectedPaths =
                List.of(); // TODO: Strategy for center start position
            case RIGHT -> expectedPaths =
                List.of("BasicNeutral-Start-Left", "SweepNeutral-Trench-Left",
                    "UnderTrench-Shoot-Left",
                    "OutpostSweep-From-Shot-Right", "Outpost-Shoot", "UnderTrench-Run-Left");
            default -> expectedPaths = List.of();
        };

        // Load the named paths
        this.loadAllPaths(expectedPaths);

        // Keep track of which paths must be mirrored for RIGHT side autos
        this.setMirrorFlags(List.of(true, true, true, false, false, true), start);

        // Defensive check: ensure we loaded exactly the expected number of paths and none are null
        if (pathPlannerPaths.size() == expectedPaths.size() && !pathPlannerPaths.contains(null))

            loadCommands(
                // Reset odometry
                start == StartPosition.LEFT
                    ? AutoCommands.resetSimOdom(drive, pathPlannerPaths.get(0))
                    : AutoCommands.resetSimOdom(drive, pathPlannerPaths.get(0).mirrorPath()),
                // Initialize intake to starting position
                AutoCommands.initializeIntake(intake),
                // Drive to the neutral zone
                start == StartPosition.LEFT ? AutoBuilder.followPath(pathPlannerPaths.get(0))
                    : AutoBuilder.followPath(pathPlannerPaths.get(0).mirrorPath()),
                // Sweep neutral zone while intaking
                AutoSegments.driveAndIntake(intake,
                    start == StartPosition.LEFT ? AutoBuilder.followPath(pathPlannerPaths.get(1))
                        : AutoBuilder.followPath(pathPlannerPaths.get(1).mirrorPath()),
                    Seconds.of(0.0)),
                // Run back under the trench and shoot
                AutoSegments.makeFullShot(drive, intake, indexer, tower, shooter,
                    start == StartPosition.LEFT ? pathPlannerPaths.get(2)
                        : pathPlannerPaths.get(2).mirrorPath()),
                // Re-initialize intake for depot / outpost run
                AutoCommands.initializeIntake(intake),
                // Run to depot / outpost
                AutoBuilder.followPath(pathPlannerPaths.get(3)),
                // Sweep through depot while intaking OR wait for FUEL to be dumped
                start == StartPosition.LEFT
                    ? AutoSegments.driveAndIntake(intake,
                        AutoBuilder.followPath(pathPlannerPaths.get(4)), Seconds.of(0.0))
                    : Commands.sequence(
                        Commands.waitSeconds(3),
                        Commands.either(
                            Commands
                                .runOnce(() -> RobotSim.getInstance().getFuelSim().setHeldFuel(20)),
                            Commands.none(), RobotBase::isSimulation)),
                // Reverse back through depot / outpost to shooting location & shoot FUEL
                start == StartPosition.LEFT
                    ? AutoSegments.makePreloadShot(drive, indexer, tower, shooter,
                        pathPlannerPaths.get(5))
                    : AutoSegments.makePreloadShot(drive, indexer, tower, shooter,
                        pathPlannerPaths.get(4)),
                // Re-initialize intake for tele-op
                AutoCommands.initializeIntake(intake));
    }
}
