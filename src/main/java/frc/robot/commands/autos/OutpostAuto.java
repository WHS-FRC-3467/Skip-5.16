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
import frc.robot.subsystems.intakeLinear.IntakeLinear;
import frc.robot.subsystems.intakeRoller.IntakeRoller;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.tower.Tower;
import frc.robot.util.RobotSim;
import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Auto routine that utilizes AutoSegment command sequences to shoot a preload, collect FUEL from
 * the OUTPOST, and then shoot them. Strategy layer.
 */
public class OutpostAuto extends AutoRoutine {
    /**
     * Constructs an OutpostAuto routine that shoots preload, collects FUEL from the OUTPOST, and
     * then shoots the collected FUEL. Path selection is based on the starting position (LEFT,
     * CENTER, or RIGHT).
     *
     * @param drive the drive subsystem
     * @param intakeLinear the intake linear subsystem for deploying/retracting intake
     * @param intake the intake roller subsystem for collecting fuel
     * @param indexer the indexer subsystem for managing fuel flow
     * @param tower the tower subsystem for moving fuel to shooter
     * @param shooter the shooter superstructure for launching fuel
     * @param start the starting position on the field
     */
    public OutpostAuto(Drive drive, IntakeLinear intakeLinear, IntakeRoller intake, Indexer indexer,
        Tower tower,
        ShooterSuperstructure shooter, StartPosition start)
    {
        // Choose path names based on start position
        List<String> expectedPaths;
        switch (start) {
            case LEFT -> expectedPaths =
                List.of("PreloadShoot-Left", "Left-Preload-To-Outpost",
                    "Outpost-Shoot");
            case CENTER -> expectedPaths =
                List.of("PreloadShoot-Center", "Center-Preload-To-Outpost",
                    "Outpost-Shoot");
            case RIGHT -> expectedPaths =
                List.of("PreloadShoot-Right", "Right-Preload-To-Outpost",
                    "Outpost-Shoot");
            default -> expectedPaths = List.of();
        }

        // Load the named paths
        this.loadAllPaths(expectedPaths);

        // Defensive check: ensure we loaded exactly the expected number of paths and none are null
        if (pathPlannerPaths.size() == expectedPaths.size() && !pathPlannerPaths.contains(null)) {
            loadCommands(
                // Reset odometry
                AutoCommands.resetSimOdom(drive, pathPlannerPaths.get(0)),
                // Initialize intake
                AutoSegments.initializeIntake(intakeLinear, intake),
                // Take preload shot
                AutoSegments.makePreloadShot(drive, indexer, tower, shooter,
                    pathPlannerPaths.get(0)),
                // Go to the OUTPOST and intake FUEL
                AutoBuilder.followPath(pathPlannerPaths.get(1)),
                // Wait for FUEL to be dumped
                Commands.waitSeconds(3),
                Commands.either(
                    Commands.runOnce(() -> RobotSim.getInstance().getFuelSim().setHeldFuel(20)),
                    Commands.none(), RobotBase::isSimulation),
                // Drive to shooting location and shoot all FUEL
                AutoSegments.makeFullShot(drive, intakeLinear, indexer, tower, shooter,
                    pathPlannerPaths.get(2)));
        }
    }
}
