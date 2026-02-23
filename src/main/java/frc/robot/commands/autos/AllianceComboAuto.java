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

/* Autos that are on our Alliance side and collect fuel in two places. */
public class AllianceComboAuto extends AutoRoutine {
    /**
     * Constructs a AllianceComboAuto routine that collects from depot (LEFT, CENTER) or outpost
     * (RIGHT), and shoots collected fuel. Then collects from outpost (LEFT, CENTER) or depot
     * (RIGHT). Path selection is based on the starting position (LEFT, CENTER, or RIGHT).
     *
     * @param drive the drive subsystem
     * @param intake the intake subsystem
     * @param indexer the indexer subsystem for managing fuel flow
     * @param tower the tower subsystem for moving fuel to shooter
     * @param shooter the shooter superstructure for launching fuel
     * @param start the starting position on the field
     */
    public AllianceComboAuto(
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
                    // DEPOT FIRST
                    expectedPaths =
                            List.of(
                                    "StartLeft-NearDepot",
                                    "Through-Depot",
                                    "Depot-Shoot",
                                    "LeftShoot-To-Outpost",
                                    "Outpost-Shoot");
            case CENTER ->
                    // DEPOT FIRST
                    expectedPaths =
                            List.of(
                                    "StartCenter-NearDepot",
                                    "Through-Depot",
                                    "Depot-Shoot",
                                    "LeftShoot-To-Outpost",
                                    "Outpost-Shoot");
            case RIGHT ->
                    // OUTPOST FIRST
                    expectedPaths =
                            List.of(
                                    "StartRight-To-Outpost",
                                    "Outpost-Shoot",
                                    "RightShoot-NearDepot",
                                    "Through-Depot",
                                    "Depot-Shoot");
            default -> expectedPaths = List.of();
        }

        // Load the named paths
        this.loadAllPaths(expectedPaths);

        // Do not mirror as DEPOT is not symmetrical to OUTPOST
        this.setMirrorFlags(Collections.nCopies(expectedPaths.size(), false), start);

        // Defensive check: ensure we loaded exactly the expected number of paths and none are null
        if (pathPlannerPaths.size() == expectedPaths.size() && !pathPlannerPaths.contains(null)) {
            loadCommands(
                    // Reset odometry
                    AutoCommands.resetSimOdom(drive, pathPlannerPaths.get(0)),
                    // Initialize intake
                    intake.retractIntake().withTimeout(1.25),

                    // Drive to depot/outpost and collect fuel
                    (start != StartPosition.RIGHT
                            // LEFT, CENTER start: Run through DEPOT while intaking FUEL
                            ? Commands.sequence(
                                    AutoCommands.driveAndIntake(
                                            AutoBuilder.followPath(pathPlannerPaths.get(0)),
                                            intake,
                                            AutoBuilder.followPath(pathPlannerPaths.get(1)),
                                            Seconds.of(0.5)))
                            // RIGHT START: OUTPOST
                            : AutoCommands.driveAndCollectAtOutpost(
                                    AutoBuilder.followPath(pathPlannerPaths.get(0)))),
                    // Drive to shooting location and shoot all FUEL
                    AutoCommands.makeFullShot(
                            drive,
                            intake,
                            indexer,
                            tower,
                            shooter,
                            pathPlannerPaths.get((start == StartPosition.RIGHT ? 1 : 2))),
                    Commands.waitSeconds(0.2),
                    // Go to other FUEL location
                    Commands.sequence(
                            (start == StartPosition.RIGHT
                                    // RIGHT start: Run to DEPOT, then through it while intaking
                                    // FUEL
                                    ? AutoCommands.driveAndIntake(
                                            AutoBuilder.followPath(pathPlannerPaths.get(2)),
                                            intake,
                                            AutoBuilder.followPath(pathPlannerPaths.get(3)),
                                            Seconds.of(0.5))
                                    // LEFT start: go to outpost and wait 3 seconds
                                    : Commands.parallel(
                                            // Retract intake to press the robot against outpost
                                            // wall
                                            intake.retractIntake().withTimeout(1.25),
                                            AutoCommands.driveAndCollectAtOutpost(
                                                    AutoBuilder.followPath(
                                                            pathPlannerPaths.get(3))))),
                            // Drive to shooting location and shoot all FUEL
                            AutoCommands.makeFullShot(
                                    drive,
                                    intake,
                                    indexer,
                                    tower,
                                    shooter,
                                    pathPlannerPaths.get(4))),

                    // Re-initialize intake for tele-op
                    intake.retractIntake().withTimeout(1.25));
        }
    }
}
