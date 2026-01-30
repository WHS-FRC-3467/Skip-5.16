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

import java.util.List;
import frc.lib.util.AutoRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intakeLinear.IntakeLinear;
import frc.robot.subsystems.intakeRoller.IntakeRoller;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.tower.Tower;

// Auto routine that utilizes AutoSegment command sequences to shoot a preload, collect pieces from
// the neutral zone, and then shoot them. Strategy layer.
public class PreloadNeutralAuto extends AutoRoutine {

    public PreloadNeutralAuto(Drive drive, IntakeLinear intakeLinear, IntakeRoller intakeRoller,
        Tower tower, Indexer indexer, ShooterSuperstructure shooter, StartPosition start)
    {
        // Choose path names based on start position
        switch (start) {
            case LEFT -> this.loadAllPaths(List.of("PreloadShoot-Left", "placeholder"));
            case CENTER -> this
                .loadAllPaths(List.of("PreloadShoot-Center", "1SweepNeutral-Bump-Center"));
            case RIGHT -> this.loadAllPaths(List.of("PreloadShoot-Right", "placeholder"));
        };

        // Load commands defensively
        if (!pathPlannerPaths.isEmpty() && pathPlannerPaths.get(0) != null) {
            loadCommands(
                AutoCommands.resetSimOdom(drive, pathPlannerPaths.get(0)),
                AutoSegments.makePreloadShot(drive, tower, indexer, shooter,
                    pathPlannerPaths.get(0)));
            // AutoSegments.makeNeutralRun(...);
        }
    }
}
