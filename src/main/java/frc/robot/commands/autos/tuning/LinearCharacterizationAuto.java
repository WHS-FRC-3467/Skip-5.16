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
package frc.robot.commands.autos.tuning;

import com.pathplanner.lib.auto.AutoBuilder;
import frc.lib.util.AutoRoutine;
import frc.robot.commands.autos.AutoCommands;
import frc.robot.subsystems.drive.Drive;
import java.util.List;

public class LinearCharacterizationAuto extends AutoRoutine {

    public LinearCharacterizationAuto(Drive drive) {
        List<String> expectedPaths = List.of("LinearCharacterization");

        // Load the named paths
        this.loadAllPaths(expectedPaths, false);

        // Defensive check: ensure we loaded exactly the expected number of paths and none are null
        if (pathPlannerPaths.size() == expectedPaths.size() && !pathPlannerPaths.contains(null))
            loadCommands(
                    // Reset odometry
                    AutoCommands.resetSimOdom(drive, pathPlannerPaths.get(0)),
                    // Follow straight path
                    AutoBuilder.followPath(pathPlannerPaths.get(0)));
    }
}
