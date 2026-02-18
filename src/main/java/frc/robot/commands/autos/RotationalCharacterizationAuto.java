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
import java.util.List;

public class RotationalCharacterizationAuto extends AutoRoutine {

    public RotationalCharacterizationAuto(Drive drive) {
        List<String> expectedPaths = List.of("RotationalCharacterization");

        // Load the named paths
        this.loadAllPaths(expectedPaths);

        // No mirroring necessary - placeholder
        this.setMirrorFlags(List.of(false), StartPosition.CENTER);

        // Defensive check: ensure we loaded exactly the expected number of paths and none are null
        if (pathPlannerPaths.size() == expectedPaths.size() && !pathPlannerPaths.contains(null))
            loadCommands(
                    // Reset odometry
                    AutoCommands.resetSimOdom(drive, pathPlannerPaths.get(0)),
                    // Follow rotational path
                    AutoBuilder.followPath(pathPlannerPaths.get(0)));
    }
}
