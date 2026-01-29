/* Copyright (C) 2026 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.AutoRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import static edu.wpi.first.units.Units.Seconds;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.pathplanner.lib.path.PathPlannerPath;

public class DepotAuto extends AutoRoutine {

    PathPlannerPath path1 = null;
    PathPlannerPath path2 = null;
    PathPlannerPath path3 = null;

    public DepotAuto(Drive drive, Intake intake, Indexer indexer,
        ShooterSuperstructure shooter, StartPosition start)
    {
        // Choose path names based on start position
        switch (start) {
            case LEFT -> this.loadAllPaths(List.of("PreloadShoot-Left", "Left-Near-Depot", "Through-Depot", "Depot-Shoot"));
            case CENTER -> this
                .loadAllPaths(List.of("PreloadShoot-Center", "Center-Near-Depot", "Through-Depot", "Depot-Shoot"));
            // We may get rid of right case as that location is far away from the depot
            case RIGHT -> this.loadAllPaths(List.of("PreloadShoot-Right", "Right-Near-Depot", "Through-Depot", "Depot-Shoot"));
        };

        // Load commands defensively
        if (!pathPlannerPaths.isEmpty() && pathPlannerPaths.get(0) != null) {
            loadCommands(
                // Reset odometry
                AutoCommands.resetOdom(drive, pathPlannerPaths.get(0)),
                // Take preload shot
                AutoSegments.makePreloadShot(drive, indexer, shooter, pathPlannerPaths.get(0)),
                // Go to the DEPOT and intake FUEL
                AutoSegments.driveAndIntake(drive, intake, pathPlannerPaths.get(1), pathPlannerPaths.get(2)),
                // Wait for last FUEL to enter intake
                Commands.waitTime(Seconds.of(0.5)),
                // Drive to shooting location and shoot all FUEL
                AutoSegments.makeFullShot(drive, indexer, shooter, pathPlannerPaths.get(3)));
        }
    }

    @Override
    public List<Pose2d> getAllPathPoses()
    {
        return Stream.of(path1.getPathPoses(), path2.getPathPoses())
            .flatMap(Collection::stream)
            .collect(Collectors.toList());
    }

    @Override
    public Pose2d getStartingPose()
    {
        return path1.getStartingHolonomicPose().get();
    }
}