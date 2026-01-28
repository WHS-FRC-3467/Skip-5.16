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
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.State;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.ShooterSuperstructure;

// Class containing command sequences that can be chained together in different configurations for
// different autos
public class AutoSegments {

    // Follow a path and shoot preload
    public static Command makePreloadShot(Drive drive, Indexer indexer,
        ShooterSuperstructure shooter, String pathName)
    {
        PathPlannerPath driveToShot;
        try {
            driveToShot = PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PreloadShootAuto: " + e.getMessage(),
                e.getStackTrace());
            return Commands.none();
        }
        // Drive to shooting location while spinning up shooter but not indexing. Once at
        // position, with the shooter still spinning, bring up the indexer to begin shooting.
        // Shoot all preload. Bring down indexer to end -- shooter will idle at speed. If path
        // doesn't complete in 1.2x path time, attempt a shot anyway.
        return Commands.sequence(
            new ParallelDeadlineGroup(
                AutoBuilder.followPath(driveToShot),
                shooter.spinUpShooter()).withTimeout(2.5),
            new ParallelDeadlineGroup(
                Commands.waitSeconds(1.0), // TODO: beam-break feedback or confirm bps for timer
                shooter.spinUpShooter(),
                indexer.holdStateUntilInterrupted(State.PULL)
                    .onlyWhile(shooter.readyToShoot())),
            indexer.holdStateUntilInterrupted(State.STOP));
    }

    // Make a run for the neutral zone TODO: logic - placeholders for now
    public static Command makeNeutralRun(Drive drive, Intake intake, String pathName)
    {
        PathPlannerPath driveToNeutral;
        try {
            driveToNeutral = PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PreloadShootAuto: " + e.getMessage(),
                e.getStackTrace());
            return Commands.none();
        }
        return Commands.none();
    }
}
