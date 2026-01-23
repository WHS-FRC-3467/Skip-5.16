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

import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;

public class PreloadShootAuto extends AutoCommands {

    private PathPlannerPath driveToShot = null;

    public PreloadShootAuto(Drive drive, Indexer indexer)
    {
        try {
            // TODO: actual path -- deploy\pathplanner
            driveToShot = PathPlannerPath.fromPathFile("PreloadShootAuto");


            if (Robot.isSimulation() && !Logger.hasReplaySource()) {
                addCommands(AutoCommands.resetOdom(drive, driveToShot));
            }

            addCommands(Commands.sequence(

                // Drive to shooting location while spinning up shooter
                Commands.parallel(
                    AutoBuilder.followPath(driveToShot))), // TODO: Add shooter/tower spin-up e.g.
                                                           // shooter.spinUpCommand()
                Commands.waitUntil(() -> true)); // TODO:
        } catch (Exception e) {
        }
    }
}

