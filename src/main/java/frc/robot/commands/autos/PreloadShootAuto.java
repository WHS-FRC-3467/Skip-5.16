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

import static edu.wpi.first.units.Units.RadiansPerSecond;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.State;
import frc.robot.subsystems.turret.ShooterSuperstructure;

/*
 * Auto moves robot to a safe shooting position and then shoots all preloaded fuel.
 */
public class PreloadShootAuto extends AutoCommands {

    private PathPlannerPath driveToShot;

    public PreloadShootAuto(Drive drive, Indexer indexer, ShooterSuperstructure shooter,
        StartPosition start)
    {
        try {
            String pathName = switch (start) {
                case LEFT -> "PreloadShoot-Left";
                case CENTER -> "PreloadShoot-Center";
                case RIGHT -> "PreloadShoot-Right";
            };
            driveToShot = PathPlannerPath.fromPathFile(pathName);

            if (Robot.isSimulation() && !Logger.hasReplaySource()) {
                addCommands(AutoCommands.resetOdom(drive, driveToShot));
            }
            // Drive to shooting location while spinning up shooter but not indexing. Once at
            // position, with the shooter still spinning, bring up the indexer to begin shooting.
            // Shoot all preload. Bring down shooter and indexer to end. If path doesn't complete in
            // 1.2x path time, attempt a shot anyway.
            addCommands(Commands.sequence(
                new ParallelDeadlineGroup(
                    AutoBuilder.followPath(driveToShot),
                    shooter.spinUpShooter()).withTimeout(2.5),
                new ParallelDeadlineGroup(
                    Commands.waitSeconds(1.0), // TODO: beam-break feedback or confirm bps for timer
                    shooter.spinUpShooter(),
                    indexer.holdStateUntilInterrupted(State.PULL)
                        .onlyWhile(shooter.readyToShoot())),
                shooter.setFlyWheelSpeed(RadiansPerSecond.zero()),
                indexer.holdStateUntilInterrupted(State.STOP)));

        } catch (Exception e) {
            DriverStation.reportError("Failed to load PreloadShootAuto: " + e.getMessage(),
                e.getStackTrace());
        }
    }
}
