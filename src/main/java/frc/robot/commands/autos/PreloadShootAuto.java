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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.State;
import frc.robot.subsystems.turret.ShooterSuperstructure;
import frc.robot.RobotState;

/*
 * Auto moves robot to a safe shooting position and then shoots all preloaded fuel.
 */
public class PreloadShootAuto extends AutoCommands {

    private PathPlannerPath driveToShot;

    public PreloadShootAuto(Drive drive, Indexer indexer, ShooterSuperstructure shooter)
    {
        try {
            RobotState robotState = RobotState.getInstance();
            driveToShot = PathPlannerPath.fromPathFile("PreloadShootAuto"); // TODO: actual path
            // Margin of error on PathPlanner ending pose for determining indexer bring-up
            Pose2d shotPose = driveToShot.getPathPoses().get(driveToShot.getPathPoses().size() - 1);
            Trigger atShotPose = new Trigger(
                () -> (robotState.getEstimatedPose().getTranslation()
                    .getDistance(shotPose.getTranslation())) < 0.15); // TODO: acceptance radius

            if (Robot.isSimulation() && !Logger.hasReplaySource()) {
                addCommands(AutoCommands.resetOdom(drive, driveToShot));
            }

            // Drive to shooting location while spinning up shooter but not feeding. Don't run
            // indexer until within shotPose acceptance radius. Once in range, spin up indexer to
            // begin feeding & shooting. Shoot preload over timeout. Fallback: if path finding fails
            // within 5s, attempt shot anyway and move on
            addCommands(Commands.sequence(
                Commands.parallel(
                    new ParallelDeadlineGroup(AutoBuilder.followPath(driveToShot),
                        shooter.prepareShot(Commands.none())).withTimeout(5.0)),
                shooter.prepareShot(indexer.holdStateUntilInterrupted(State.PULL))
                    .onlyWhile(atShotPose)
                    .withTimeout(1.0))); // TODO: beambreak feedback or confirmed bps for timer
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PreloadShootAuto: " + e.getMessage(),
                e.getStackTrace());
        }
    }
}
