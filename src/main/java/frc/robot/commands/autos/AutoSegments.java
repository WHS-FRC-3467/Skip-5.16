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
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intakeLinear.IntakeLinear;
import frc.robot.subsystems.intakeRoller.IntakeRoller;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.tower.Tower;

/**
 * Class containing larger command units consisting of individual commands or small-group command
 * sequences (AutoCommands) strung together for use in creating full Autos. Command integration
 * layer.
 */
public class AutoSegments {

    /**
     * Follow a path and then shoot the preload for a fixed duration. Drive to shooting location
     * while spinning up shooter but not indexing. Once at position, with the shooter still
     * spinning, bring up the indexer to begin shooting. Shoot the preload for a short, fixed period
     * of time. Bring down indexer to end -- shooter will idle at speed. If path doesn't complete in
     * 2.75s, attempt a shot anyway.
     * 
     * @param drive The Drive subsystem
     * @param intakeLinear The IntakeLinear subsystem
     * @param indexer The Indexer subsystem
     * @param tower The Tower subsystem
     * @param shooter The ShooterSuperstructure subsystem
     * @param path The path to drive to the shooting location, the robot will shoot from the path's
     *        end pose
     */
    public static Command makePreloadShot(Drive drive, IntakeLinear intakeLinear, Indexer indexer,
        Tower tower,
        ShooterSuperstructure shooter, PathPlannerPath path)
    {
        return Commands.sequence(
            new ParallelDeadlineGroup(
                AutoBuilder.followPath(path),
                shooter.spinUpShooter()).withTimeout(2.75),
            AutoCommands.shootFuel(intakeLinear, indexer, tower, shooter, 1));
    }



    /**
     * Follow a path to shooting location while spinning up shooter but not indexing. Once at
     * position, with the shooter still spinning, bring up the indexer to begin shooting. Shoot all
     * FUEL for up to 3s. Bring down indexer to end -- shooter will idle at speed. If path doesn't
     * complete in 3.5s, attempt a shot anyway.
     * 
     * @param drive The Drive subsystem
     * @param intakeLinear The IntakeLinear subsystem
     * @param indexer The Indexer subsystem
     * @param tower The Tower subsystem
     * @param shooter The ShooterSuperstructure subsystem
     * @param path The path to drive to the shooting location, the robot will shoot from the path's
     *        end pose
     */
    public static Command makeFullShot(Drive drive, IntakeLinear intakeLinear, Indexer indexer,
        Tower tower,
        ShooterSuperstructure shooter, PathPlannerPath path)
    {
        return Commands.sequence(
            new ParallelDeadlineGroup(
                AutoBuilder.followPath(path),
                shooter.spinUpShooter()).withTimeout(3.5),
            AutoCommands.shootFuel(intakeLinear, indexer, tower, shooter, 3));
    }


    /**
     * Returns a Command to Follow a path and collect FUEL in AUTO
     * 
     * @param drive The Drive subsystem
     * @param intakeLinear The IntakeLinear subsystem
     * @param intakeRoller The IntakeRoller subsystem
     * @param drivePath The path to drive to the intaking location
     * @param intakingPath The path to drive while intaking FUEL
     * @param afterPathWait The time to wait after the intaking path is complete before stopping the
     *        intake
     */
    public static Command driveAndIntake(Drive drive, IntakeLinear intakeLinear,
        IntakeRoller intakeRoller, PathPlannerPath drivePath, PathPlannerPath intakingPath,
        Time afterPathWait)
    {
        // Drive to near the intaking location, start up intake, and drive into the FUEL. Once the
        // intaking path is complete, stop the intake.
        return Commands.sequence(
            AutoBuilder.followPath(drivePath),
            new ParallelDeadlineGroup(
                AutoBuilder.followPath(intakingPath),
                intakeLinear.extend(),
                intakeRoller.runIntake(IntakeRoller.State.INTAKE)),
            // TODO: Tune after-path wait time to ensure all FUEL is intaken
            Commands.waitSeconds(afterPathWait.in(Seconds)),
            // End intaking
            intakeLinear.retract(),
            intakeRoller.stop());
    }
}
