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
import frc.robot.subsystems.intakeRoller.IntakeRoller.State;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.tower.Tower;


/**
 * Class containing larger command units consisting of individual commands or small-group command
 * sequences (AutoCommands) strung together for use in creating full Autos. Command integration
 * layer.
 */
public class AutoSegments {

    /**
     * Drive to shooting location while spinning up shooter but not feeding game pieces. Once at
     * target position, with the shooter still spinning, verify subsystem process variables. Upon
     * confirmation of shooter-ready PVs, bring up the tower and indexer to begin shooting. Shoot
     * the PRELOADED FUEL for 1.5s. Bring down shooter, tower, and indexer to finish. If path
     * doesn't complete in 2.75s, attempt a shot anyway.
     * 
     * @param drive The Drive subsystem
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
                shooter.spinUpShooter()).withTimeout(2.75), // ~ +40% of max preload path time
            AutoCommands.shootFuel(indexer, tower, shooter, () -> true, 1.5));
    }

    /**
     * Drive to shooting location while spinning up shooter but not feeding game pieces. Once at
     * target position, with the shooter still spinning, verify subsystem process variables. Upon
     * confirmation of shooter-ready PVs, bring up the tower and indexer to begin shooting. Shoot
     * all FUEL for 5s. Bring down shooter, tower, and indexer to finish. If path doesn't complete
     * in 3.5s, attempt a shot anyway.
     * 
     * @param drive The Drive subsystem
     * @param indexer The Indexer subsystem
     * @param tower The Tower subsystem
     * @param shooter The ShooterSuperstructure subsystem
     * @param path The path to drive to the shooting location, the robot will shoot from the path's
     *        end pose
     * @param path The path to drive to the shooting location, the robot will shoot from the path's
     *        end pose
     */
    public static Command makeFullShot(Drive drive, IntakeLinear intakeLinear, Indexer indexer,
        Tower tower, ShooterSuperstructure shooter, PathPlannerPath path)
    {
        return Commands.sequence(
            new ParallelDeadlineGroup(
                AutoBuilder.followPath(path),
                shooter.spinUpShooter()).withTimeout(3.5), // ~ +40% of max preload path time
            new ParallelDeadlineGroup(
                AutoCommands.shootFuel(indexer, tower, shooter, () -> true, 5.0),
                AutoCommands.agitateHopper(intakeLinear, tower, indexer,
                    HopperAgitation.INTAKE_CYCLE)));
    }

    /**
     * Drive to the end of the drive path, extend the intake, and drive into the FUEL with rollers
     * running. Once the intaking path is complete, stop the intake. This AutoSegment only linearly
     * actuates the intake while the robot is at stationary positions. Non-blocking command.
     */
    public static Command driveAndIntake(Drive drive, IntakeLinear intakeLinear,
        IntakeRoller intakeRoller, PathPlannerPath drivePath, PathPlannerPath intakingPath,
        Time afterPathWait)
    {
        // Drive to near the intaking location, start up intake, and drive into the FUEL. Once the
        // intaking path is complete, stop the intake.
        return Commands.sequence(
            AutoBuilder.followPath(drivePath),
            AutoCommands.extendIntake(intakeLinear),
            new ParallelDeadlineGroup(
                AutoBuilder.followPath(intakingPath),
                intakeLinear.extend(), // Keep linear intake scheduled
                intakeRoller.holdStateUntilInterrupted(IntakeRoller.State.INTAKE)),
            Commands.waitSeconds(afterPathWait.in(Seconds)),
            // Spin down intake
            intakeLinear.retract(),
            intakeRoller.stop());
    }

    /**
     * A non-blocking command that initializes the intake by stopping the rollers and retracting the
     * linear stage. Timeout after 1.5s.
     * 
     * @param intakeLinear the linear intake subsystem
     * @param intakeRoller
     * @return a command that initializes the intake.
     */
    public static Command initializeIntake(IntakeLinear intakeLinear, IntakeRoller intakeRoller)
    {
        return Commands.sequence(
            intakeRoller.setStateCommand(State.STOP),
            AutoCommands.retractIntake(intakeLinear));
    }
}
