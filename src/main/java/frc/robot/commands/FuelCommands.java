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
package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.indexer.IndexerSuperstructure;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.tower.Tower;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;

/**
 * Utility class for FUEL manipulation commands anticipated for use in teleop OR auto that require
 * coordination of multiple subsystems.
 */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class FuelCommands {

    /**
     * Creates a command sequence that attempts to shoot fuel from the robot for duration.
     * DYNAMICALLY CORRECTS shooter setpoints to ACTUAL CURRENT POSE (bringing it from rest or
     * trimming error associated with static spin up) and only pulls fuel through the feeder when
     * ready (i.e. proper shooter state + robot alignment), then stops indexer and tower after
     * duration. Shooter remains spun-up.
     *
     * <p>If shooting is disrupted during duration because shooting readiness drops or robot
     * misalignment is detected, attempt a flywheel/hood adjustment and, if successful, re-commence
     * shooting within the remaining window. See {@code ShooterSuperstructure.shootFuel()}.
     * Alignment correction is not attempted here. Unconditionally STOPS SHOTS attempts after
     * duration.
     *
     * @param indexer the indexer subsystem
     * @param tower the tower subsystem
     * @param shooter the shooter superstructure
     * @param duration the approximate duration in seconds to run the shooting sequence
     * @return a command that shoots fuel and then stops the indexer / tower after the given
     *     duration
     */
    public static Command prepareShot(
            IndexerSuperstructure indexer,
            Tower tower,
            IntakeSuperstructure intake,
            ShooterSuperstructure shooter,
            LinearVelocity retractSpeed,
            double duration) {
        return Commands.parallel(
                        shooter.spinUpShooter(),
                        intake.slowRetract(retractSpeed).asProxy(),
                        Commands.parallel(
                                        indexer.shoot()
                                                .withInterruptBehavior(
                                                        InterruptionBehavior.kCancelIncoming),
                                        tower.shoot())
                                .onlyWhile(
                                        shooter.readyToShoot.and(
                                                RobotState.getInstance().facingTarget))
                                .repeatedly())
                .withTimeout(duration)
                .finallyDo(
                        () -> {
                            CommandScheduler.getInstance()
                                    .schedule(shooter.setFlywheelSpeed(RotationsPerSecond.zero()));
                            CommandScheduler.getInstance()
                                    .schedule(shooter.setHoodAngle(Rotations.zero()));
                        });
    }
}
