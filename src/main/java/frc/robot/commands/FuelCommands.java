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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.IndexerSuperstructure;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.tower.Tower;
import java.util.function.BooleanSupplier;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;

/**
 * Utility class for FUEL manipulation commands anticipated for use in teleop or auto that require
 * coordination of multiple subsystems.
 */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class FuelCommands {

    /**
     * Returns a command that feeds the tower until one of the laserCANs is tripped to increase
     * hopper capacity. Does nothing if a laserCAN is already tripped.
     *
     * @param indexer the indexer subsystem
     * @return a command that feeds the tower until a laserCAN is tripped
     */
    public static Command stageFuel(IndexerSuperstructure indexer, Tower tower) {
        return Commands.parallel(indexer.feed(), tower.feed())
                .until(tower.isStaged)
                .withName("StageFuel");
    }

    /**
     * Creates a command sequence that attempts to shoot fuel from the robot for duration. Spins up
     * the shooter, only pulls fuel through the feeder when ready (i.e. proper shooter state +
     * alignment), then stops indexer and tower after duration. Shooter remains spun-up. If shooting
     * is disrupted during duration because shooting readiness drops, attempt a flywheel/hood
     * adjustment and, if successful, re-commence shooting within the remaining window. Alignment
     * correction must be handled by external means. Unconditionally stops shot attempts after
     * duration.
     *
     * @param indexer the indexer subsystem
     * @param tower the tower subsystem
     * @param shooter the shooter superstructure
     * @param canShoot secondary check on whether the robot is properly aligned to the target,
     *     independent of whether the shooter is at the proper state
     * @param duration the approximate duration in seconds to run the shooting sequence
     * @return a command that shoots fuel and then stops the indexer / tower after the given
     *     duration
     */
    public static Command shootFuel(
            IndexerSuperstructure indexer,
            Tower tower,
            ShooterSuperstructure shooter,
            BooleanSupplier canShoot,
            double duration) {
        Command feed =
                Commands.parallel(indexer.shoot(), tower.shoot())
                        .until(() -> !canShoot.getAsBoolean());

        return shooter.prepareShot(Commands.waitUntil(canShoot).andThen(feed))
                .withTimeout(duration);
    }
}
