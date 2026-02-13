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
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.indexer.Indexer;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;

/**
 * Utility class for FUEL manipulation commands anticipated for use in teleop that require
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
    public static Command stageFuel(Indexer indexer, Tower tower)
    {
        return Commands.parallel(
            indexer.holdStateUntilInterrupted(Indexer.State.IDLE),
            tower.holdStateUntilInterrupted(Tower.State.IDLE))
            .until(tower.isStaged())
            .withName("StageFuel");
    }
}
