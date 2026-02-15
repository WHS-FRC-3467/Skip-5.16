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
package frc.robot.subsystems.indexer;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;

/**
 * Constants class for creating the IndexerSuperstructure subsystem. Provides factory method to
 * construct the complete indexer with the floor and centering mechanisms.
 */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class IndexerSuperstructureConstants {
    /**
     * Creates and configures a complete IndexerSuperstructure subsystem. Combines floor centering
     * flywheel mechanisms into a unified indexer subsystem.
     *
     * @return configured IndexerSuperstructure instance
     */
    public static IndexerSuperstructure get() {
        return new IndexerSuperstructure(
            IndexerFloorConstants.get(),
            IndexerCenterConstants.get());
    }
}
