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
package frc.robot.subsystems.intake;


public class IntakeSuperstructureConstants {
    /**
     * Creates and configures an {@link IntakeSuperstructure} subsystem. Combines the linear intake
     * and roller intake mechanisms into a unified intake system.
     *
     * @return a configured {@link IntakeSuperstructure} instance
     */
    public static IntakeSuperstructure get() {
        return new IntakeSuperstructure(
            IntakeLinearConstants.getMechanism(),
            IntakeRollerConstants.getMechanism());
    }
}
