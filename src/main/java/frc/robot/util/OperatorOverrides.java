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
package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Interface defining methods a generic operator override device should support. */
public interface OperatorOverrides {
    // Stop intake, indexer, tower, and shooter
    Trigger emergencyStop();

    // Shooter spindown -- stop flywheel and lower hood
    Trigger shooterSpinDown();

    // Reverse intake, indexer, and tower
    Trigger unjam();

    // Force retract intake
    Trigger forceRetractIntake();

    // Bypass readiness/alignment check and shoot
    Trigger forceShot();

    // X-lock drivetrain
    Trigger lockDrive();
}
