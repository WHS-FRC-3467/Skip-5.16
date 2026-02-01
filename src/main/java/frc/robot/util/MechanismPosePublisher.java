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

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.intakeLinear.IntakeLinear;
import frc.robot.subsystems.shooter.ShooterSuperstructure;

public record MechanismPosePublisher(IntakeLinear intake, ShooterSuperstructure shooter) {
    public void update()
    {
        Logger.recordOutput("MechanismPoses/Intake (1)",
            new Pose3d(intake.getExtension(), Meters.zero(), Meters.zero(), Rotation3d.kZero));
        Logger.recordOutput("MechanismPoses/Hood (2)",
            new Pose3d(Translation3d.kZero,
                new Rotation3d(shooter.getHoodAngle(), Rotations.zero(), Rotations.zero())));
    }
}
