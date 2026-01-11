// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;

/** Add your docs here. */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class TurretSuperstructureConstants {
    public static TurretSuperstructure get()
    {
        return new TurretSuperstructure(
            TurretConstants.get(),
            IndexerConstants.get(),
            HoodConstants.get(),
            FlywheelConstants.get(),
            TurretSensorConstants.get());
    }
}
