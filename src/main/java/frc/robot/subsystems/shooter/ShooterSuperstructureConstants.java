// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;

/**
 * Constants class for creating the ShooterSuperstructure subsystem. Provides factory method to
 * construct the complete shooter with hood and flywheel mechanisms.
 */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class ShooterSuperstructureConstants {

    public static final String NAME = "Shooter Superstructure";

    /**
     * Creates and configures a complete ShooterSuperstructure subsystem. Combines hood and both
     * flywheel mechanisms into a unified shooter system.
     *
     * @return configured ShooterSuperstructure instance
     */
    public static ShooterSuperstructure get() {
        return new ShooterSuperstructure(
                HoodConstants.get(), FlywheelConstants.get());
    }
}
