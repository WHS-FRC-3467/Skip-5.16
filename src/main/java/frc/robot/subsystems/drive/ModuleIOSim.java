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

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.util.LoggedTunableBoolean;
import frc.lib.util.LoggedTunableNumber;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ModuleIOSim implements ModuleIO {
    // TunerConstants doesn't support separate sim constants, so they are declared
    // locally
    private static final double DRIVE_KP = 0.05;
    private static final double DRIVE_KD = 0.0;
    private static final double DRIVE_KS = 0.0;
    // Updated from 0.91035 to reflect new gear ratio (6.0 vs 7.3636): 0.91035 * (6.0/7.3636) =
    // 0.742
    private static final double DRIVE_KV_ROT = 0.742;
    private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
    private static final double TURN_KP = 12.5;
    private static final double TURN_KD = 0.0;
    private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim;

    /**
     * Dashboard input for whether or not to enable simulated module skid.
     *
     * <p>When enabled, the drive motor voltage applied to the simulation is multiplied by {@link
     * #skidMultiplier}. This intentionally makes the simulated wheel "over-achieve" compared to the
     * rest of the drivetrain, which is a simple way to create a repeatable skid-like outlier for
     * testing detection and filtering logic.
     */
    private final LoggedTunableBoolean enableSkid;

    /** The multiplier applied to the drive voltage when {@link #enableSkid} is active. */
    private final LoggedTunableNumber skidMultiplier;

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
    private PIDController turnController = new PIDController(TURN_KP, 0, TURN_KD);
    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    /**
     * Constructs a new ModuleIOSim instance.
     *
     * @param constants Module-specific constants for configuring the simulation
     */
    public ModuleIOSim(
            String name,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                    constants) {
        // Create drive and turn sim models
        driveSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                DRIVE_GEARBOX,
                                constants.DriveInertia,
                                constants.DriveMotorGearRatio),
                        DRIVE_GEARBOX);
        turnSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                TURN_GEARBOX,
                                constants.SteerInertia,
                                constants.SteerMotorGearRatio),
                        TURN_GEARBOX);

        // Enable wrapping for turn PID
        turnController.enableContinuousInput(-Math.PI, Math.PI);

        // Instantiate tuning values
        enableSkid = new LoggedTunableBoolean("Drive/" + name + "/Enable Skid Simulation", false);
        skidMultiplier = new LoggedTunableNumber("Drive/" + name + "/Skid Multiplier", 1.8);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVolts =
                    driveFFVolts
                            + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
        } else {
            driveController.reset();
        }

        if (turnClosedLoop) {
            turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
        } else {
            turnController.reset();
        }

        // Update simulation state
        driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
        turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
        driveSim.update(0.02);
        turnSim.update(0.02);

        // Update drive inputs
        inputs.driveConnected = true;
        inputs.drivePositionRad = driveSim.getAngularPositionRad();
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

        // Update turn inputs
        inputs.turnConnected = true;
        inputs.turnEncoderConnected = true;
        inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPositionRad());
        inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
        inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

        // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
        inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
        inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
        inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
    }

    @Override
    public void setDriveOpenLoop(double output) {
        output =
                MathUtil.clamp(
                    output
                        * (enableSkid.get()
                                ? skidMultiplier.get()
                                : 1.0), -12.0, 12.0);

        driveClosedLoop = false;
        driveAppliedVolts = output;
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnClosedLoop = false;
        turnAppliedVolts = output;
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        velocityRadPerSec = velocityRadPerSec * (enableSkid.get() ? skidMultiplier.get() : 1.0);

        driveClosedLoop = true;
        driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec;
        driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnClosedLoop = true;
        turnController.setSetpoint(rotation.getRadians());
    }
}
