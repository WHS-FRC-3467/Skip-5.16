// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.lib.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.IndexerSuperstructure;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.shooter.ShooterSuperstructure;

import lombok.AccessLevel;
import lombok.Getter;
import lombok.NoArgsConstructor;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class RobotSim {
    @Getter(lazy = true)
    private static final RobotSim instance = new RobotSim();

    /**
     * A fudge factor (multiplier) for the velocity of launched fuel in simulation vs the velocity
     * of the flywheels
     */
    private static final LoggedTunableNumber LAUNCH_VELOCITY_FUDGE =
            new LoggedTunableNumber("Sim/LaunchVelocityFudge", 0.72);

    private final RobotState robotState = RobotState.getInstance();

    private final FuelSim fuelSim = new FuelSim();

    private MechanismPosePublisher posePublisher = null;

    private void registerFuelSimMechanisms(
            Drive drive,
            ShooterSuperstructure shooter,
            IndexerSuperstructure indexer,
            IntakeSuperstructure intake) {
        Trigger shootSimFuel =
                new Trigger(
                        () ->
                                (shooter.readyToShoot.getAsBoolean()
                                        && (indexer.getFloorSpeed() > 0.1)
                                        && (fuelSim.getHeldFuel() > 0)));

        shootSimFuel.whileTrue(
                Commands.repeatingSequence(
                        Commands.waitSeconds(.1),
                        Commands.runOnce(
                                () -> {
                                    LinearVelocity launchVelocity =
                                            shooter.getAverageLinearVelocity()
                                                    .times(LAUNCH_VELOCITY_FUDGE.get());

                                    fuelSim.spawnFuel(
                                            new Pose3d(robotState.getEstimatedPose())
                                                    .plus(
                                                            new Transform3d(
                                                                    Inches.of(-10),
                                                                    Inches.of(-3.6),
                                                                    Inches.of(21),
                                                                    Rotation3d.kZero))
                                                    .getTranslation(),
                                            fuelSim.launchVel(
                                                    launchVelocity,
                                                    Degrees.of(75.0)
                                                            .minus(shooter.getHoodAngle())));

                                    // Only shoot out of one shooter if there is one ball
                                    if (fuelSim.getHeldFuel() == 1) {
                                        fuelSim.fillHopperBy(-1);
                                        return;
                                    }

                                    fuelSim.spawnFuel(
                                            new Pose3d(robotState.getEstimatedPose())
                                                    .plus(
                                                            new Transform3d(
                                                                    Inches.of(-10),
                                                                    Inches.of(3.6),
                                                                    Inches.of(21),
                                                                    Rotation3d.kZero))
                                                    .getTranslation(),
                                            fuelSim.launchVel(
                                                    launchVelocity,
                                                    Degrees.of(75.0)
                                                            .minus(shooter.getHoodAngle())));

                                    fuelSim.fillHopperBy(-2);
                                })));

        Trigger intakeSimFuel = new Trigger(intake::isIntaking);

        fuelSim.enableAirResistance();
        fuelSim.spawnStartingFuel();
        fuelSim.setHopperFuel(8);
        fuelSim.registerRobot(
                Constants.FULL_ROBOT_WIDTH.in(Meters),
                Constants.FULL_ROBOT_LENGTH.in(Meters),
                Constants.BUMPER_HEIGHT.in(Meters),
                robotState::getEstimatedPose,
                robotState::getFieldRelativeVelocity);
        fuelSim.registerIntake(
                -Constants.FULL_ROBOT_LENGTH.div(2).in(Meters),
                Constants.FULL_ROBOT_LENGTH.div(2).plus(Inches.of(12)).in(Meters),
                -Constants.FULL_ROBOT_WIDTH.div(2).in(Meters),
                Constants.FULL_ROBOT_WIDTH.div(2).in(Meters),
                intakeSimFuel);

        fuelSim.start();
        SmartDashboard.putData(
                Commands.runOnce(
                                () -> {
                                    // Reset field fuels, then fill the hopper with placeholder
                                    // fuels.
                                    fuelSim.clearFuel();
                                    fuelSim.spawnStartingFuel();
                                    fuelSim.setHopperFuel(8);
                                })
                        .withName("Reset Fuel")
                        .ignoringDisable(true));
    }

    /**
     * Adds mechanism data to the sim
     *
     * @param drive Drive subsystem for robot pose tracking
     * @param shooter Shooter superstructure
     * @param indexer IndexerSuperstructure subsystem
     * @param intake Intake subsystem
     */
    public void addMechanismData(
            Drive drive,
            ShooterSuperstructure shooter,
            IndexerSuperstructure indexer,
            IntakeSuperstructure intake) {
        registerFuelSimMechanisms(drive, shooter, indexer, intake);
        posePublisher = new MechanismPosePublisher(intake, shooter);
    }

    public FuelSim getFuelSim() {
        return fuelSim;
    }

    /** Updates all data that only matters in sim/replay. Should be called periodically */
    public void updateSim() {
        fuelSim.updateSim();

        if (posePublisher != null) posePublisher.update();
    }
}
