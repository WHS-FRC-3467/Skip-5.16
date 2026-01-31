// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intakeLinear.IntakeLinear;
import frc.robot.subsystems.intakeRoller.IntakeRoller;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import lombok.AccessLevel;
import lombok.Getter;
import lombok.NoArgsConstructor;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class RobotSim {
    @Getter(lazy = true)
    private static final RobotSim instance = new RobotSim();

    private final RobotState robotState = RobotState.getInstance();
    private final FuelSim fuelSim = new FuelSim();

    /**
     * Adds mechanism data to the fuel sim
     * 
     * @param drive Drive subsystem for robot pose tracking
     * @param shooter Shooter superstructure for shooting fuel simulation
     * @param indexer Indexer subsystem for detecting when to shoot
     * @param intakeRoller Intake roller subsystem for intake velocity detection
     * @param intakeLinear Linear intake subsystem for intake position detection
     */
    public void addMechanismData(
        Drive drive,
        ShooterSuperstructure shooter,
        Indexer indexer,
        IntakeRoller intakeRoller,
        IntakeLinear intakeLinear)
    {
        Trigger shootSimFuel = new Trigger(() -> (shooter.readyToShoot.getAsBoolean()
            && (indexer.getSpeed() > 0.1) && (fuelSim.getHeldFuel() > 0)));

        shootSimFuel.whileTrue(
            Commands.repeatingSequence(
                Commands.waitSeconds(.1),
                Commands.runOnce(() -> fuelSim.setHeldFuel(fuelSim.getHeldFuel() - 1)),
                Commands.runOnce(() -> fuelSim.spawnFuel(
                    new Translation3d(robotState.getEstimatedPose().getTranslation())
                        .plus(new Translation3d(Inches.of(0), Inches.of(0), Inches.of(20))),
                    fuelSim.launchVel(shooter.getAverageLinearVelocity(),
                        shooter.getHoodAngle())))));

        Trigger intakeSimFuel =
            new Trigger(() -> (intakeRoller.getVelocity().in(RotationsPerSecond) > 1.0)
                && intakeLinear.isExtended.getAsBoolean());

        fuelSim.spawnStartingFuel();
        fuelSim.registerRobot(
            Constants.FULL_ROBOT_WIDTH.in(Meters),
            Constants.FULL_ROBOT_LENGTH.in(Meters),
            Constants.BUMPER_HEIGHT.in(Meters),
            robotState::getEstimatedPose,
            robotState::getVelocity);
        fuelSim.registerIntake(
            -Constants.FULL_ROBOT_LENGTH.div(2).in(Meters),
            Constants.FULL_ROBOT_LENGTH.div(2).plus(Inches.of(10)).in(Meters),
            -Constants.FULL_ROBOT_WIDTH.div(2).in(Meters),
            Constants.FULL_ROBOT_WIDTH.div(2).in(Meters),
            intakeSimFuel);

        fuelSim.start();
        SmartDashboard.putData(Commands.runOnce(() -> {
            fuelSim.clearFuel();
            fuelSim.spawnStartingFuel();
        }).withName("Reset Fuel").ignoringDisable(true));
    }

    public FuelSim getFuelSim()
    {
        return fuelSim;
    }

    /** Updates all data that only matters in sim/replay. Should be called periodically */
    public void updateSim()
    {
        fuelSim.updateSim();
    }
}
