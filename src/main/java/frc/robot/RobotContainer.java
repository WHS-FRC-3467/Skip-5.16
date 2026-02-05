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

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.LoggedDashboardChooser;
import frc.lib.util.AutoRoutine;
import frc.lib.util.CommandXboxControllerExtended;
import frc.robot.Constants.Mode;
import frc.robot.Constants.PathConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.autos.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.intakeLinear.IntakeLinear;
import frc.robot.subsystems.intakeLinear.IntakeLinearConstants;
import frc.robot.subsystems.intakeRoller.IntakeRoller;
import frc.robot.subsystems.intakeRoller.IntakeRollerConstants;
import frc.robot.subsystems.objectdetector.ObjectDetector;
import frc.robot.subsystems.objectdetector.ObjectDetectorConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.shooter.ShooterSuperstructureConstants;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.tower.TowerConstants;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.RobotSim;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * Container class for the robot that holds all subsystems, controllers, and command bindings. This
 * class is responsible for:
 * <ul>
 * <li>Instantiating all subsystems</li>
 * <li>Configuring controller button bindings</li>
 * <li>Providing autonomous command selection</li>
 * <li>Setting up dashboard controls and telemetry</li>
 * </ul>
 */
public class RobotContainer {
    private final RobotState robotState = RobotState.getInstance();

    // Subsystems
    public final Drive drive;
    private final ShooterSuperstructure shooter;
    private final IntakeRoller intakeRoller;
    private final IntakeLinear intakeLinear;
    private final Indexer indexer;
    private final Tower tower;
    private final ObjectDetector objectDetector;

    // Controller
    private final CommandXboxControllerExtended controller =
        new CommandXboxControllerExtended(0).withDeadband(0.1);

    // Dashboard inputs
    private final LoggedDashboardChooser<AutoRoutine> autoChooser;
    public static Field2d autoPreviewField = new Field2d();

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    public RobotContainer()
    {
        drive = DriveConstants.get();
        shooter = ShooterSuperstructureConstants.get();
        intakeRoller = IntakeRollerConstants.get();
        intakeLinear = IntakeLinearConstants.get();
        indexer = IndexerConstants.get();
        tower = TowerConstants.get();
        VisionConstants.create();
        objectDetector = ObjectDetectorConstants.get();

        if (RobotBase.isSimulation()) {
            RobotSim.getInstance().addMechanismData(drive, shooter, indexer, intakeRoller,
                intakeLinear);
        }

        autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        SmartDashboard.putData("Auto Preview", autoPreviewField);
        autoChooser.addDefaultOption("None", new NoneAuto());
        autoChooser.addOption("PreloadNeutralAuto",
            new PreloadNeutralAuto(drive, intakeLinear, intakeRoller, indexer, tower, shooter,
                StartPosition.CENTER));

        // Depot Auto - Start at Center
        autoChooser.addOption("DepotAuto",
            new DepotAuto(drive, intakeLinear, intakeRoller, indexer, tower, shooter,
                StartPosition.CENTER));

        autoChooser.addOption("OutpostAuto",
            new OutpostAuto(drive, intakeLinear, intakeRoller, indexer, tower, shooter,
                StartPosition.LEFT));

        autoChooser.onChange(auto -> {
            autoPreviewField.getObject("path").setPoses(auto.getAllPathPoses());
        });

        autoChooser.addOption("Drive Wheel Radius Characterization",
            new WheelCharacterizationAuto(drive));

        autoChooser.addOption("Wheel Slip Characterization", new WheelSlipAuto(drive));

        // Configure the button bindings
        configureButtonBindings();
        initializeDashboard();
        // boilder plate which activates on a triggers true or false
        robotState.getIsAuto()
            .onTrue(Commands.runOnce(() -> leds.smartHandler(LEDs.State.RUNNING_AUTO)));
        robotState.getIsAuto()
            .onFalse(Commands.runOnce(() -> changeState()));

        shooter.getIsShooting()
            .onTrue(Commands.runOnce(() -> leds.smartHandler(LEDs.State.SHOOTING)));
        shooter.getIsShooting()
            .onFalse(Commands.runOnce(() -> changeState()));

        shooter.readyToShoot
            .onTrue(Commands.runOnce(() -> leds.smartHandler(LEDs.State.READY_TO_SHOOT)));
        shooter.readyToShoot
            .onFalse(Commands.runOnce(() -> changeState()));

        intakeRoller.getIntakeRunning()
            .onTrue(Commands.runOnce(() -> leds.smartHandler(LEDs.State.RUNNING_INTAKE)));
        intakeRoller.getIntakeRunning()
            .onFalse(Commands.runOnce(() -> changeState()));

    }

    // on a triggers false the change state function will activate and it will go down the list of
    // triggers and when it finds one which is true it will return that respective state
    public void changeState()
    {
        LEDs.State returnState = LEDs.State.NONE; // returns led state NONE if no triggers are true
        if (robotState.getIsAuto().getAsBoolean()) { // highest priority
            returnState = LEDs.State.RUNNING_AUTO;

        } else if (shooter.getIsShooting().getAsBoolean()) {
            returnState = LEDs.State.SHOOTING;

        } else if (shooter.readyToShoot.getAsBoolean()) {
            returnState = LEDs.State.READY_TO_SHOOT;

        } else if (intakeRoller.getIntakeRunning().getAsBoolean()) { // lowest pritoty
            returnState = LEDs.State.RUNNING_INTAKE;
        }
        leds.dumbHandler(returnState);
    }

    /**
     * Configures button bindings for the Xbox controller. Maps controller inputs to robot commands
     * for teleop control.
     */
    private void configureButtonBindings()
    {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()));

        // Right Trigger: Shoot/Pass
        controller.rightTrigger().whileTrue(
            Commands.parallel(
                // Aim towards target
                DriveCommands.joystickDriveFacingTarget(
                    drive,
                    () -> -controller.getLeftY(),
                    () -> -controller.getLeftX()),
                // Prepare shooter superstructure
                shooter.prepareShot(
                    // While shooter superstructure is prepared,
                    // and drivetrain is aiming towards the target,
                    // shoot
                    Commands.parallel(
                        indexer.holdStateUntilInterrupted(Indexer.State.PULL),
                        tower.holdStateUntilInterrupted(Tower.State.SHOOT),
                        intakeLinear.cycle())
                        .onlyWhile(
                            () -> Math.abs(robotState.getAngleToTarget()
                                .minus(robotState.getEstimatedPose().getRotation())
                                .getDegrees()) < 1.0)
                        .repeatedly())))
            .onFalse(Commands.parallel(
                shooter.setFlywheelSpeed(RotationsPerSecond.zero()),
                indexer.setStateCommand(Indexer.State.STOP),
                tower.setStateCommand(Tower.State.IDLE),
                intakeLinear.extend()));

        // Left Trigger: Intake
        controller.leftTrigger().onTrue(
            Commands.parallel(
                intakeLinear.extend(),
                intakeRoller.setStateCommand(IntakeRoller.State.INTAKE)))
            .onFalse(intakeRoller.setStateCommand(IntakeRoller.State.STOP));

        // Right Bumper: Trench Align
        controller.rightBumper()
            .whileTrue(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -controller.getLeftY(),
                    () -> -controller.getLeftX(),
                    robotState::getTunnelAssistHeading));

        // D-Pad Up: Force Intake Linear Slide Back
        controller.povUp().onTrue(intakeLinear.retract());

        // D-Pad Down: Unjam
        controller.povDown().whileTrue(Commands.parallel(
            intakeRoller.holdStateUntilInterrupted(IntakeRoller.State.EJECT),
            indexer.holdStateUntilInterrupted(Indexer.State.EXPEL),
            tower.holdStateUntilInterrupted(Tower.State.EJECT)));
    }

    /**
     * Initializes SmartDashboard with test commands and controls for subsystems. Adds commands to
     * the dashboard for manual testing and debugging.
     */
    private void initializeDashboard()
    {
        SmartDashboard.putData("Indexer/Expel", indexer.setStateCommand(Indexer.State.EXPEL));
        SmartDashboard.putData("Indexer/Intake", indexer.setStateCommand(Indexer.State.PULL));
        SmartDashboard.putData("Indexer/Stop", indexer.setStateCommand(Indexer.State.STOP));

        SmartDashboard.putData(IntakeRollerConstants.NAME + "/Eject",
            intakeRoller.setStateCommand(IntakeRoller.State.EJECT));
        SmartDashboard.putData(IntakeRollerConstants.NAME + "/Intake",
            intakeRoller.setStateCommand(IntakeRoller.State.INTAKE));
        SmartDashboard.putData(IntakeRollerConstants.NAME + "/Stop",
            intakeRoller.setStateCommand(IntakeRoller.State.STOP));

        SmartDashboard.putData(IntakeLinearConstants.NAME + "/Extend", intakeLinear.extend());
        SmartDashboard.putData(IntakeLinearConstants.NAME + "/Retract", intakeLinear.retract());
        SmartDashboard.putData(IntakeLinearConstants.NAME + "/Cycle", intakeLinear.cycle());

        SmartDashboard.putData(shooter.getName() + "/Ready", shooter.spinUpShooter());

        SmartDashboard.putData("Face Target",
            DriveCommands.joystickDriveFacingTarget(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX()));

        if (Constants.currentMode == Mode.SIM) {
            var fuelSim = RobotSim.getInstance().getFuelSim();
            SmartDashboard.putData("Shoot Fuel", Commands.runOnce(() -> {
                fuelSim.spawnFuel(
                    new Pose3d(robotState.getEstimatedPose())
                        .plus(new Transform3d(Inches.of(-10), Inches.of(-3.6),
                            Inches.of(21), Rotation3d.kZero))
                        .getTranslation(),
                    fuelSim.launchVel(shooter.getAverageLinearVelocity(),
                        Degrees.of(75.0).minus(shooter.getHoodAngle())));
                fuelSim.spawnFuel(
                    new Pose3d(robotState.getEstimatedPose())
                        .plus(new Transform3d(Inches.of(-10), Inches.of(3.6),
                            Inches.of(21), Rotation3d.kZero))
                        .getTranslation(),
                    fuelSim.launchVel(shooter.getAverageLinearVelocity(),
                        Degrees.of(75.0).minus(shooter.getHoodAngle())));

                SmartDashboard.putData("Toggle Tip Drivebase",
                    Commands.run(
                        () -> robotState.setDrivetrainAngled(!robotState.isDrivetrainAngled())));
            }));
        }
    }

    /**
     * Gets the selected autonomous command from the dashboard chooser.
     * 
     * @return the autonomous command to run
     */
    public Command getAutonomousCommand()
    {
        return autoChooser.get();
    }

    /**
     * Checks and displays the robot's starting pose accuracy relative to the selected autonomous
     * path. This function is called periodically by Robot.java when disabled.
     */
    public void checkStartPose()
    {

        /* Starting pose checker for auto */
        autoPreviewField.setRobotPose(robotState.getEstimatedPose());

        try {
            double distanceFromStartPose = robotState.getEstimatedPose().getTranslation()
                .getDistance(autoPreviewField.getObject("path").getPoses().get(0).getTranslation());
            double degreesFromStartPose = Math.abs(robotState.getEstimatedPose().getRotation()
                .minus(
                    autoPreviewField.getObject("path").getPoses().get(0).getRotation())
                .getDegrees());

            SmartDashboard.putNumber("Auto Pose Check/Inches from Start",
                Math.round(distanceFromStartPose * 100.0) / 100.0);
            SmartDashboard.putBoolean(
                "Auto Pose Check/Robot Position within "
                    + PathConstants.STARTING_POSE_DRIVE_TOLERANCE.in(Inches) + " inches",
                distanceFromStartPose < PathConstants.STARTING_POSE_DRIVE_TOLERANCE.in(Inches));
            SmartDashboard.putNumber("Auto Pose Check/Degrees from Start",
                Math.round(degreesFromStartPose * 100.0) / 100.0);
            SmartDashboard.putBoolean(
                "Auto Pose Check/Robot Rotation within "
                    + PathConstants.STARTING_POSE_ROT_TOLERANCE_DEGREES + " degrees",
                degreesFromStartPose < PathConstants.STARTING_POSE_ROT_TOLERANCE_DEGREES
                    .in(Degrees));

        } catch (Exception e) {
            DriverStation.reportError(
                "Failed to check starting pose: " + e.getMessage(), e.getStackTrace());
            SmartDashboard.putNumber("Auto Pose Check/Inches from Start", -1);
            SmartDashboard.putBoolean(
                "Auto Pose Check/Robot Position within "
                    + PathConstants.STARTING_POSE_DRIVE_TOLERANCE.in(Inches) + " inches",
                false);
            SmartDashboard.putNumber("Auto Pose Check/Degrees from Start", -1);
            SmartDashboard.putBoolean(
                "Auto Pose Check/Robot Rotation within "
                    + PathConstants.STARTING_POSE_ROT_TOLERANCE_DEGREES.in(Degrees) + " degrees",
                false);
        }
    }
}