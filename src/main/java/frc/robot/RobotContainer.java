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
import frc.lib.util.FieldUtil;
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
import frc.robot.subsystems.intakeRoller.IntakeRoller.State;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDsConstants;
import frc.robot.subsystems.objectdetector.ObjectDetector;
import frc.robot.subsystems.objectdetector.ObjectDetectorConstants;
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
import edu.wpi.first.math.geometry.Rotation2d;
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
    private final LEDs leds;
    private final ObjectDetector objectDetector;
    private final ShooterSuperstructure shooter;
    private final IntakeRoller intakeRoller;
    private final IntakeLinear intakeLinear;
    private final Indexer indexer;
    private final Tower tower;

    // Controller
    private final CommandXboxControllerExtended controller = new CommandXboxControllerExtended(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<AutoRoutine> autoChooser;
    public static Field2d autoPreviewField = new Field2d();

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    public RobotContainer()
    {
        drive = DriveConstants.get();
        leds = LEDsConstants.get();
        objectDetector = ObjectDetectorConstants.get();
        shooter = ShooterSuperstructureConstants.get();
        intakeRoller = IntakeRollerConstants.get();
        intakeLinear = IntakeLinearConstants.get();
        indexer = IndexerConstants.get();
        tower = TowerConstants.get();
        VisionConstants.create();

        if (RobotBase.isSimulation()) {
            RobotSim.getInstance().addMechanismData(drive, shooter, indexer, intakeRoller,
                intakeLinear);
        }

        autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        SmartDashboard.putData("Auto Preview", autoPreviewField);

        // Default - No Auto
        autoChooser.addDefaultOption("None", new NoneAuto());

        // Preload + Neutral Auto
        autoChooser.addOption("BasicNeutralAuto-Center",
            new PreloadNeutralAuto(drive, intakeLinear, intakeRoller, indexer, tower, shooter,
                StartPosition.CENTER));
        // Depot + Neutral Auto
        autoChooser.addOption("DepotNeutralAuto-Left", new DepotNeutralAuto(drive, intakeLinear,
            intakeRoller, indexer, tower, shooter, StartPosition.LEFT));


        // Depot Auto
        autoChooser.addOption("DepotAuto-Center",
            new DepotAuto(drive, intakeLinear, intakeRoller, indexer, tower, shooter,
                StartPosition.CENTER));

        autoChooser.onChange(auto -> {
            autoPreviewField.getObject("path").setPoses(auto.getAllPathPoses());
        });

        autoChooser.addOption("Drive Wheel Radius Characterization",
            new WheelCharacterizationAuto(drive));

        autoChooser.addOption("Wheel Slip Characterization", new WheelSlipAuto(drive));

        // Configure the button bindings
        configureButtonBindings();
        initializeDashboard();
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

        // Right Trigger: Teleop vision align to largest contour (translation allowed)
        // controller.rightTrigger(0.2)
        // .whileTrue(new TeleopAlignToObject(drive, objectDetector, ContourSelectionMode.LARGEST,
        // () -> -controller.getLeftY(), // forward/back
        // () -> -controller.getLeftX(), // strafe
        // () -> -controller.getRightX())); // fallback rotation

        controller.button(1).whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> robotState.getAngleToTarget()));

        // Left Bumper: Intake while held
        controller.leftBumper().onTrue(intakeRoller.setStateCommand(State.INTAKE))
            .onFalse(intakeRoller.stop());

        // Back Button: Eject while held
        controller.back().onTrue(intakeRoller.setStateCommand(State.EJECT))
            .onFalse(intakeRoller.stop());

        controller.rightTrigger().whileTrue(
            shooter.prepareShot(
                indexer.holdStateUntilInterrupted(Indexer.State.PULL)));

        // TODO: change button bindings as necessary
        // X button: Trench align rotational assist
        controller.x().whileTrue(
            DriveCommands.joystickDriveAtAngle(drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> (FieldUtil.shouldFlip() ? Rotation2d.k180deg : Rotation2d.kZero)));

        // Y button: Align rotation parallel to trench while held - Returning from CENTER
        controller.y().whileTrue(
            DriveCommands.joystickDriveAtAngle(drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> robotState.getTunnelAssistHeading()));
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

        SmartDashboard.putData("Intake Roller/Eject",
            intakeRoller.setStateCommand(IntakeRoller.State.EJECT));
        SmartDashboard.putData("Intake Roller/Intake",
            intakeRoller.setStateCommand(IntakeRoller.State.INTAKE));
        SmartDashboard.putData("Intake Roller/Stop",
            intakeRoller.setStateCommand(IntakeRoller.State.STOP));
        SmartDashboard.putData("Intake Linear/Extend", intakeLinear.extend());
        SmartDashboard.putData("Intake Linear/Retract", intakeLinear.retract());
        SmartDashboard.putData("Intake Linear/Cycle", intakeLinear.cycle());

        SmartDashboard.putData("Sim Test: Toggle Tip Drivebase",
            Commands.run(() -> RobotState.getInstance().setDrivetrainAngled(true)));

        SmartDashboard.putData("Set flywheel to 30",
            shooter.setFlyWheelSpeed(RotationsPerSecond.of(30)));

        SmartDashboard.putData("Set hood to 45 deg", shooter.setHoodAngle(Degrees.of(45)));

        SmartDashboard.putData("Intake Roller/Eject",
            intakeRoller.setStateCommand(IntakeRoller.State.EJECT));
        SmartDashboard.putData("Intake Roller/Intake",
            intakeRoller.setStateCommand(IntakeRoller.State.INTAKE));
        SmartDashboard.putData("Intake Roller/Stop",
            intakeRoller.setStateCommand(IntakeRoller.State.STOP));
        SmartDashboard.putData("Intake Linear/Extend", intakeLinear.extend());
        SmartDashboard.putData("Intake Linear/Retract", intakeLinear.retract());
        SmartDashboard.putData("Intake Linear/Cycle", intakeLinear.cycle());
        SmartDashboard.putData("Sim Test: Toggle Tip Drivebase",
            Commands.run(() -> RobotState.getInstance().setDrivetrainAngled(true)));
        SmartDashboard.putData("Ready Shooter", shooter.spinUpShooter());
        SmartDashboard.putData("Run Indexer", indexer.setStateCommand(Indexer.State.PULL));

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
