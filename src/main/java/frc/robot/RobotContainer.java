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
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import frc.robot.subsystems.intake.IntakeLinearConstants;
import frc.robot.subsystems.intake.IntakeRollerConstants;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.intake.IntakeSuperstructureConstants;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDsConstants;
import frc.robot.subsystems.objectdetector.ObjectDetector;
import frc.robot.subsystems.objectdetector.ObjectDetectorConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.shooter.HoodConstants;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.shooter.ShooterSuperstructureConstants;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.tower.TowerConstants;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.RobotSim;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Distance;

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
    private final IntakeSuperstructure intake;
    private final Indexer indexer;
    private final Tower tower;
    private final ObjectDetector objectDetector;
    private final LEDs leds;

    // Controller
    private final CommandXboxControllerExtended controller =
        new CommandXboxControllerExtended(0).withDeadband(0.1);

    // Dashboard inputs
    private final LoggedDashboardChooser<AutoRoutine> autoChooser;
    public final Field2d autoPreviewField = new Field2d();

    private final Trigger isAutonomous = new Trigger(DriverStation::isAutonomous);

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    public RobotContainer()
    {

        drive = DriveConstants.get();
        shooter = ShooterSuperstructureConstants.get();
        intake = IntakeSuperstructureConstants.get();
        indexer = IndexerConstants.get();
        tower = TowerConstants.get();
        VisionConstants.create();
        objectDetector = ObjectDetectorConstants.get();
        leds = LEDsConstants.get();

        if (RobotBase.isSimulation()) {
            RobotSim.getInstance().addMechanismData(drive, shooter, indexer,
                intake);
        }

        autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        SmartDashboard.putData("Auto Preview", autoPreviewField);

        // Default - No Auto
        autoChooser.addDefaultOption("None", new NoneAuto());

        // Preload Autos
        autoChooser.addOption("PreloadAuto-Left", new PreloadAuto(drive, intake,
            indexer, tower, shooter, StartPosition.LEFT));
        autoChooser.addOption("PreloadAuto-Center",
            new PreloadAuto(drive, intake,
                indexer, tower, shooter, StartPosition.CENTER));
        autoChooser.addOption("PreloadAuto-Right",
            new PreloadAuto(drive, intake,
                indexer, tower, shooter, StartPosition.RIGHT));

        // Basic Neutral Autos
        autoChooser.addOption("BasicNeutralAuto-Left", new BasicNeutralAuto(drive, intake,
            indexer, tower, shooter, StartPosition.LEFT));
        autoChooser.addOption("BasicNeutralAuto-Right", new BasicNeutralAuto(drive, intake,
            indexer, tower, shooter, StartPosition.RIGHT));

        // Depot Auto
        autoChooser.addOption("DepotAuto-Left",
            new DepotAuto(drive, intake, indexer, tower, shooter,
                StartPosition.LEFT));
        autoChooser.addOption("DepotAuto-Center",
            new DepotAuto(drive, intake, indexer, tower, shooter,
                StartPosition.CENTER));
        autoChooser.addOption("DepotAuto-Right",
            new DepotAuto(drive, intake, indexer, tower, shooter,
                StartPosition.RIGHT));

        // Outpost Autos
        autoChooser.addOption("OutpostAuto-Left",
            new OutpostAuto(drive, intake, indexer, tower, shooter,
                StartPosition.LEFT));
        autoChooser.addOption("OutpostAuto-Center",
            new OutpostAuto(drive, intake, indexer, tower, shooter,
                StartPosition.CENTER));
        autoChooser.addOption("OutpostAuto-Right",
            new OutpostAuto(drive, intake, indexer, tower, shooter,
                StartPosition.RIGHT));

        autoChooser.onChange(auto -> {
            autoPreviewField.getObject("path")
                .setPoses(auto.getAllPathPoses().stream()
                    .map(p -> FieldUtil.apply(p)).toArray(Pose2d[]::new));
        });

        autoChooser.addOption("Drive Wheel Radius Characterization",
            new WheelCharacterizationAuto(drive));

        autoChooser.addOption("Wheel Slip Characterization", new WheelSlipAuto(drive));



        // Configure the button bindings
        configureButtonBindings();
        initializeDashboard();
        configureLEDTriggers();
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
                DriveCommands.staticAimTowardsTarget(drive),
                // Prepare shooter superstructure
                shooter.prepareShot(
                    // While shooter superstructure is prepared,
                    // and drivetrain is aiming towards the target,
                    // shoot
                    Commands.parallel(
                        indexer.holdStateUntilInterrupted(Indexer.State.PULL),
                        tower.holdStateUntilInterrupted(Tower.State.SHOOT),
                        intake.cycle())
                        .onlyWhile(robotState.facingTarget)
                        .repeatedly())))
            .onFalse(Commands.parallel(
                shooter.setFlywheelSpeed(RotationsPerSecond.zero()),
                indexer.setStateCommand(Indexer.State.STOP),
                tower.setStateCommand(Tower.State.IDLE),
                intake.extend()));

        // Left Trigger: Intake
        controller.leftTrigger().onTrue(
            Commands.parallel(
                intake.holdStateUntilInterruptedAndExtend(IntakeSuperstructure.State.INTAKE)))
            .onFalse(intake.setStateCommand(IntakeSuperstructure.State.STOP));

        // Right Bumper: Trench Align
        controller.rightBumper()
            .whileTrue(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -controller.getLeftY(),
                    () -> -controller.getLeftX(),
                    robotState::getTunnelAssistHeading));

        // D-Pad Up: Force Intake Linear Slide Back
        controller.povUp().onTrue(intake.retract());

        // D-Pad Down: Unjam
        controller.povDown().whileTrue(Commands.parallel(
            intake.holdStateUntilInterrupted(IntakeSuperstructure.State.EJECT),
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
            intake.setStateCommand(IntakeSuperstructure.State.EJECT));
        SmartDashboard.putData(IntakeRollerConstants.NAME + "/Intake",
            intake.setStateCommand(IntakeSuperstructure.State.INTAKE));
        SmartDashboard.putData(IntakeRollerConstants.NAME + "/Stop",
            intake.setStateCommand(IntakeSuperstructure.State.STOP));
        SmartDashboard.putData(IntakeLinearConstants.NAME + "/Extend", intake.extend());
        SmartDashboard.putData(IntakeLinearConstants.NAME + "/Retract", intake.retract());
        SmartDashboard.putData(IntakeLinearConstants.NAME + "/Cycle", intake.cycle());

        SmartDashboard.putData(shooter.getName() + "/Ready", shooter.spinUpShooter());
        SmartDashboard.putData("Hood angle", Commands.runOnce(() -> System.out.println(
            Degrees.of(90).minus(HoodConstants.MIN_ANGLE_OFFSET).minus(shooter.getHoodAngle())
                .in(Degrees))));

        SmartDashboard.putData("Intake Roller/Eject",
            intake.setStateCommand(IntakeSuperstructure.State.EJECT));
        SmartDashboard.putData("Intake Roller/Intake",
            intake.setStateCommand(IntakeSuperstructure.State.INTAKE));
        SmartDashboard.putData("Intake Roller/Stop",
            intake.setStateCommand(IntakeSuperstructure.State.STOP));
        SmartDashboard.putData("Intake Linear/Extend", intake.extend());
        SmartDashboard.putData("Intake Linear/Retract", intake.retract());
        SmartDashboard.putData("Intake Linear/Cycle", intake.cycle());
        SmartDashboard.putData("Intake Linear/Coast", intake.coast());
        SmartDashboard.putData("Ready Shooter", shooter.spinUpShooter());
        SmartDashboard.putData("Run Indexer", indexer.setStateCommand(Indexer.State.PULL));
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
                        .plus(Constants.LEFT_SHOOTER_EXIT_TRANSFORM)
                        .getTranslation(),
                    fuelSim.launchVel(shooter.getAverageLinearVelocity(),
                        shooter.getExitAngle()));
                fuelSim.spawnFuel(
                    new Pose3d(robotState.getEstimatedPose())
                        .plus(Constants.RIGHT_SHOOTER_EXIT_TRANSFORM)
                        .getTranslation(),
                    fuelSim.launchVel(shooter.getAverageLinearVelocity(),
                        shooter.getExitAngle()));

                SmartDashboard.putData("Toggle Tip Drivebase",
                    Commands.run(
                        () -> robotState.setDrivetrainAngled(!robotState.isDrivetrainAngled())));
            }));
        }
    }

    /** Creates and/or binds triggers to LED states */
    private void configureLEDTriggers()
    {
        isAutonomous
            .onTrue(leds.scheduleStateCommand(LEDs.State.RUNNING_AUTO))
            .onFalse(leds.unscheduleStateCommand(LEDs.State.RUNNING_AUTO));

        shooter.readyToShoot
            .onTrue(leds.scheduleStateCommand(LEDs.State.READY_TO_SHOOT))
            .onFalse(leds.unscheduleStateCommand(LEDs.State.READY_TO_SHOOT));

        new Trigger(() -> intake.getState() == IntakeSuperstructure.State.INTAKE)
            .onTrue(leds.scheduleStateCommand(LEDs.State.RUNNING_INTAKE))
            .onFalse(leds.unscheduleStateCommand(LEDs.State.RUNNING_INTAKE));
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
            Pose2d startPose = autoPreviewField.getObject("path").getPoses().get(0);
            autoPreviewField.getObject("startPose").setPose(startPose);

            Distance distanceFromStartPose =
                Meters.of(robotState.getEstimatedPose().getTranslation()
                    .getDistance(startPose.getTranslation()));
            double degreesFromStartPose = Math.abs(robotState.getEstimatedPose().getRotation()
                .minus(startPose.getRotation())
                .getDegrees());

            double[] startPoseArray =
                {startPose.getX(), startPose.getY(), startPose.getRotation().getDegrees()};
            SmartDashboard.putNumberArray("Start Pose (x, y, degrees)", startPoseArray);

            SmartDashboard.putNumber("Auto Pose Check/Inches from Start",
                (int) Math.round(distanceFromStartPose.in(Inches) * 100.0) / 100.0);
            SmartDashboard.putBoolean(
                "Auto Pose Check/Robot Position Within Tolerance",
                distanceFromStartPose.in(Inches) < PathConstants.STARTING_POSE_DRIVE_TOLERANCE
                    .in(Inches));
            SmartDashboard.putNumber("Auto Pose Check/Degrees from Start",
                (int) Math.round(degreesFromStartPose * 100.0) / 100.0);
            SmartDashboard.putBoolean(
                "Auto Pose Check/Robot Rotation Within Tolerance",
                degreesFromStartPose < PathConstants.STARTING_POSE_ROT_TOLERANCE_DEGREES
                    .in(Degrees));

        } catch (Exception e) {
            SmartDashboard.putNumber("Auto Pose Check/Inches from Start", -1);
            SmartDashboard.putBoolean(
                "Auto Pose Check/Robot Position Within Tolerance",
                false);
            SmartDashboard.putNumber("Auto Pose Check/Degrees from Start", -1);
            SmartDashboard.putBoolean(
                "Auto Pose Check/Robot Rotation Within Tolerance",
                false);
        }
    }
}
