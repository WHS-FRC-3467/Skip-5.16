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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.lib.util.CommandXboxControllerExtended;
import frc.lib.util.FieldUtil;
import frc.lib.util.LoggedDashboardChooser;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.autos.*;
import frc.robot.commands.autos.tuning.FeedforwardCharacterizationAuto;
import frc.robot.commands.autos.tuning.WheelCharacterizationAuto;
import frc.robot.commands.autos.utils.AutoContext;
import frc.robot.commands.autos.utils.AutoOption;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.indexer.IndexerSuperstructure;
import frc.robot.subsystems.indexer.IndexerSuperstructureConstants;
import frc.robot.subsystems.intake.IntakeLinearConstants;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.intake.IntakeSuperstructureConstants;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.shooter.ShooterSuperstructureConstants;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.tower.TowerConstants;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.RobotSim;

import org.littletonrobotics.junction.Logger;

/**
 * Container class for the robot that holds all subsystems, controllers, and command bindings. This
 * class is responsible for:
 *
 * <ul>
 *   <li>Instantiating all subsystems
 *   <li>Configuring controller button bindings
 *   <li>Providing autonomous command selection
 *   <li>Setting up dashboard controls and telemetry
 * </ul>
 */
public class RobotContainer {
    private final RobotState robotState = RobotState.getInstance();

    // Subsystems
    public final Drive drive;
    final ShooterSuperstructure shooter;
    private final IntakeSuperstructure intake;
    private final IndexerSuperstructure indexer;
    private final Tower tower;
    // private final ObjectDetector objectDetector;

    // Reusable composite command to stop/stow/eject used in multiple bindings
    private final Command stopAllShooterAndRetract;

    // Controller
    private final CommandXboxControllerExtended controller =
            new CommandXboxControllerExtended(0).withDeadband(0.1);
    private final CommandXboxControllerExtended operatorController =
            new CommandXboxControllerExtended(1).withDeadband(0.1);

    // Dashboard inputs
    private final LoggedDashboardChooser<AutoOption> autoChooser;
    public final Field2d autoPreviewField = new Field2d();
    private Pose2d startPose = new Pose2d(); // Initialize start pose for auto dashboard tab

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {

        drive = DriveConstants.get();
        shooter = ShooterSuperstructureConstants.get();
        intake = IntakeSuperstructureConstants.get();
        indexer = IndexerSuperstructureConstants.get();
        tower = TowerConstants.get();
        VisionConstants.create();
        // VisionOdometryCharacterizer.enable();
        // objectDetector = ObjectDetectorConstants.get();

        stopAllShooterAndRetract =
                Commands.parallel(
                        shooter.stopAndStow(),
                        indexer.stopCommand(),
                        tower.stopCommand(),
                        intake.extendIntake(),
                        shooter.retractHood());

        if (RobotBase.isSimulation()) {
            RobotSim.getInstance().addMechanismData(drive, shooter, indexer, intake);
        }
        AutoContext ctx = AutoContext.create(drive, intake, indexer, tower, shooter);

        autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        SmartDashboard.putData("Auto Preview", autoPreviewField);

        // Default - No Auto
        autoChooser.addDefaultOption("None", NoneAuto.create());

        // Preload Autos
        autoChooser.addOption("PreloadAuto", PreloadAuto.create(ctx));

        // Neutral Autos
        NeutralAuto.create(ctx, false, false)
                .ifPresent(a -> autoChooser.addOption("Aggressive-Left", a));
        NeutralAuto.create(ctx, true, false)
                .ifPresent(a -> autoChooser.addOption("Aggressive-Right", a));
        NeutralAuto.create(ctx, false, true).ifPresent(a -> autoChooser.addOption("Safe-Left", a));
        NeutralAuto.create(ctx, true, true).ifPresent(a -> autoChooser.addOption("Safe-Right", a));

        DepotShootAuto.create(ctx, false)
                .ifPresent(a -> autoChooser.addOption("Aggressive-Depot", a));
        DepotShootAuto.create(ctx, true).ifPresent(a -> autoChooser.addOption("Safe-Depot", a));

        autoChooser.onChange(
                auto -> {
                    if (auto == null) {
                        autoPreviewField.getObject("path").setPoses(new Pose2d[] {});
                        return;
                    }
                    var pathPoses =
                            auto.previewPoses().stream()
                                    .map(FieldUtil::apply)
                                    .toArray(Pose2d[]::new);
                    if (pathPoses.length == 0) return;
                    pathPoses[0] = FieldUtil.apply(auto.startingPose());

                    autoPreviewField.getObject("path").setPoses(pathPoses);

                    auto.command();
                });

        autoChooser.addOption(
                "Drive Wheel Radius Characterization", WheelCharacterizationAuto.create(ctx));

        autoChooser.addOption(
                "Feedforward Characterization", FeedforwardCharacterizationAuto.create(ctx));

        // Configure the button bindings
        configureButtonBindings();
        initializeDashboard();
    }

    /**
     * Configures button bindings for the Xbox controller. Maps controller inputs to robot commands
     * for teleop control.`
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX()));

        // Right Trigger: Shoot/Pass
        controller
                .rightTrigger()
                .whileTrue(
                        Commands.parallel(
                                Commands.either(
                                        DriveCommands.joystickDriveFacingFutureTarget(
                                                drive,
                                                () -> -controller.getLeftY() * 0.4,
                                                () -> -controller.getLeftX() * 0.4,
                                                robotState.feedLookaheadSeconds),
                                        DriveCommands.staticAimTowardsTarget(drive),
                                        shooter.shouldFeed),
                                shooter.spinUpShooter(),
                                Commands.parallel(indexer.shoot(), tower.shoot())
                                        .onlyWhile(
                                                shooter.readyToShoot.and(
                                                        robotState.facingFeedTarget))
                                        .repeatedly()))
                .onFalse(stopAllShooterAndRetract);

        // Tap Right Bumper while Right Trigger held: Manually cycle intake
        controller
                .rightBumper()
                .and(controller.x().negate())
                .onTrue(intake.shuffleStep())
                .onFalse(intake.stopRoller());

        // Tap Right Bumper while X held: Manually cycle intake within bumpers for hub shot
        controller
                .rightBumper()
                .and(controller.x())
                .onTrue(intake.hubShuffleStep())
                .onFalse(intake.stopRoller());

        // Left Trigger: Intake
        controller.leftTrigger().onTrue(intake.intake()).onFalse(intake.stopRoller());

        // D-Pad Up: Force Intake Linear Slide Back
        controller.leftBumper().onTrue(intake.retractIntake());

        // D-Pad Down: Unjam
        controller
                .povDown()
                .whileTrue(Commands.parallel(intake.ejectRoller(), indexer.eject(), tower.eject()));

        // Driver X: Hub Shot (No-Vision Fallback)
        controller
                .x()
                .whileTrue(
                        Commands.parallel(
                                DriveCommands.stopWithX(drive),
                                shooter.spinUpShooterToFixedDistance(
                                        FieldConstants.Hub.HUB_SHOT_DISTANCE),
                                Commands.parallel(indexer.shoot(), tower.shoot())))
                .onFalse(stopAllShooterAndRetract);

        // Driver Y: Midline Feed/Pass (No-Vision Fallback)
        controller
                .y()
                .whileTrue(
                        Commands.parallel(
                                shooter.spinUpShooterMidlineFeed(),
                                Commands.sequence(
                                        Commands.waitUntil(shooter.atMidlineFeedSetpoints)
                                                .withTimeout(0.75),
                                        Commands.parallel(indexer.shoot(), tower.shoot()))))
                .onFalse(stopAllShooterAndRetract);

        // Driver A: Shot From Back of Robot Against Tower (No-Vision Fallback)
        controller
                .a()
                .whileTrue(
                        Commands.parallel(
                                DriveCommands.stopWithX(drive),
                                shooter.spinUpShooterToFixedDistance(
                                        FieldConstants.Tower.TOWER_SHOT_DISTANCE),
                                Commands.parallel(indexer.shoot(), tower.shoot())))
                .onFalse(stopAllShooterAndRetract);

        controller
                .start()
                .and(controller.back())
                .onTrue(
                        Commands.runOnce(
                                () ->
                                        robotState.resetPose(
                                                new Pose2d(
                                                        robotState
                                                                .getEstimatedPose()
                                                                .getTranslation(),
                                                        Rotation2d.kZero))));

        // Operator A: Home Hood and Intake
        operatorController
                .a()
                .whileTrue(Commands.parallel(intake.homeLinear(), shooter.homeHood()));

        // Operator B: Eject
        operatorController
                .b()
                .whileTrue(Commands.parallel(intake.ejectRoller(), indexer.eject(), tower.eject()));

        // Operator X: Retract Intake
        operatorController.x().onTrue(intake.retractIntake());

        // Operator POV Up: Trim shot power up
        operatorController.povUp().onTrue(shooter.trimFlywheelSpeedUp());

        // Operator POV Down: Trim shot power down
        operatorController.povDown().onTrue(shooter.trimFlywheelSpeedDown());

        // Operator Y: Manual Spinup
        operatorController
                .y()
                .whileTrue(
                        shooter.spinUpShooter()
                                .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        controller
                .rightTrigger()
                .negate()
                .and(operatorController.y().negate())
                .onTrue(shooter.stopFlywheels());
    }

    /**
     * Initializes SmartDashboard with test commands and controls for subsystems. Adds commands to
     * the dashboard for manual testing and debugging.
     */
    private void initializeDashboard() {

        // Intake Commands
        SmartDashboard.putData(IntakeLinearConstants.NAME + "/Intake", intake.intake());
        SmartDashboard.putData(IntakeLinearConstants.NAME + "/Retract", intake.retractIntake());
        SmartDashboard.putData(IntakeLinearConstants.NAME + "/Coast", intake.linearCoast());
        SmartDashboard.putData("Home", Commands.parallel(intake.homeLinear(), shooter.homeHood()));
        SmartDashboard.putData(IntakeLinearConstants.NAME + "/SlowRetract", intake.slowRetract());

        SmartDashboard.putData(
                "Debug/SetOdometryToTestPose",
                Commands.runOnce(() -> robotState.resetPose(new Pose2d(8, 5, Rotation2d.k180deg))));

        SmartDashboard.putData(
                "Fountain",
                Commands.sequence(
                                Commands.sequence(
                                        Commands.parallel(
                                                shooter.fountain(),
                                                indexer.fountain(),
                                                tower.fountain())),
                                Commands.parallel(shooter.idle(), indexer.idle(), tower.idle()))
                        .finallyDo(
                                () ->
                                        CommandScheduler.getInstance()
                                                .schedule(
                                                        shooter.stopAndStow(),
                                                        indexer.stopCommand(),
                                                        tower.stopCommand())));

        // Drivetrain Commands
        SmartDashboard.putData(
                "Drive to Start Pose",
                new DriveToPose(drive, () -> startPose)
                        .withDistanceTolerance(Meters.of(0.04))
                        .withAngularTolerance(Degrees.of(3)));
    }

    /**
     * Gets the selected autonomous command from the dashboard chooser.
     *
     * @return the autonomous command to run
     */
    public Command getAutonomousCommand() {
        AutoOption option = autoChooser.get();
        return option == null ? Commands.none() : option.command();
    }

    /**
     * Checks and displays the robot's starting pose accuracy relative to the selected autonomous
     * path every 100ms.
     *
     * @param counter an integer from 0 to 4, that determines whether to spend the time to display
     *     info.
     */
    public void checkStartPose(int counter) {
        var pathObj = autoPreviewField.getObject("path");
        var poses = pathObj.getPoses();
        if (poses == null || poses.isEmpty()) {
            startPose = robotState.getEstimatedPose();
        } else {
            startPose = poses.get(0);
        }

        // Only update dashboard every 100 ms
        if (counter == 0) {
            /* Starting pose checker for auto */
            autoPreviewField.setRobotPose(robotState.getEstimatedPose());

            if (poses == null || poses.isEmpty()) {
                // No path defined; show defaults
                startPose = robotState.getEstimatedPose();
                SmartDashboard.putNumber("Auto Pose Check/Inches from Start", -1);
                SmartDashboard.putBoolean("Auto Pose Check/Robot Position Within Tolerance", false);
                SmartDashboard.putNumber("Auto Pose Check/Degrees from Start", -1);
                SmartDashboard.putBoolean("Auto Pose Check/Robot Rotation Within Tolerance", false);
                return;
            }

            Logger.recordOutput("Auto/StartPose", startPose);
            autoPreviewField.getObject("startPose").setPose(startPose);

            Pose2d robotPose = robotState.getEstimatedPose();
            Distance distanceFromStartPose =
                    Meters.of(robotPose.getTranslation().getDistance(startPose.getTranslation()));
            double degreesFromStartPose =
                    Math.abs(robotPose.getRotation().minus(startPose.getRotation()).getDegrees());

            double[] startPoseArray = {
                startPose.getX(), startPose.getY(), startPose.getRotation().getDegrees()
            };
            SmartDashboard.putNumberArray("Start Pose (x, y, degrees)", startPoseArray);

            SmartDashboard.putNumber(
                    "Auto Pose Check/Inches from Start",
                    (int) Math.round(distanceFromStartPose.in(Inches) * 100.0) / 100.0);
            SmartDashboard.putBoolean(
                    "Auto Pose Check/Robot Position Within Tolerance",
                    distanceFromStartPose.in(Inches)
                            < Constants.STARTING_POSE_DRIVE_TOLERANCE.in(Inches));
            SmartDashboard.putNumber(
                    "Auto Pose Check/Degrees from Start",
                    (int) Math.round(degreesFromStartPose * 100.0) / 100.0);
            SmartDashboard.putBoolean(
                    "Auto Pose Check/Robot Rotation Within Tolerance",
                    degreesFromStartPose
                            < Constants.STARTING_POSE_ROT_TOLERANCE_DEGREES.in(Degrees));
        }
    }
}
