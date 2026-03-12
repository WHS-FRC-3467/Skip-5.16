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
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.lib.util.AutoRoutine;
import frc.lib.util.CommandXboxControllerExtended;
import frc.lib.util.FieldUtil;
import frc.lib.util.LoggedDashboardChooser;
import frc.lib.util.VisionOdometryCharacterizer;
import frc.robot.Constants.PathConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.autos.*;
import frc.robot.commands.autos.tuning.FeedforwardCharacterizationAuto;
import frc.robot.commands.autos.tuning.WheelCharacterizationAuto;
import frc.robot.commands.autos.tuning.WheelSlipAuto;
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
import frc.robot.util.HubState;
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
    // private final LEDs leds;
    // private final ObjectDetector objectDetector;

    // Controller
    private final CommandXboxControllerExtended controller =
            new CommandXboxControllerExtended(0).withDeadband(0.1);
    private final CommandXboxControllerExtended operatorController =
            new CommandXboxControllerExtended(1).withDeadband(0.1);

    // Dashboard inputs
    private final LoggedDashboardChooser<AutoRoutine> autoChooser;
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
        // leds = LEDsConstants.get();
        // objectDetector = ObjectDetectorConstants.get();

        if (RobotBase.isSimulation()) {
            RobotSim.getInstance().addMechanismData(drive, shooter, indexer, intake);
        }
        autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        SmartDashboard.putData("Auto Preview", autoPreviewField);

        // Default - No Auto
        autoChooser.addDefaultOption("None", new NoneAuto());

        // Preload Autos
        autoChooser.addOption(
                "PreloadAuto", new PreloadAuto(drive, intake, indexer, tower, shooter));

        // Neutral Autos
        autoChooser.addOption(
                "Aggressive-Left",
                new NeutralAuto(drive, intake, indexer, tower, shooter, false, false));
        autoChooser.addOption(
                "Aggressive-Right",
                new NeutralAuto(drive, intake, indexer, tower, shooter, true, false));
        autoChooser.addOption(
                "Safe-Left", new NeutralAuto(drive, intake, indexer, tower, shooter, false, true));
        autoChooser.addOption(
                "Safe-Right", new NeutralAuto(drive, intake, indexer, tower, shooter, true, true));
        autoChooser.addOption(
                "AntiNashoba-Left", new AntiNashoba(drive, intake, indexer, tower, shooter, false));
        autoChooser.addOption(
                "AntiNashoba-Right", new AntiNashoba(drive, intake, indexer, tower, shooter, true));

        // Depot Autos
        autoChooser.addOption(
                "DepotAuto-Left", new DepotSideAuto(drive, intake, indexer, tower, shooter));
        autoChooser.addOption(
                "DepotAuto-Center", new DepotCenterAuto(drive, intake, indexer, tower, shooter));

        autoChooser.onChange(
                auto -> {
                    var pathPoses =
                            auto.getAllPathPoses().stream()
                                    .map(p -> FieldUtil.apply(p))
                                    .toArray(Pose2d[]::new);
                    if (pathPoses.length == 0) return;
                    pathPoses[0] = FieldUtil.apply(auto.getStartingPose());

                    autoPreviewField.getObject("path").setPoses(pathPoses);
                });

        autoChooser.addOption(
                "Drive Wheel Radius Characterization", new WheelCharacterizationAuto(drive));

        autoChooser.addOption("Wheel Slip Characterization", new WheelSlipAuto(drive));

        autoChooser.addOption(
                "Feedforward Characterization", new FeedforwardCharacterizationAuto(drive));

        // Configure the button bindings
        configureButtonBindings();
        initializeDashboard();
        configureLEDTriggers();
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
                        () -> -controller.slewLeftY(),
                        () -> -controller.slewLeftX(),
                        () -> -controller.slewRightX() * 0.75));

        // Right Trigger: Shoot/Pass
        controller
                .rightTrigger()
                .whileTrue(
                        Commands.parallel(
                                DriveCommands.staticAimTowardsTarget(drive),
                                shooter.spinUpShooter(),
                                Commands.parallel(indexer.shoot(), tower.shoot())
                                        .onlyWhile(
                                                shooter.readyToShoot.and(robotState.facingTarget))
                                        .repeatedly()))
                .onFalse(
                        Commands.parallel(
                                shooter.stopAndStow(),
                                indexer.stopCommand(),
                                tower.stopCommand(),
                                intake.extendIntake(),
                                shooter.retractHood()));

        // Right Bumper: Manually cycle intake
        controller.rightBumper().onTrue(intake.shuffleStep()).onFalse(intake.stopRoller());

        // Left Trigger: Intake
        controller.leftTrigger().onTrue(intake.intake()).onFalse(intake.stopRoller());

        // A: Trench Align
        controller
                .a()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                drive,
                                () -> -controller.slewLeftY(),
                                () -> -controller.slewLeftX(),
                                robotState::getTunnelAssistHeading));

        controller.rightBumper().onTrue(intake.retractIntake());
        controller
                .rightBumper()
                .onFalse(Commands.sequence(Commands.waitSeconds(0.25), intake.extendIntake()));

        // D-Pad Up: Force Intake Linear Slide Back
        controller.leftBumper().onTrue(intake.retractIntake());

        // D-Pad Down: Unjam
        controller
                .povDown()
                .whileTrue(Commands.parallel(intake.ejectRoller(), indexer.eject(), tower.eject()));

        // Tap D-Pad Right: Prepare shot from up against the HUB (No-Vision Fallback)
        controller
                .x()
                .whileTrue(
                        Commands.parallel(
                                shooter.spinUpShooterToHubDistance(),
                                Commands.parallel(indexer.shoot(), tower.shoot())))
                .onFalse(
                        Commands.parallel(
                                shooter.stopAndStow(),
                                indexer.stopCommand(),
                                tower.stopCommand(),
                                intake.extendIntake(),
                                shooter.retractHood()));
        operatorController
                .a()
                .whileTrue(Commands.parallel(intake.homeLinear(), shooter.homeHood()));
        operatorController
                .b()
                .whileTrue(Commands.parallel(intake.ejectRoller(), indexer.eject(), tower.eject()));
        operatorController.x().onTrue(intake.retractIntake());
        operatorController.povUp().onTrue(shooter.trimFlywheelSpeedUp());
        operatorController.povDown().onTrue(shooter.trimFlywheelSpeedDown());

        // robotState.enteringTrench.whileTrue(
        //         shooter.forceHoodAngle(Rotations.zero())
        //                 .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        //                 .onlyIf(isAutonomous.negate()));

        new EventTrigger("RetractIntake").onTrue(intake.retractIntake());
        new EventTrigger("ExtendIntake").onTrue(intake.autoIntake());
        new EventTrigger("Spinup").onTrue(shooter.spinUpShooterToHubDistance(Meters.of(3.555)));

        HubState.getInstance()
                .getHubChange()
                .whileTrue(
                        Commands.repeatingSequence(
                                controller.rumbleForTime(
                                        RumbleType.kBothRumble, 1.0, Seconds.of(0.5)),
                                Commands.waitSeconds(0.5)))
                .onFalse(controller.rumble(RumbleType.kBothRumble, 0.0));
    }

    /**
     * Initializes SmartDashboard with test commands and controls for subsystems. Adds commands to
     * the dashboard for manual testing and debugging.
     */
    private void initializeDashboard() {

        // Indexer Commands
        SmartDashboard.putData(IndexerSuperstructureConstants.NAME + "/Shoot", indexer.shoot());
        SmartDashboard.putData(IndexerSuperstructureConstants.NAME + "/Expel", indexer.eject());
        SmartDashboard.putData(IndexerSuperstructureConstants.NAME + "/Feed", indexer.feed());
        SmartDashboard.putData(
                IndexerSuperstructureConstants.NAME + "/Stop", indexer.stopCommand());

        // Intake Commands
        SmartDashboard.putData(IntakeLinearConstants.NAME + "/Intake", intake.intake());
        SmartDashboard.putData(IntakeLinearConstants.NAME + "/Retract", intake.retractIntake());
        SmartDashboard.putData(IntakeLinearConstants.NAME + "/Coast", intake.linearCoast());
        SmartDashboard.putData("Home", Commands.parallel(intake.homeLinear(), shooter.homeHood()));
        SmartDashboard.putData(IntakeLinearConstants.NAME + "/SlowRetract", intake.slowRetract());

        // Tower Commands
        SmartDashboard.putData(TowerConstants.NAME + "/Stop", tower.stopCommand());
        SmartDashboard.putData(TowerConstants.NAME + "/Shoot", tower.shoot());
        SmartDashboard.putData(TowerConstants.NAME + "/Feed", tower.feed());
        SmartDashboard.putData(TowerConstants.NAME + "/Eject", tower.eject());

        // Shooter Commands
        SmartDashboard.putData(
                ShooterSuperstructureConstants.NAME + "/Stop", shooter.stopFlywheels());
        SmartDashboard.putData(
                ShooterSuperstructureConstants.NAME + "/Spinup", shooter.spinUpShooter());
        SmartDashboard.putData(
                ShooterSuperstructureConstants.NAME + "/SlowSpinup", shooter.slowSpinup());

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
                "Face Target",
                DriveCommands.joystickDriveFacingTarget(
                        drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));
        SmartDashboard.putData(
                "Drive to Start Pose",
                new DriveToPose(drive, () -> startPose)
                        .withDistanceTolerance(Meters.of(0.04))
                        .withAngularTolerance(Degrees.of(3)));

        SmartDashboard.putData("Drive/Zero", drive.runOnce(() -> drive.runCharacterization(0.0)));
        SmartDashboard.putData(
                "Drive/Face Zero",
                DriveCommands.joystickDriveAtAngle(
                        drive, () -> 0.0, () -> 0.0, () -> Rotation2d.kZero));
        SmartDashboard.putData(
                "Drive/Face 180",
                DriveCommands.joystickDriveAtAngle(
                        drive, () -> 0.0, () -> 0.0, () -> Rotation2d.k180deg));

        // Diagnostics
        SmartDashboard.putData(
                "Enable Vision Odometry Characterization",
                Commands.runOnce(() -> VisionOdometryCharacterizer.enable()));
        SmartDashboard.putData(
                "Disable Vision Odometry Characterization",
                Commands.runOnce(() -> VisionOdometryCharacterizer.disable()));
        SmartDashboard.putData(
                "Reset Vision Odometry Characterization",
                Commands.runOnce(() -> VisionOdometryCharacterizer.reset()));
    }

    /** Creates and/or binds triggers to LED states */
    private void configureLEDTriggers() {
        // isAutonomous
        //         .onTrue(leds.scheduleStateCommand(LEDs.State.RUNNING_AUTO))
        //         .onFalse(leds.unscheduleStateCommand(LEDs.State.RUNNING_AUTO));

        // shooter.readyToShoot
        //         .onTrue(leds.scheduleStateCommand(LEDs.State.READY_TO_SHOOT))
        //         .onFalse(leds.unscheduleStateCommand(LEDs.State.READY_TO_SHOOT));

        // new Trigger(() -> intake.isIntaking())
        //         .onTrue(leds.scheduleStateCommand(LEDs.State.RUNNING_INTAKE))
        //         .onFalse(leds.unscheduleStateCommand(LEDs.State.RUNNING_INTAKE));
    }

    /**
     * Gets the selected autonomous command from the dashboard chooser.
     *
     * @return the autonomous command to run
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    /**
     * Checks and displays the robot's starting pose accuracy relative to the selected autonomous
     * path.
     */
    public void checkStartPose() {

        /* Starting pose checker for auto */
        autoPreviewField.setRobotPose(robotState.getEstimatedPose());

        try {
            Pose2d startPose = autoPreviewField.getObject("path").getPoses().get(0);
            Logger.recordOutput("Auto/StartPose", startPose);

            autoPreviewField.getObject("startPose").setPose(startPose);

            Distance distanceFromStartPose =
                    Meters.of(
                            robotState
                                    .getEstimatedPose()
                                    .getTranslation()
                                    .getDistance(startPose.getTranslation()));
            double degreesFromStartPose =
                    Math.abs(
                            robotState
                                    .getEstimatedPose()
                                    .getRotation()
                                    .minus(startPose.getRotation())
                                    .getDegrees());

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
                            < PathConstants.STARTING_POSE_DRIVE_TOLERANCE.in(Inches));
            SmartDashboard.putNumber(
                    "Auto Pose Check/Degrees from Start",
                    (int) Math.round(degreesFromStartPose * 100.0) / 100.0);
            SmartDashboard.putBoolean(
                    "Auto Pose Check/Robot Rotation Within Tolerance",
                    degreesFromStartPose
                            < PathConstants.STARTING_POSE_ROT_TOLERANCE_DEGREES.in(Degrees));

        } catch (Exception e) {
            startPose = robotState.getEstimatedPose();
            SmartDashboard.putNumber("Auto Pose Check/Inches from Start", -1);
            SmartDashboard.putBoolean("Auto Pose Check/Robot Position Within Tolerance", false);
            SmartDashboard.putNumber("Auto Pose Check/Degrees from Start", -1);
            SmartDashboard.putBoolean("Auto Pose Check/Robot Rotation Within Tolerance", false);
        }
    }
}
