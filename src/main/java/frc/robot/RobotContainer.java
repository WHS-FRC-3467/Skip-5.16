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

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.commands.DriveToPoseBase;
import frc.lib.commands.SteppableCommandGroup;
import frc.lib.commands.AlignToPoseBase.AlignMode;
import frc.lib.devices.AprilTagCamera;
import frc.lib.io.vision.VisionIO;
import frc.lib.io.vision.VisionIOPhotonVision;
import frc.lib.io.vision.VisionIOPhotonVisionSim;
import frc.lib.mechanisms.rotary.RotaryMechanism;
import frc.lib.posestimator.PoseEstimator;
import frc.lib.util.LoggedDashboardChooser;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggedTuneableProfiledPID;
import frc.lib.util.PointInPolygon;
import frc.lib.util.AutoCommand;
import frc.lib.util.CommandXboxControllerExtended;
import frc.lib.util.GamePieceVisualizer;
import frc.robot.Constants.Mode;
import frc.robot.Constants.PathConstants;
import frc.robot.commands.AlignToPose;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.OnTheFlyPathCommand;
import frc.robot.commands.autos.BranchingAuto;
import frc.robot.commands.autos.ExampleAuto;
import frc.robot.commands.autos.NoneAuto;
import frc.robot.commands.autos.WheelCharacterizationAuto;
import frc.robot.commands.autos.WheelSlipAuto;
import frc.robot.subsystems.beambreak1.BeamBreak1;
import frc.robot.subsystems.beambreak1.BeamBreak1Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDsConstants;
import frc.robot.subsystems.objectdetector.ObjectDetector;
import frc.robot.subsystems.objectdetector.ObjectDetectorConstants;
import frc.robot.subsystems.objectdetector.ObjectDetector;
import frc.robot.subsystems.objectdetector.ObjectDetectorConstants;
import frc.robot.subsystems.servo1.Servo1;
import frc.robot.subsystems.servo1.Servo1Constants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.turret.FlywheelConstants;
import frc.robot.subsystems.turret.TurretSuperstructure;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turret.TurretSuperstructureConstants;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.BallSimulator;
import frc.robot.subsystems.lasercan1.LaserCAN1;
import frc.robot.subsystems.lasercan1.LaserCAN1Constants;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Volts;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import java.util.function.Supplier;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.VisionSystemSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
    private final RobotState robotState = RobotState.getInstance();

    // Subsystems
    public final Drive drive;
    private final LEDs leds;
    private final LaserCAN1 laserCAN1;
    private final BeamBreak1 beamBreak1;
    private final Servo1 servo1;
    private final ObjectDetector objectDetector;
    private final Superstructure superstructure;
    private final TurretSuperstructure turret;
    private final Indexer indexer;

    // Controller
    private final CommandXboxControllerExtended controller = new CommandXboxControllerExtended(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<AutoCommand> autoChooser;
    private final LoggedDashboardChooser<Boolean> conditionalChooser;
    public static Field2d autoPreviewField = new Field2d();

    private final Trigger inAllianceRegionTrigger;

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    public RobotContainer()
    {
        drive = DriveConstants.get();
        laserCAN1 = LaserCAN1Constants.get();
        leds = LEDsConstants.get();
        beamBreak1 = BeamBreak1Constants.get();
        superstructure = SuperstructureConstants.get();
        servo1 = Servo1Constants.get();
        objectDetector = ObjectDetectorConstants.get();
        turret = TurretSuperstructureConstants.get();
        indexer = IndexerConstants.get();
        VisionConstants.create();

        conditionalChooser = new LoggedDashboardChooser<>("Conditional Choice");
        conditionalChooser.addOption("True", true);
        conditionalChooser.addOption("False", false);

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        SmartDashboard.putData("Auto Preview", autoPreviewField);
        SmartDashboard.putData("Run Indexer expel", indexer.intakeCommand(Indexer.State.EXPEL));
        SmartDashboard.putData("Run Indexer intake", indexer.intakeCommand(Indexer.State.PULL));
        autoChooser.addDefaultOption("None", new NoneAuto());
        autoChooser.addOption("ExampleAuto", new ExampleAuto(drive));
        autoChooser.addOption("BranchingAuto",
            new BranchingAuto(drive, () -> conditionalChooser.get()));

        autoChooser.onChange(auto -> {
            autoPreviewField.getObject("path").setPoses(auto.getAllPathPoses());
        });

        autoChooser.addOption("Drive Wheel Radius Characterization",
            new WheelCharacterizationAuto(drive));

        autoChooser.addOption("Wheel Slip Characterization", new WheelSlipAuto(drive));

        inAllianceRegionTrigger = new Trigger(() -> PointInPolygon.pointInPolygon(
            robotState.getEstimatedPose().getTranslation(),
            FieldConstants.ALLIANCE_STATION_POLYGON));

        // Configure the button bindings
        configureButtonBindings();

        GamePieceVisualizer algae = new GamePieceVisualizer("Algae",
            new Pose3d(new Translation3d(3, 3, 1), new Rotation3d(0, 0, 0)));


    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
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

        // Lock to 0° when A button is held
        controller
            .a()
            .whileTrue(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -controller.getLeftY(),
                    () -> -controller.getLeftX(),
                    () -> new Rotation2d()));

        // Switch to X pattern when X button is pressed
        // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro to 0° when B button is pressed
        controller
            .b()
            .onTrue(
                Commands.runOnce(
                    () -> robotState.resetPose(
                        new Pose2d(robotState.getEstimatedPose().getTranslation(),
                            new Rotation2d())))
                    .ignoringDisable(true));

        // Pathfind to Pose when the Y button is pressed
        controller.y().onTrue(
            DriveCommands.pathFindToPose(() -> robotState.getEstimatedPose(),
                new Pose2d(1, 4, Rotation2d.kZero),
                PathConstants.ON_THE_FLY_PATH_CONSTRAINTS, MetersPerSecond.of(0.0),
                PathConstants.PATHGENERATION_DRIVE_TOLERANCE));

        // On-the-fly path with waypoints while the Right Bumper is held
        controller.rightBumper().whileTrue(
            new OnTheFlyPathCommand(drive, () -> robotState.getEstimatedPose(),
                new ArrayList<>(Arrays.asList()), // List
                // of
                // waypoints
                new Pose2d(6, 6, Rotation2d.k180deg), PathConstants.ON_THE_FLY_PATH_CONSTRAINTS,
                MetersPerSecond.of(0.0), false, PathConstants.PATHGENERATION_DRIVE_TOLERANCE,
                PathConstants.PATHGENERATION_ROT_TOLERANCE));

        SmartDashboard.putData("Superstructure: Stow",
            superstructure.setGoal(Superstructure.Setpoint.STOW));
        SmartDashboard.putData("Superstructure: Raised",
            superstructure.setGoal(Superstructure.Setpoint.RAISED));

        LoggedTunableNumber ballVel = new LoggedTunableNumber("Ball Sim Velocity (fps)", 15);
        SmartDashboard.putData("Shoot Ball", Commands
            .runOnce(() -> BallSimulator.launch(FeetPerSecond.of(ballVel.getAsDouble()))));

        GamePieceVisualizer algaeViz =
            new GamePieceVisualizer("Algae #1", new Pose3d(1, 1, 1, new Rotation3d()));
        SmartDashboard.putData("Hide Algae", Commands.runOnce(() -> algaeViz.hide()));

        LoggedTuneableProfiledPID linearController =
            new LoggedTuneableProfiledPID("DriveToPose/LinearController", 3.0, 0, 0.1, 0, 3.0);

        SmartDashboard.putData("DriveToPose Command",
            new DriveToPose(drive, () -> new Pose2d(5, 5, Rotation2d.fromDegrees(90)))
                .withTolerance(Inches.of(3), Degrees.of(5)));
        Command steppableCommand = new SteppableCommandGroup(
            controller.x(),
            controller.y(),
            Commands.runOnce(() -> System.out.println("Step 1")),
            Commands.runOnce(() -> System.out.println("Step 2")),
            Commands.runOnce(() -> System.out.println("Step 3")));

        SmartDashboard.putData("Steppable Command", steppableCommand);

        // controller.x()
        // .whileTrue(new DriveToPose(drive, () -> new Pose2d(5, 5, Rotation2d.fromDegrees(90)))
        // .withTolerance(Inches.of(3), Degrees.of(5)));

        // controller.x()
        // .whileTrue(new AlignToPose(drive, () -> new Pose2d(5, 5, Rotation2d.fromDegrees(0)),
        // AlignMode.STRAFE, () -> controller.getRightX()));

        // Right bumper: Shoot on the Move
        controller.rightBumper().whileTrue(
            turret.shoot(drive, () -> -controller.getLeftX(), () -> -controller.getLeftY()));

        inAllianceRegionTrigger.onTrue(
            Commands.runOnce(() -> Logger.recordOutput("InAllianceRegionTrigger", true))
                .ignoringDisable(true));
        inAllianceRegionTrigger.onFalse(
            Commands.runOnce(() -> Logger.recordOutput("InAllianceRegionTrigger", false))
                .ignoringDisable(true));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return autoChooser.get();
    }

    /** This function is called periodically by Robot.java when disabled. */
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
