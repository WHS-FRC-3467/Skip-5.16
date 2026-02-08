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

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import au.grapplerobotics.CanBridge;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.Elastic;
import frc.robot.util.HubState;
import frc.robot.util.RobotSim;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private final RobotState robotState = RobotState.getInstance();

    private Command autonomousCommand;
    private RobotContainer robotContainer;
    private boolean checkedHubGameData = false; // whether we've checked for hub game data at the
                                                // start of the first alliance phase
    private Field2d fieldMap = new Field2d();

    public Robot()
    {
        CanBridge.runTCP(); // Used for configuring LaserCANs via Grapplehook

        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0 -> Logger.recordMetadata("GitDirty", "All changes committed");
            case 1 -> Logger.recordMetadata("GitDirty", "Uncommitted changes");
            default -> Logger.recordMetadata("GitDirty", "Unknown");
        }

        // Set up data receivers & replay source
        switch (Constants.currentMode) {
            case REAL -> {
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                LoggedPowerDistribution.getInstance(Ports.pdh.id(), ModuleType.kRev);
            }

            case SIM -> {
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
            }

            case REPLAY -> {
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger
                    .setReplaySource(new WPILOGReader(logPath));
                Logger
                    .addDataReceiver(
                        new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            }
        }

        // Start AdvantageKit logger
        Logger.start();

        // Check for valid swerve config
        var modules = new SwerveModuleConstants[] {
                DriveConstants.FrontLeft,
                DriveConstants.FrontRight,
                DriveConstants.BackLeft,
                DriveConstants.BackRight
        };
        for (var constants : modules) {
            if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
                || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
                throw new RuntimeException(
                    "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
            }
        }

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        DriverStation.silenceJoystickConnectionWarning(!Robot.isReal());
    }

    @Override
    public void robotInit()
    {
        /*
         * Due to the nature of how Java works, the first run of a pathfinding command could have a
         * significantly higher delay compared with subsequent runs. To help alleviate this issue,
         * run this warmup command in the background when code starts. This command will not control
         * the robot, it will simply run through a full pathfinding command to warm up the library.
         * Source: PathPlanner Docs
         */
        // DO THIS AFTER CONFIGURATION OF YOUR DESIRED PATHFINDER
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

        // Log first 8 character of robot serial
        Logger.recordOutput("Robot Serial",
            Robot.isReal() ? Constants.RobotConstants.serial.subSequence(0, 8).toString()
                : Constants.RobotConstants.serial);

        SmartDashboard.putData("Robot Pose Field Map", fieldMap);
    }

    /**
     * This function is called periodically during all modes. Runs the CommandScheduler and updates
     * robot state.
     */
    @Override
    public void robotPeriodic()
    {
        // Optionally switch the thread to high priority to improve loop
        // timing (see the template project documentation for details)
        // Threads.setCurrentThreadPriority(true, 99);

        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        CommandScheduler.getInstance().run();

        // Return to non-RT thread priority (do not modify the first argument)
        // Threads.setCurrentThreadPriority(false, 10);

        // Driver Elastic Dashboard - Update the robot's pose on the main fieldmap
        fieldMap.setRobotPose(RobotState.getInstance().getEstimatedPose());
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit()
    {
        // Switch to Autonomous tab in Elastic Dashboard
        if (DriverStation.isFMSAttached()) {
            Elastic.selectTab(1);
        }
        // Reset hub game data check before starting the next match
        checkedHubGameData = false;
    }

    /**
     * This function is called periodically when disabled. Checks and displays the robot's starting
     * pose for autonomous mode.
     */
    @Override
    public void disabledPeriodic()
    {
        robotContainer.checkStartPose();
        robotContainer.autoPreviewField.setRobotPose(robotState.getEstimatedPose());
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit()
    {
        // Switch to Autonomous tab in Elastic Dashboard
        if (DriverStation.isFMSAttached()) {
            Elastic.selectTab(1);
        }

        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic()
    {
        robotContainer.autoPreviewField.setRobotPose(robotState.getEstimatedPose());
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit()
    {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        // Switch to Teleop tab in Elastic Dashboard
        if (DriverStation.isFMSAttached()) {
            Elastic.selectTab(0);
        }
    }

    /**
     * This function is called periodically during operator control. Manages hub state timing and
     * game data updates during teleop.
     */
    @Override
    public void teleopPeriodic()
    {
        // Hub State management
        double firstHubChangeTime = HubState.getHubChangeTimes()[0];
        if (!checkedHubGameData && DriverStation.getMatchTime() <= firstHubChangeTime
            && DriverStation.getMatchTime() > firstHubChangeTime - 1.0) {
            // At the beginning of the first alliance phase, check for hub game data
            HubState.getInstance().setFirstActiveAlliance();
            checkedHubGameData = true;
        }
        HubState.getInstance().periodic();
    }


    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit()
    {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic()
    {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit()
    {
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic()
    {
        RobotSim.getInstance().updateSim();
    }
}
