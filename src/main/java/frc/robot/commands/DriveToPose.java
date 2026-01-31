// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.commands.DriveToPoseBase;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggedTuneableProfiledPID;
import frc.robot.subsystems.drive.Drive;

public class DriveToPose extends DriveToPoseBase {

    private final static LoggedTuneableProfiledPID linearController =
        new LoggedTuneableProfiledPID("DriveToPose/LinearController", 3.0, 0, 0.1, 3.0, 0.0);
    private final static LoggedTuneableProfiledPID angularController =
        new LoggedTuneableProfiledPID("DriveToPose/AngularController", 3.0, 0, 0, 0, 0);

    private final static LoggedTunableNumber maxLinearVel =
        new LoggedTunableNumber("DriveToPose/MaxLinearVelocity (m s)", 3.0);
    private final static LoggedTunableNumber maxAngularVel =
        new LoggedTunableNumber("DriveToPose/MaxAngularVelocity (rad s)", 9.0);

    /**
     * Creates a command to autonomously drive the robot to a target pose.
     * 
     * @param drive The Drive subsystem to control robot movement.
     * @param targetPose A Supplier that provides the target pose to drive to.
     */
    public DriveToPose(
        Drive drive,
        Supplier<Pose2d> targetPose)
    {
        super(
            drive,
            targetPose,
            linearController,
            angularController,
            maxLinearVel,
            maxAngularVel);
    }
}
