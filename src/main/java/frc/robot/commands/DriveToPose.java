// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.commands.DriveToPoseBase;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggedTuneableProfiledPID;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

/**
 * Command to autonomously drive the robot to a target pose on the field.
 *
 * <p>Uses profiled PID controllers for both linear (x, y) and angular (rotation) motion. The
 * controllers have tunable gains that can be adjusted through NetworkTables for optimization.
 * Maximum velocities are also tunable.
 *
 * <p>This command extends {@link DriveToPoseBase} and provides the robot-specific controller
 * configuration. It's commonly used for autonomous positioning and driver-assistance features.
 */
public class DriveToPose extends DriveToPoseBase {

    private static final LoggedTuneableProfiledPID linearController =
            new LoggedTuneableProfiledPID("DriveToPose/LinearController", 3.0, 0, 0.1, 3.0, 0.0);
    private static final LoggedTuneableProfiledPID angularController =
            new LoggedTuneableProfiledPID("DriveToPose/AngularController", 3.0, 0, 0, 0, 0);

    private static final LoggedTunableNumber maxLinearVel =
            new LoggedTunableNumber("DriveToPose/MaxLinearVelocity (m s)", 3.0);
    private static final LoggedTunableNumber maxAngularVel =
            new LoggedTunableNumber("DriveToPose/MaxAngularVelocity (rad s)", 9.0);

    /**
     * Creates a command to autonomously drive the robot to a target pose.
     *
     * @param drive the Drive subsystem to control robot movement
     * @param targetPose a Supplier that provides the target pose to drive to
     */
    public DriveToPose(Drive drive, Supplier<Pose2d> targetPose) {
        super(drive, targetPose, linearController, angularController, maxLinearVel, maxAngularVel);
    }
}
