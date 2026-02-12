// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.commands.AlignToPoseBase;
import frc.lib.util.LoggedTuneableProfiledPID;
import frc.robot.subsystems.drive.Drive;

public class AlignToPose extends AlignToPoseBase {

    private final static LoggedTuneableProfiledPID linearController =
        new LoggedTuneableProfiledPID("DriveToPose/LinearController", 3.0, 0, 0.1, 3.0, 0.0);

    private final static LoggedTuneableProfiledPID angularController =
        new LoggedTuneableProfiledPID("DriveToPose/AngularController", 3.0, 0, 0, 0, 0);

    /**
     * Creates a command to align the robot to a target pose while allowing translational movement.
     *
     * @param drive the Drive subsystem to control robot movement
     * @param targetPose a Supplier that provides the target pose to align to
     * @param mode the alignment mode (e.g., align translation, rotation, or both)
     * @param joystickInput a DoubleSupplier providing driver input for translational movement
     */
    public AlignToPose(Drive drive, Supplier<Pose2d> targetPose, AlignMode mode,
        DoubleSupplier joystickInput) {
        super(drive,
            targetPose,
            mode,
            joystickInput,
            linearController,
            angularController);
    }
}
