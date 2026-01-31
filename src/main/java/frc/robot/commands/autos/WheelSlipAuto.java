// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.AutoRoutine;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;

/**
 * Autonomous routine for testing wheel slip characteristics by ramping motor voltage until
 * a target velocity is reached. Useful for understanding traction limits.
 */
public class WheelSlipAuto extends AutoRoutine {
    private final RobotState robotState = RobotState.getInstance();

    /**
     * Constructs a WheelSlipAuto that ramps drive motors to test slip characteristics.
     *
     * @param drive the drive subsystem to test
     */
    public WheelSlipAuto(Drive drive)
    {
        loadCommands(Commands.sequence(
            Commands.run(() -> drive.runCharacterization(0.0)).withTimeout(2),
            rampUntilVelocity(drive, 0.2, RotationsPerSecond.of(1))));
    }

    /**
     * Returns the starting pose for this autonomous routine based on current robot pose estimate.
     *
     * @return the current estimated pose of the robot
     */
    @Override
    public Pose2d getStartingPose()
    {
        return robotState.getEstimatedPose();
    }

    /**
     * Creates a command that ramps motor voltage linearly until the drivetrain reaches a target velocity.
     * Logs the applied voltage to help analyze wheel slip characteristics.
     *
     * @param drive the drive subsystem to ramp
     * @param rampRate the rate of voltage increase per second (volts/second)
     * @param speedLimit the target angular velocity to reach before stopping
     * @return a command that ramps voltage until the speed limit is reached
     */
    private static Command rampUntilVelocity(Drive drive, double rampRate,
        AngularVelocity speedLimit)
    {
        Timer timer = new Timer();

        return Commands.sequence(
            Commands.runOnce(() -> timer.restart()),
            Commands.deadline(
                Commands.waitUntil(() -> drive.getFFCharacterizationVelocity() > speedLimit
                    .in(RotationsPerSecond)),
                Commands.run(() -> {
                    double current = timer.get() * rampRate;
                    drive.runCharacterization(current);
                    Logger.recordOutput("Wheel Slip Current: ", current);
                })));
    }
}
