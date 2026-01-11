// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lasercan1;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.devices.DistanceSensor;
import frc.lib.io.distancesensor.DistanceSensorIO;
import frc.robot.RobotState;

public class LaserCAN1 extends SubsystemBase {
    private final DistanceSensor distanceSensor;
    private final RobotState robotState;

    public final Trigger inside =
        new Trigger(() -> betweenDistance(Millimeters.of(5), Millimeters.of(10)));

    public LaserCAN1(DistanceSensorIO io)
    {
        distanceSensor = new DistanceSensor(io);
        robotState = RobotState.getInstance();
    }

    @Override
    public void periodic()
    {
        distanceSensor.periodic();
        Logger.recordOutput(LaserCAN1Constants.NAME + "Sensor Reading Pose",
            new Pose3d(robotState.getEstimatedPose())
                .plus(LaserCAN1Constants.LASERCAN_TRANSFORM.plus(
                    new Transform3d(
                        getDistance(),
                        Inches.of(0),
                        Inches.of(0),
                        new Rotation3d()))));
    }

    public Distance getDistance()
    {
        if (distanceSensor.getDistance().isEmpty()) {
            return Inches.of(-1.0);
        } else {
            return distanceSensor.getDistance().get();
        }
    }

    public boolean betweenDistance(Distance min, Distance max)
    {
        if (distanceSensor.getDistance().isEmpty()) {
            return false;
        }

        Distance distance = distanceSensor.getDistance().get();
        return distance.gt(min) && distance.lt(max);
    }
}
