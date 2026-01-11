// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// https://github.com/3015RangerRobotics/2024Public/blob/main/RobotCode2024/src/main/java/frc/robot/util/NoteSimulator.java#L11

package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.FieldConstants;
import frc.robot.RobotState;

/** Simulates a ball being launched from the robot. */
public class BallSimulator {
    private static Pose3d currentPose = new Pose3d();
    private static Translation3d objectVelocity = new Translation3d();
    private static List<Translation3d> objectTrajectory = new ArrayList<>();

    private static double dT = 0.02; // Time step
    private static final double AIR_DENSITY = 1.225;
    private static final double DRAG_COEFFICIENT = 0.45;
    private static final double CROSSECTION_AREA =
        Math.PI * Math.pow(FieldConstants.ALGAE_DIAMETER.in(Meters) / 2, 2);
    private static final double MASS = 0.68;

    private static Distance ballDiameter = FieldConstants.ALGAE_DIAMETER;

    private static Transform3d shooterOffset =
        new Transform3d(
            Inches.of(0),
            Inches.of(0),
            Inches.of(30),
            new Rotation3d(
                0,
                -Math.toRadians(45), 0 // 45 degree launch angle
            ));

    public static void launch(LinearVelocity velocity)
    {
        RobotState robotState = RobotState.getInstance();

        objectTrajectory.clear();

        // Set initial position
        currentPose = new Pose3d(robotState.getEstimatedPose()).plus(shooterOffset);

        // Set initial velocity
        objectVelocity = new Translation3d(velocity.in(MetersPerSecond), currentPose.getRotation());

        // Add drivetrain velocity
        objectVelocity = objectVelocity.plus(new Translation3d(
            robotState.getFieldRelativeVelocity().vxMetersPerSecond,
            robotState.getFieldRelativeVelocity().vyMetersPerSecond,
            0));

    }

    public static void update()
    {
        // Update position
        currentPose = new Pose3d(
            currentPose.getTranslation().plus(objectVelocity.times(dT)),
            currentPose.getRotation());

        // Check for ground collision
        if (currentPose.getZ() < ballDiameter.in(Meters) / 2) {
            currentPose = new Pose3d(
                currentPose.getX(),
                currentPose.getY(),
                ballDiameter.in(Meters) / 2 - 0.001,
                currentPose.getRotation());
            objectVelocity = new Translation3d(0, 0, 0);
        } else {
            // Add gravitation effects
            objectVelocity = objectVelocity.plus(new Translation3d(0, 0, -9.81 * dT));

            // Add drag effects
            objectVelocity = addDrag(objectVelocity);

            // Record trajectory
            objectTrajectory.add(currentPose.getTranslation());

            Logger.recordOutput("Ball Simulator/Current Pose", currentPose);
            Logger.recordOutput("Ball Simulator/Current Trajectory",
                objectTrajectory.toArray(new Translation3d[objectTrajectory.size()]));
        }

    }

    public static Translation3d addDrag(Translation3d velocity)
    {
        double speed = velocity.getNorm();
        double fDrag =
            0.5 * DRAG_COEFFICIENT * CROSSECTION_AREA * AIR_DENSITY * Math.pow(speed, 2);
        double deltaV = (fDrag * MASS) * dT;
        double t = (speed - deltaV) / speed;
        return velocity.times(t);
    }
}
