// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;

/** Utility class for field-related methods. */
public class FieldUtil {
    private static final Translation2d fieldCenter = FieldConstants.FIELD_CENTER;

    /**
     * Whether or not a pose needs to be flipped
     * 
     * @return True if the robot is on the red alliance, false if on the blue alliance.
     */
    public static boolean shouldFlip()
    {
        return DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    public static Pose2d handleAllianceFlip(Pose2d blue_pose)
    {
        return shouldFlip() ? blue_pose.rotateAround(fieldCenter, Rotation2d.k180deg)
            : blue_pose;
    }

    public static Translation2d handleAllianceFlip(Translation2d blue_translation)
    {
        return shouldFlip() ? blue_translation.rotateAround(fieldCenter,
            Rotation2d.k180deg) : blue_translation;
    }

    public static Rotation2d handleAllianceFlip(Rotation2d blue_rotation)
    {
        return shouldFlip() ? blue_rotation.plus(Rotation2d.k180deg) : blue_rotation;
    }

    /**
     * Checks if a given pose is within the field boundaries, with a specified margin.
     * 
     * @param translation The tranlslation to check.
     * @param margin The distance inset from the boundaries.
     * @return True if the pose is within the field boundaries, false otherwise.
     */
    public static boolean isPoseInField(Translation2d translation, Distance margin)
    {
        return translation.getX() >= margin.in(Meters)
            && translation.getX() <= FieldConstants.FIELD_LENGTH.in(Meters) - margin.in(Meters)
            && translation.getY() >= margin.in(Meters)
            && translation.getY() <= FieldConstants.FIELD_WIDTH.in(Meters) - margin.in(Meters);
    }
}
