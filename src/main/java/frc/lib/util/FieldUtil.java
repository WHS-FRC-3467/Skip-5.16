// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.FieldConstants;

/** Utility class for field-related methods. */
public class FieldUtil {
    /**
     * Applies alliance-specific transformation to X coordinate.
     *
     * @param x The X coordinate in blue alliance frame
     * @return The X coordinate in the current alliance frame
     */
    public static double applyX(double x) {
        return shouldFlip() ? FieldConstants.FIELD_LENGTH - x : x;
    }

    /**
     * Applies alliance-specific transformation to Y coordinate.
     *
     * @param y The Y coordinate in blue alliance frame
     * @return The Y coordinate in the current alliance frame
     */
    public static double applyY(double y) {
        return shouldFlip() ? FieldConstants.FIELD_WIDTH - y : y;
    }

    /**
     * Applies alliance-specific transformation to a translation.
     *
     * @param translation The translation in blue alliance frame
     * @return The translation in the current alliance frame
     */
    public static Translation2d apply(Translation2d translation) {
        return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
    }

    /**
     * Applies alliance-specific transformation to a rotation.
     *
     * @param rotation The rotation in blue alliance frame
     * @return The rotation in the current alliance frame
     */
    public static Rotation2d apply(Rotation2d rotation) {
        return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
    }

    /**
     * Applies alliance-specific transformation to a 2D pose.
     *
     * @param pose The pose in blue alliance frame
     * @return The pose in the current alliance frame
     */
    public static Pose2d apply(Pose2d pose) {
        return shouldFlip()
            ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
            : pose;
    }

    /**
     * Applies alliance-specific transformation to a 3D translation.
     *
     * @param translation The 3D translation in blue alliance frame
     * @return The 3D translation in the current alliance frame
     */
    public static Translation3d apply(Translation3d translation) {
        return new Translation3d(
            applyX(translation.getX()), applyY(translation.getY()), translation.getZ());
    }

    /**
     * Applies alliance-specific transformation to a 3D rotation.
     *
     * @param rotation The 3D rotation in blue alliance frame
     * @return The 3D rotation in the current alliance frame
     */
    public static Rotation3d apply(Rotation3d rotation) {
        return shouldFlip() ? rotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI)) : rotation;
    }

    /**
     * Applies alliance-specific transformation to a 3D pose.
     *
     * @param pose The 3D pose in blue alliance frame
     * @return The 3D pose in the current alliance frame
     */
    public static Pose3d apply(Pose3d pose) {
        return new Pose3d(apply(pose.getTranslation()), apply(pose.getRotation()));
    }

    /**
     * Determines if field coordinates should be flipped based on alliance color.
     *
     * @return True if on red alliance, false if on blue alliance
     */
    public static boolean shouldFlip() {
        return !Constants.disableHAL
            && DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }



    // /**
    // * Whether or not a pose needs to be flipped
    // *
    // * @return True if the robot is on the red alliance, false if on the blue alliance.
    // */
    // public static boolean shouldFlip()
    // {
    // return DriverStation.getAlliance().isPresent()
    // && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    // }

    // public static Pose2d handleAllianceFlip(Pose2d blue_pose)
    // {
    // return shouldFlip() ? blue_pose.rotateAround(fieldCenter, Rotation2d.k180deg)
    // : blue_pose;
    // }

    // public static Translation2d handleAllianceFlip(Translation2d blue_translation)
    // {
    // return shouldFlip() ? blue_translation.rotateAround(fieldCenter,
    // Rotation2d.k180deg) : blue_translation;
    // }

    // public static Rotation2d handleAllianceFlip(Rotation2d blue_rotation)
    // {
    // return shouldFlip() ? blue_rotation.plus(Rotation2d.k180deg) : blue_rotation;
    // }

    // public static Translation3d handleAllianceFlip(Translation3d blue_translation)
    // {
    // return shouldFlip()
    // ? blue_translation.rotateAround(fieldCenter3D, new Rotation3d(0, 0, 180.00))
    // : blue_translation;
    // }

    // /**
    // * Checks if a given pose is within the field boundaries, with a specified margin.
    // *
    // * @param translation The tranlslation to check.
    // * @param margin The distance inset from the boundaries.
    // * @return True if the pose is within the field boundaries, false otherwise.
    // */
    // public static boolean isPoseInField(Translation2d translation, Distance margin)
    // {
    // return translation.getX() >= margin.in(Meters)
    // && translation.getX() <= FieldConstants.FIELD_LENGTH - margin.in(Meters)
    // && translation.getY() >= margin.in(Meters)
    // && translation.getY() <= FieldConstants.FIELD_WIDTH - margin.in(Meters);
    // }
}
