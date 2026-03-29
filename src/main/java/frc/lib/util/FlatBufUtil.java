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
package frc.lib.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;

/** Utility conversions from generated FlatBuffer geometry types to WPILib geometry types. */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class FlatBufUtil {
    /**
     * Converts a generated FlatBuffer rotation into WPILib form.
     *
     * @param rotation FlatBuffer rotation value
     * @return equivalent WPILib rotation
     */
    public static Rotation3d toWPILib(frc.robot.generated.flatbuffers.Rotation3d rotation) {
        return new Rotation3d(
                new Quaternion(rotation.w(), rotation.x(), rotation.y(), rotation.z()));
    }

    /**
     * Converts a generated FlatBuffer translation into WPILib form.
     *
     * @param translation FlatBuffer translation value
     * @return equivalent WPILib translation
     */
    public static Translation3d toWPILib(
            frc.robot.generated.flatbuffers.Translation3d translation) {
        return new Translation3d(translation.x(), translation.y(), translation.z());
    }

    /**
     * Converts a generated FlatBuffer pose into WPILib form.
     *
     * @param pose FlatBuffer pose value
     * @return equivalent WPILib pose
     */
    public static Pose3d toWPILib(frc.robot.generated.flatbuffers.Pose3d pose) {
        return new Pose3d(toWPILib(pose.translation()), toWPILib(pose.rotation()));
    }
}
