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

package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.intakeLinear.IntakeLinear;
import frc.robot.subsystems.intakeLinear.IntakeLinearConstants;
import frc.robot.subsystems.shooter.ShooterSuperstructure;

public record MechanismPosePublisher(IntakeLinear intake,ShooterSuperstructure shooter){private static final double DOOHICKEY_MAX_ROTATION_DEGREES=60;

public void update(){var hood=new Pose3d(
// Offset from origin (inverse of 1st component in
// ascope_assets/Robot_Alpha_2026/config.json)
new Translation3d(-0.1,0,0.48),
// Rotate by hood angle
new Rotation3d(Rotations.zero(),shooter.getHoodAngle(),Rotations.zero()));


var directionRot=new Rotation3d(Degrees.zero(),Degrees.of(8.3),Degrees.zero());

// Start with +X, scale by extension, then rotate that vector by 8.3 degrees.
var offset=new Translation3d(intake.getExtension(),Meters.zero(),Meters.zero()).rotateBy(directionRot);

// Apply ONLY a translation (identity rotation), so pose orientation does not change.
var linearSlide=Pose3d.kZero.transformBy(new Transform3d(offset,new Rotation3d()));

double linearExtensionPercent=(intake.getExtension().in(Inches)/IntakeLinearConstants.MAX_DISTANCE.in(Inches));

// Offset from origin (inverse of 3rd component in
// ascope_assets/Robot_Alpha_2026/config.json)
var doohickey=new Pose3d(0.192,0,0.246,Rotation3d.kZero).transformBy(new Transform3d(
// Move with linear slide
offset,
// Rotate along slide
new Rotation3d(Degrees.zero(),Degrees.of(DOOHICKEY_MAX_ROTATION_DEGREES*linearExtensionPercent),Degrees.zero())));

Logger.recordOutput("MechanismPoses",new Pose3d[]{hood,linearSlide,doohickey});}}
