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

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;

import frc.lib.devices.CamCamCoproc;
import frc.lib.devices.CamCamCoproc.CameraProperties;
import frc.lib.io.vision.VisionIO;
import frc.lib.io.vision.VisionIOCamCam;
import frc.lib.io.vision.VisionIOCamCamSim;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.objectdetector.ObjectDetector;
import frc.robot.subsystems.objectdetector.ObjectDetectorConstants;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;

/** Shared CamCam configuration for AprilTag localization and object detection. */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class VisionConstants {
    /**
     * NetworkTables subtable name for the deployed CamCam instance.
     *
     * <p>Change this if the coprocessor publishes under a different instance name.
     */
    public static final String COPROCESSOR_INSTANCE_NAME = "main";

    public static final String FRONT_NAME = "front";
    public static final String BACK_NAME = "back";

    public static final Transform3d FRONT_TRANSFORM =
            new Transform3d(
                    Units.inchesToMeters(-7.5),
                    Units.inchesToMeters(12.00),
                    Units.inchesToMeters(19.95),
                    new Rotation3d(
                            0.0,
                            Units.degreesToRadians(-31.973),
                            Units.degreesToRadians(-8.965230)));

    public static final Transform3d BACK_TRANSFORM =
            new Transform3d(
                    Units.inchesToMeters(-12.00),
                    Units.inchesToMeters(-9.50),
                    Units.inchesToMeters(17.00),
                    new Rotation3d(
                            0.0, Units.degreesToRadians(-18.173), Units.degreesToRadians(166.577)));

    public static final Matrix<N3, N3> FRONT_MATRIX =
            MatBuilder.fill(
                    Nat.N3(),
                    Nat.N3(),
                    1380.531127613026,
                    0.0,
                    765.2809506558299,
                    0.0,
                    1381.5640806035888,
                    626.9982046388377,
                    0.0,
                    0.0,
                    1.0);

    public static final Vector<N8> FRONT_DIST_COEFFS =
            VecBuilder.fill(
                    -0.029800389169817133,
                    0.011335486937787855,
                    -0.0007758929871136083,
                    -0.000979966348825576,
                    0.004322943914551887,
                    0.000977625446561076,
                    -0.001486288707185138,
                    0.0003931042262115059);

    public static final Matrix<N3, N3> BACK_MATRIX =
            MatBuilder.fill(
                    Nat.N3(),
                    Nat.N3(),
                    1994.3346993932607,
                    0,
                    772.1194963312653,
                    0,
                    1990.5668437725847,
                    559.9647344141187,
                    0,
                    0,
                    1);

    public static final Vector<N8> BACK_DIST_COEFFS =
            VecBuilder.fill(
                    0.10622095692661696,
                    -0.05338512237989565,
                    -0.0030328707104387877,
                    0.0003234715047390942,
                    -0.3410348671624887,
                    -0.005797616380180348,
                    0.0033869449584776738,
                    0.013942594262017396);

    public static final int FRONT_RESOLUTION_WIDTH = 1600;
    public static final int FRONT_RESOLUTION_HEIGHT = 1304;
    public static final int BACK_RESOLUTION_WIDTH = 1600;
    public static final int BACK_RESOLUTION_HEIGHT = 1304;

    public static final Angle FRONT_FOV = Degrees.of(80);
    public static final Angle BACK_FOV = Degrees.of(55);

    public static final double FRONT_FPS = 22;
    public static final double BACK_FPS = 22;

    public static final Time FRONT_LATENCY = Milliseconds.of(30);
    public static final Time BACK_LATENCY = Milliseconds.of(30);

    public static final Time FRONT_LATENCY_STDDEV = Milliseconds.of(5);
    public static final Time BACK_LATENCY_STDDEV = Milliseconds.of(5);

    public static final CameraProperties FRONT =
            new CameraProperties(
                    FRONT_NAME,
                    FRONT_TRANSFORM,
                    FRONT_MATRIX,
                    FRONT_DIST_COEFFS,
                    FRONT_RESOLUTION_WIDTH,
                    FRONT_RESOLUTION_HEIGHT,
                    FRONT_FOV,
                    FRONT_FPS,
                    FRONT_LATENCY,
                    FRONT_LATENCY_STDDEV);

    public static final CameraProperties BACK =
            new CameraProperties(
                    BACK_NAME,
                    BACK_TRANSFORM,
                    BACK_MATRIX,
                    BACK_DIST_COEFFS,
                    BACK_RESOLUTION_WIDTH,
                    BACK_RESOLUTION_HEIGHT,
                    BACK_FOV,
                    BACK_FPS,
                    BACK_LATENCY,
                    BACK_LATENCY_STDDEV);

    /**
     * Creates the robot's vision subsystem for the current runtime mode.
     *
     * @param objectDetector optional object detector to receive CamCam object frames
     * @return fully configured vision subsystem for real, sim, or replay operation
     */
    public static VisionSubsystem create(ObjectDetector objectDetector) {
        VisionIO io =
                switch (Constants.currentMode) {
                    case REAL -> new VisionIOCamCam(COPROCESSOR_INSTANCE_NAME);
                    case SIM ->
                            new VisionIOCamCamSim(
                                    new CameraProperties[] {FRONT, BACK},
                                    ObjectDetectorConstants.CAMERA0,
                                    FieldConstants.DEFAULT_APRIL_TAG_TYPE.getLayout(),
                                    () -> RobotState.getInstance().getOdometryPose(),
                                    ObjectDetectorConstants.OBJECT0_NAME,
                                    ObjectDetectorConstants.visionTargetSimSupplier);
                    case REPLAY -> new VisionIO() {};
                };

        CamCamCoproc coprocessor =
                new CamCamCoproc(
                        COPROCESSOR_INSTANCE_NAME,
                        io,
                        FieldConstants.DEFAULT_APRIL_TAG_TYPE.getLayout(),
                        new CameraProperties[] {FRONT, BACK},
                        ObjectDetectorConstants.CAMERA0);
        return new VisionSubsystem(objectDetector, coprocessor);
    }
}
