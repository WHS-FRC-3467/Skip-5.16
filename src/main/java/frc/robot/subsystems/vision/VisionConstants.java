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

import frc.lib.devices.AprilTagCamera;
import frc.lib.devices.AprilTagCamera.CameraProperties;
import frc.lib.io.vision.VisionIO;
import frc.lib.io.vision.VisionIOPhotonVision;
import frc.lib.io.vision.VisionIOPhotonVisionSim;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.RobotState;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;

import org.photonvision.simulation.VisionSystemSim;

import java.util.Optional;

/**
 * Configuration constants for the vision subsystem.
 *
 * <p>Contains camera calibration data, mounting positions, and factory methods for creating vision
 * cameras. Each camera has:
 *
 * <ul>
 *   <li>Extrinsics: Physical mounting transform (position and orientation on robot)
 *   <li>Intrinsics: Camera matrix and distortion coefficients from calibration
 *   <li>Performance: Resolution, FPS, latency, and standard deviation factors
 * </ul>
 *
 * <p>Camera intrinsics should be recalibrated when cameras are changed or remounted. Use
 * PhotonVision's calibration tool to generate new intrinsic matrices.
 */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class VisionConstants {
    // Extrinsics
    public static final String FRONT_NAME = "front";
    public static final String LEFT_NAME = "left";
    public static final String RIGHT_NAME = "right";
    public static final String BACK_NAME = "back";

    public static final Transform3d FRONT_TRANSFORM =
            new Transform3d(
                    Units.inchesToMeters(7.5),
                    Units.inchesToMeters(-12.00),
                    Units.inchesToMeters(19.95),
                    new Rotation3d(
                            0.0,
                            Units.degreesToRadians(-31.973),
                            Units.degreesToRadians(171.03477)));
    public static final Transform3d LEFT_TRANSFORM =
            new Transform3d(
                    Units.inchesToMeters(8.00),
                    Units.inchesToMeters(-11.75),
                    Units.inchesToMeters(17.0),
                    new Rotation3d(
                            0.0, Units.degreesToRadians(-18.173), Units.degreesToRadians(-90.00)));
    public static final Transform3d RIGHT_TRANSFORM =
            new Transform3d(
                    Units.inchesToMeters(8.00),
                    Units.inchesToMeters(11.75),
                    Units.inchesToMeters(17.0),
                    new Rotation3d(
                            0.0, Units.degreesToRadians(-18.173), Units.degreesToRadians(90.00)));
    public static final Transform3d BACK_TRANSFORM =
            new Transform3d(
                    Units.inchesToMeters(12.00),
                    Units.inchesToMeters(9.50),
                    Units.inchesToMeters(17.00),
                    new Rotation3d(
                            0.0, Units.degreesToRadians(-18.173), Units.degreesToRadians(-13.423)));

    // Intrinsics
    // ThriftyCam Default Calibrations
    // FRONT = 001
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
                    -.0007758929871136083,
                    -.000979966348825576,
                    0.004322943914551887,
                    .000977625446561076,
                    -0.001486288707185138,
                    .0003931042262115059);

    // LEFT = 002
    public static final Matrix<N3, N3> LEFT_MATRIX =
            MatBuilder.fill(
                    Nat.N3(),
                    Nat.N3(),
                    2000.0032334309244,
                    0,
                    813.8184817447087,
                    0,
                    1997.6334218966347,
                    688.2406381921382,
                    0,
                    0,
                    1);

    public static final Vector<N8> LEFT_DIST_COEFFS =
            VecBuilder.fill(
                    0.06485476013833127,
                    0.28421734688292644,
                    -0.000861882706123709,
                    0.0005123699898998346,
                    -1.0948497906268129,
                    -0.005476103701277743,
                    -0.0075079750325369385,
                    0.044951833493942085);

    // RIGHT = 004
    public static final Matrix<N3, N3> RIGHT_MATRIX =
            MatBuilder.fill(
                    Nat.N3(),
                    Nat.N3(),
                    1999.9919360266583,
                    0,
                    759.8976902097196,
                    0,
                    2000.756601602442,
                    610.5233343357824,
                    0,
                    0,
                    1);

    public static final Vector<N8> RIGHT_DIST_COEFFS =
            VecBuilder.fill(
                    0.10368101039813213,
                    -0.08495511830294374,
                    -0.001968724475841204,
                    -0.0005170525371013939,
                    -0.24440475107943974,
                    -0.005569982774319356,
                    0.0038802482316867243,
                    0.010015510605366526);

    // BACK = 003
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
    public static final int LEFT_RESOLUTION_WIDTH = 1600;
    public static final int LEFT_RESOLUTION_HEIGHT = 1304;
    public static final int RIGHT_RESOLUTION_WIDTH = 1600;
    public static final int RIGHT_RESOLUTION_HEIGHT = 1304;
    public static final int BACK_RESOLUTION_WIDTH = 1600;
    public static final int BACK_RESOLUTION_HEIGHT = 1304;

    public static final Angle FRONT_FOV = Degrees.of(80); // from Thrifty docs
    public static final Angle LEFT_FOV = Degrees.of(55);
    public static final Angle RIGHT_FOV = Degrees.of(55);
    public static final Angle BACK_FOV = Degrees.of(55);

    // Performance
    public static final double FRONT_FPS = 22;
    public static final double LEFT_FPS = 22;
    public static final double RIGHT_FPS = 22;
    public static final double BACK_FPS = 22;

    public static final double FRONT_STDDEV_FACTOR = 1.0;
    public static final double LEFT_STDDEV_FACTOR = 1.0;
    public static final double RIGHT_STDDEV_FACTOR = 1.0;
    public static final double BACK_STDDEV_FACTOR = 1.0;

    // Exposure 5 ms, USB 5 ms, detection 15 ms, scheduling 5 ms
    public static final Time FRONT_LATENCY = Milliseconds.of(30);
    public static final Time LEFT_LATENCY = Milliseconds.of(30);
    public static final Time RIGHT_LATENCY = Milliseconds.of(30);
    public static final Time BACK_LATENCY = Milliseconds.of(30);

    public static final Time FRONT_LATENCY_STDDEV = Milliseconds.of(5);
    public static final Time LEFT_LATENCY_STDDEV = Milliseconds.of(5);
    public static final Time RIGHT_LATENCY_STDDEV = Milliseconds.of(5);
    public static final Time BACK_LATENCY_STDDEV = Milliseconds.of(5);

    public static final CameraProperties FRONT =
            new CameraProperties(
                    FRONT_NAME,
                    FRONT_TRANSFORM,
                    FRONT_MATRIX,
                    FRONT_DIST_COEFFS,
                    FRONT_RESOLUTION_WIDTH,
                    FRONT_RESOLUTION_HEIGHT,
                    FRONT_STDDEV_FACTOR,
                    FRONT_FOV,
                    FRONT_FPS,
                    FRONT_LATENCY,
                    FRONT_LATENCY_STDDEV);

    public static final CameraProperties LEFT =
            new CameraProperties(
                    LEFT_NAME,
                    LEFT_TRANSFORM,
                    LEFT_MATRIX,
                    LEFT_DIST_COEFFS,
                    LEFT_RESOLUTION_WIDTH,
                    LEFT_RESOLUTION_HEIGHT,
                    LEFT_STDDEV_FACTOR,
                    LEFT_FOV,
                    LEFT_FPS,
                    LEFT_LATENCY,
                    LEFT_LATENCY_STDDEV);

    public static final CameraProperties RIGHT =
            new CameraProperties(
                    RIGHT_NAME,
                    RIGHT_TRANSFORM,
                    RIGHT_MATRIX,
                    RIGHT_DIST_COEFFS,
                    RIGHT_RESOLUTION_WIDTH,
                    RIGHT_RESOLUTION_HEIGHT,
                    RIGHT_STDDEV_FACTOR,
                    RIGHT_FOV,
                    RIGHT_FPS,
                    RIGHT_LATENCY,
                    RIGHT_LATENCY_STDDEV);

    public static final CameraProperties BACK =
            new CameraProperties(
                    BACK_NAME,
                    BACK_TRANSFORM,
                    BACK_MATRIX,
                    BACK_DIST_COEFFS,
                    BACK_RESOLUTION_WIDTH,
                    BACK_RESOLUTION_HEIGHT,
                    BACK_STDDEV_FACTOR,
                    BACK_FOV,
                    BACK_FPS,
                    BACK_LATENCY,
                    BACK_LATENCY_STDDEV);

    private static Optional<VisionSystemSim> visionSim = Optional.empty();

    private static VisionIOPhotonVision getFrontIOReal() {
        return new VisionIOPhotonVision(FRONT);
    }

    private static VisionIOPhotonVisionSim getFrontIOSim() {
        if (visionSim.isEmpty()) {
            visionSim = Optional.of(new VisionSystemSim("main"));
            visionSim.get().addAprilTags(AprilTagLayoutType.OFFICIAL.getLayout());
        }

        return new VisionIOPhotonVisionSim(
                FRONT,
                visionSim.get(),
                () -> RobotState.getInstance().getOdometryPose(),
                AprilTagLayoutType.OFFICIAL.getLayout());
    }

    private static VisionIOPhotonVision getLeftIOReal() {
        return new VisionIOPhotonVision(LEFT);
    }

    private static VisionIOPhotonVisionSim getLeftIOSim() {
        if (visionSim.isEmpty()) {
            visionSim = Optional.of(new VisionSystemSim("main"));
            visionSim.get().addAprilTags(AprilTagLayoutType.OFFICIAL.getLayout());
        }

        return new VisionIOPhotonVisionSim(
                LEFT,
                visionSim.get(),
                () -> RobotState.getInstance().getOdometryPose(),
                AprilTagLayoutType.OFFICIAL.getLayout());
    }

    private static VisionIOPhotonVision getRightIOReal() {
        return new VisionIOPhotonVision(RIGHT);
    }

    private static VisionIOPhotonVisionSim getRightIOSim() {
        if (visionSim.isEmpty()) {
            visionSim = Optional.of(new VisionSystemSim("main"));
            visionSim.get().addAprilTags(AprilTagLayoutType.OFFICIAL.getLayout());
        }

        return new VisionIOPhotonVisionSim(
                RIGHT,
                visionSim.get(),
                () -> RobotState.getInstance().getOdometryPose(),
                AprilTagLayoutType.OFFICIAL.getLayout());
    }

    private static VisionIOPhotonVision getBackIOReal() {
        return new VisionIOPhotonVision(BACK);
    }

    private static VisionIOPhotonVisionSim getBackIOSim() {
        if (visionSim.isEmpty()) {
            visionSim = Optional.of(new VisionSystemSim("main"));
            visionSim.get().addAprilTags(AprilTagLayoutType.OFFICIAL.getLayout());
        }

        return new VisionIOPhotonVisionSim(
                BACK,
                visionSim.get(),
                () -> RobotState.getInstance().getOdometryPose(),
                AprilTagLayoutType.OFFICIAL.getLayout());
    }

    /**
     * Creates and configures a VisionSubsystem with AprilTag cameras based on the current robot
     * mode. Instantiates cameras with appropriate IO implementations (real, sim, or replay).
     */
    public static void create() {
        // switch (Constants.currentMode) {
        //     case REAL -> {
        //         var camera1 = new AprilTagCamera(FRONT, getFrontIOReal());
        //         // var camera2 = new AprilTagCamera(LEFT, getLeftIOReal());
        //         // var camera3 = new AprilTagCamera(RIGHT, getRightIOReal());
        //         var camera4 = new AprilTagCamera(BACK, getBackIOReal());
        //         new VisionSubsystem(camera1, camera4);
        //     }
        //     case SIM -> {
        //         var camera1 = new AprilTagCamera(FRONT, getFrontIOSim());
        //         // var camera2 = new AprilTagCamera(LEFT, getLeftIOSim());
        //         // var camera3 = new AprilTagCamera(RIGHT, getRightIOSim());
        //         var camera4 = new AprilTagCamera(BACK, getBackIOSim());
        //         new VisionSubsystem(camera1, camera4);
        //     }
        //     case REPLAY -> {
        //         var camera1 = new AprilTagCamera(FRONT, new VisionIO() {});
        //         // var camera2 = new AprilTagCamera(LEFT, new VisionIO() {});
        //         // var camera3 = new AprilTagCamera(RIGHT, new VisionIO() {});
        //         var camera4 = new AprilTagCamera(BACK, new VisionIO() {});
        //         new VisionSubsystem(camera1, camera4);
        //     }
        // }

        var camera1 = new AprilTagCamera(FRONT, new VisionIO() {});
        // var camera2 = new AprilTagCamera(LEFT, new VisionIO() {});
        // var camera3 = new AprilTagCamera(RIGHT, new VisionIO() {});
        var camera4 = new AprilTagCamera(BACK, new VisionIO() {});
        new VisionSubsystem(camera1, camera4);
    }
}
