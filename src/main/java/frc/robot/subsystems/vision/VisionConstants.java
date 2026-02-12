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
import java.util.Optional;
import org.photonvision.simulation.VisionSystemSim;
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
import frc.robot.Constants;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.RobotState;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;

/**
 * Configuration constants for the vision subsystem.
 *
 * <p>
 * Contains camera calibration data, mounting positions, and factory methods for creating vision
 * cameras. Each camera has:
 * <ul>
 * <li>Extrinsics: Physical mounting transform (position and orientation on robot)</li>
 * <li>Intrinsics: Camera matrix and distortion coefficients from calibration</li>
 * <li>Performance: Resolution, FPS, latency, and standard deviation factors</li>
 * </ul>
 *
 * <p>
 * Camera intrinsics should be recalibrated when cameras are changed or remounted. Use
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
        new Transform3d(Units.inchesToMeters(-7.5), Units.inchesToMeters(12.00),
            Units.inchesToMeters(19.95),
            new Rotation3d(0.0, Units.degreesToRadians(-31.973),
                Units.degreesToRadians(-8.965230)));
    public static final Transform3d LEFT_TRANSFORM =
        new Transform3d(Units.inchesToMeters(-8.00), Units.inchesToMeters(11.75),
            Units.inchesToMeters(17.0),
            new Rotation3d(0.0, Units.degreesToRadians(-18.173), Units.degreesToRadians(90.00)));
    public static final Transform3d RIGHT_TRANSFORM =
        new Transform3d(Units.inchesToMeters(-8.00), Units.inchesToMeters(-11.75),
            Units.inchesToMeters(17.0),
            new Rotation3d(0.0, Units.degreesToRadians(-18.173), Units.degreesToRadians(-90.00)));
    public static final Transform3d BACK_TRANSFORM =
        new Transform3d(Units.inchesToMeters(-12.00), Units.inchesToMeters(-9.50),
            Units.inchesToMeters(17.00),
            new Rotation3d(0.0, Units.degreesToRadians(-18.173),
                Units.degreesToRadians(-166.577)));

    // Intrinsics
    // ThriftyCam Default Calibrations
    public static final Matrix<N3, N3> FRONT_MATRIX =
        MatBuilder.fill(Nat.N3(), Nat.N3(),
            2002.948392331919,
            0.0,
            783.9099067246102,
            0.0,
            1999.0390684862123,
            662.7694019679813,
            0.0,
            0.0,
            1.0);

    public static final Vector<N8> FRONT_DIST_COEFFS = VecBuilder.fill(
        0.09905119793103302,
        -0.06388083628565337,
        3.87402720846368E-5,
        1.4421218015997156E-4,
        -0.16329892957216433,
        -0.004599206903333014,
        0.0029050841273878885,
        0.0067195798658376375);

    public static final Matrix<N3, N3> LEFT_MATRIX =
        MatBuilder.fill(Nat.N3(), Nat.N3(),
            2002.948392331919,
            0.0,
            783.9099067246102,
            0.0,
            1999.0390684862123,
            662.7694019679813,
            0.0,
            0.0,
            1.0);

    public static final Vector<N8> LEFT_DIST_COEFFS = VecBuilder.fill(
        0.09905119793103302,
        -0.06388083628565337,
        3.87402720846368E-5,
        1.4421218015997156E-4,
        -0.16329892957216433,
        -0.004599206903333014,
        0.0029050841273878885,
        0.0067195798658376375);

    public static final Matrix<N3, N3> RIGHT_MATRIX =
        MatBuilder.fill(Nat.N3(), Nat.N3(),
            2013.7145941329916,
            0.0,
            813.5600211516376,
            0.0,
            2010.9021854554633,
            705.7489358922749,
            0.0,
            0.0,
            1.0);

    public static final Vector<N8> RIGHT_DIST_COEFFS = VecBuilder.fill(
        0.10838581263006249,
        -0.11418498861043114,
        1.2747353334518889E-4,
        -5.523828072189691E-4,
        -0.08722021094520614,
        -0.004272598848412149,
        0.0049167243044280235,
        0.0035452581738189713);

    public static final Matrix<N3, N3> BACK_MATRIX =
        MatBuilder.fill(Nat.N3(), Nat.N3(),
            2013.7145941329916,
            0.0,
            813.5600211516376,
            0.0,
            2010.9021854554633,
            705.7489358922749,
            0.0,
            0.0,
            1.0);

    public static final Vector<N8> BACK_DIST_COEFFS = VecBuilder.fill(
        0.10838581263006249,
        -0.11418498861043114,
        1.2747353334518889E-4,
        -5.523828072189691E-4,
        -0.08722021094520614,
        -0.004272598848412149,
        0.0049167243044280235,
        0.0035452581738189713);

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

    private static VisionIOPhotonVision getFrontIOReal()
    {
        return new VisionIOPhotonVision(FRONT);
    }

    private static VisionIOPhotonVisionSim getFrontIOSim()
    {
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

    private static VisionIOPhotonVision getLeftIOReal()
    {
        return new VisionIOPhotonVision(LEFT);
    }

    private static VisionIOPhotonVisionSim getLeftIOSim()
    {
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

    private static VisionIOPhotonVision getRightIOReal()
    {
        return new VisionIOPhotonVision(
            RIGHT);
    }

    private static VisionIOPhotonVisionSim getRightIOSim()
    {
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

    private static VisionIOPhotonVision getBackIOReal()
    {
        return new VisionIOPhotonVision(BACK);
    }

    private static VisionIOPhotonVisionSim getBackIOSim()
    {
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
    public static void create()
    {
        switch (Constants.currentMode) {
            case REAL -> {
                var camera1 = new AprilTagCamera(FRONT, getFrontIOReal());
                var camera2 = new AprilTagCamera(LEFT, getLeftIOReal());
                var camera3 = new AprilTagCamera(RIGHT, getRightIOReal());
                var camera4 = new AprilTagCamera(BACK, getBackIOReal());
                new VisionSubsystem(camera1, camera2, camera3, camera4);
            }
            case SIM -> {
                var camera1 = new AprilTagCamera(FRONT, getFrontIOSim());
                var camera2 = new AprilTagCamera(LEFT, getLeftIOSim());
                var camera3 = new AprilTagCamera(RIGHT, getRightIOSim());
                var camera4 = new AprilTagCamera(BACK, getBackIOSim());
                new VisionSubsystem(camera1, camera2, camera3, camera4);
            }
            case REPLAY -> {
                var camera1 = new AprilTagCamera(FRONT, new VisionIO() {});
                var camera2 = new AprilTagCamera(LEFT, new VisionIO() {});
                var camera3 = new AprilTagCamera(RIGHT, new VisionIO() {});
                var camera4 = new AprilTagCamera(BACK, new VisionIO() {});
                new VisionSubsystem(camera1, camera2, camera3, camera4);
            }
        }
    }
}
