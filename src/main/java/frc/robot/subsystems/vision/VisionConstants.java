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

@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class VisionConstants {
    // Extrinsics
    public static final String FRONT_LEFT_NAME = "front_left";
    public static final String FRONT_RIGHT_NAME = "front_right";

    public static final Transform3d FRONT_LEFT_TRANSFORM =
        new Transform3d(Units.inchesToMeters(9.287), Units.inchesToMeters(10.9704),
            Units.inchesToMeters(7.9167),
            new Rotation3d(0.0, Units.degreesToRadians(-15), Units.degreesToRadians(-30)));
    public static final Transform3d FRONT_RIGHT_TRANSFORM =
        new Transform3d(Units.inchesToMeters(9.287), Units.inchesToMeters(-10.9704),
            Units.inchesToMeters(7.9167),
            new Rotation3d(0.0, Units.degreesToRadians(-15), Units.degreesToRadians(30)));

    // Intrinsics
    // ThriftyCam Default Calibrations
    public static final Matrix<N3, N3> FRONT_LEFT_MATRIX =
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

    public static final Vector<N8> FRONT_LEFT_DIST_COEFFS = VecBuilder.fill(
        0.09905119793103302,
        -0.06388083628565337,
        3.87402720846368E-5,
        1.4421218015997156E-4,
        -0.16329892957216433,
        -0.004599206903333014,
        0.0029050841273878885,
        0.0067195798658376375);

    public static final Matrix<N3, N3> FRONT_RIGHT_MATRIX =
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

    public static final Vector<N8> FRONT_RIGHT_DIST_COEFFS = VecBuilder.fill(
        0.10838581263006249,
        -0.11418498861043114,
        1.2747353334518889E-4,
        -5.523828072189691E-4,
        -0.08722021094520614,
        -0.004272598848412149,
        0.0049167243044280235,
        0.0035452581738189713);

    public static final int FRONT_LEFT_RESOLUTION_WIDTH = 1600;
    public static final int FRONT_LEFT_RESOLUTION_HEIGHT = 1304;
    public static final int FRONT_RIGHT_RESOLUTION_WIDTH = 1600;
    public static final int FRONT_RIGHT_RESOLUTION_HEIGHT = 1304;

    public static final Angle FRONT_LEFT_FOV = Degrees.of(55); // from Thrifty docs
    public static final Angle FRONT_RIGHT_FOV = Degrees.of(55);

    // Performance
    public static final double FRONT_LEFT_FPS = 22;
    public static final double FRONT_RIGHT_FPS = 22;

    public static final double FRONT_LEFT_STDDEV_FACTOR = 1.0;
    public static final double FRONT_RIGHT_STDDEV_FACTOR = 1.0;

    // Exposure 5 ms, USB 5 ms, detection 15 ms, scheduling 5 ms
    public static final Time FRONT_LEFT_LATENCY = Milliseconds.of(30);
    public static final Time FRONT_RIGHT_LATENCY = Milliseconds.of(30);

    public static final Time FRONT_LEFT_LATENCY_STDDEV = Milliseconds.of(5);
    public static final Time FRONT_RIGHT_LATENCY_STDDEV = Milliseconds.of(5);

    public static final CameraProperties FRONT_LEFT =
        new CameraProperties(
            FRONT_LEFT_NAME,
            FRONT_LEFT_TRANSFORM,
            FRONT_LEFT_MATRIX,
            FRONT_LEFT_DIST_COEFFS,
            FRONT_LEFT_RESOLUTION_WIDTH,
            FRONT_LEFT_RESOLUTION_HEIGHT,
            FRONT_LEFT_STDDEV_FACTOR,
            FRONT_LEFT_FOV,
            FRONT_LEFT_FPS,
            FRONT_LEFT_LATENCY,
            FRONT_LEFT_LATENCY_STDDEV);

    public static final CameraProperties FRONT_RIGHT =
        new CameraProperties(
            FRONT_RIGHT_NAME,
            FRONT_RIGHT_TRANSFORM,
            FRONT_RIGHT_MATRIX,
            FRONT_RIGHT_DIST_COEFFS,
            FRONT_RIGHT_RESOLUTION_WIDTH,
            FRONT_RIGHT_RESOLUTION_HEIGHT,
            FRONT_RIGHT_STDDEV_FACTOR,
            FRONT_RIGHT_FOV,
            FRONT_RIGHT_FPS,
            FRONT_RIGHT_LATENCY,
            FRONT_RIGHT_LATENCY_STDDEV);

    private static Optional<VisionSystemSim> visionSim = Optional.empty();

    private static VisionIOPhotonVision getFrontLeftIOReal()
    {
        return new VisionIOPhotonVision(FRONT_LEFT);
    }

    private static VisionIOPhotonVisionSim getFrontLeftIOSim()
    {
        if (visionSim.isEmpty()) {
            visionSim = Optional.of(new VisionSystemSim("main"));
            visionSim.get().addAprilTags(AprilTagLayoutType.OFFICIAL.getLayout());
        }

        return new VisionIOPhotonVisionSim(
            FRONT_LEFT,
            visionSim.get(),
            () -> RobotState.getInstance().getOdometryPose(),
            AprilTagLayoutType.OFFICIAL.getLayout());
    }

    private static VisionIOPhotonVision getFrontRightIOReal()
    {
        return new VisionIOPhotonVision(
            FRONT_RIGHT);
    }

    private static VisionIOPhotonVisionSim getFrontRightIOSim()
    {
        if (visionSim.isEmpty()) {
            visionSim = Optional.of(new VisionSystemSim("main"));
            visionSim.get().addAprilTags(AprilTagLayoutType.OFFICIAL.getLayout());
        }

        return new VisionIOPhotonVisionSim(
            FRONT_RIGHT,
            visionSim.get(),
            () -> RobotState.getInstance().getOdometryPose(),
            AprilTagLayoutType.OFFICIAL.getLayout());
    }

    /**
     * Creates and configures a VisionSubsystem with AprilTag cameras based on the current robot mode.
     * Instantiates cameras with appropriate IO implementations (real, sim, or replay).
     */
    public static void create()
    {
        switch (Constants.currentMode) {
            case REAL -> {
                var camera1 = new AprilTagCamera(FRONT_LEFT, getFrontLeftIOReal());
                var camera2 = new AprilTagCamera(FRONT_RIGHT, getFrontRightIOReal());
                new VisionSubsystem(camera1, camera2);
            }
            case SIM -> {
                var camera1 = new AprilTagCamera(FRONT_LEFT, getFrontLeftIOSim());
                var camera2 = new AprilTagCamera(FRONT_RIGHT, getFrontRightIOSim());
                new VisionSubsystem(camera1, camera2);
            }
            case REPLAY -> {
                var camera1 = new AprilTagCamera(FRONT_LEFT, new VisionIO() {});
                var camera2 = new AprilTagCamera(FRONT_RIGHT, new VisionIO() {});
                new VisionSubsystem(camera1, camera2);
            }
        }
    }
}
