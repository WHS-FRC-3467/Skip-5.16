// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.objectdetector;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;
import java.util.function.Supplier;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionTargetSim;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.devices.AprilTagCamera.CameraProperties;
import frc.lib.io.objectdetection.*;
import frc.robot.Constants;
import frc.robot.RobotState;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;

/*
 * Subsystem constants (e.g. names, transforms) for the various object detector cameras on the
 * robot. Used to create object detector subsystems within RobotContainer.
 */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class ObjectDetectorConstants {
    // Object detection camera #0
    // Extrinsics
    // Transform sign convention: +X -> towards other alliance's station, +Y -> towards center of
    // field from starting starboard edge, +theta -> right-hand rule.
    public static final String CAMERA0_NAME = "Detection Camera #0";
    public static final Transform3d CAMERA0_TRANSFORM =
        new Transform3d(Units.Inches.of(0.0), Units.Inches.of(0.0), Units.Inches.of(30),
            new Rotation3d(Units.Degrees.of(0.0), Units.Degrees.of(0.0), Units.Degrees.of(0.0)));

    // Intrinsics
    public static final int CAMERA0_RESOLUTION_WIDTH = 1600;
    public static final int CAMERA0_RESOLUTION_HEIGHT = 1304;
    // from Thrifty docs
    public static final Angle CAMERA0_FOV = Degrees.of(55);

    // ThriftyCam Default Calibrations
    public static final Matrix<N3, N3> CAMERA0_MATRIX =
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

    public static final Vector<N8> CAMERA0_DIST_COEFFS = VecBuilder.fill(
        0.09905119793103302,
        -0.06388083628565337,
        3.87402720846368E-5,
        1.4421218015997156E-4,
        -0.16329892957216433,
        -0.004599206903333014,
        0.0029050841273878885,
        0.0067195798658376375);

    // Performance
    public static final double CAMERA0_FPS = 22;

    public static final double CAMERA0_STDDEV_FACTOR = 1.0;

    // Exposure 5 ms, USB 5 ms, detection 15 ms, scheduling 5 ms
    public static final Time CAMERA0_LATENCY = Milliseconds.of(30);

    public static final Time CAMERA0_LATENCY_STDDEV = Milliseconds.of(5);

    public static final CameraProperties CAMERA0 =
        new CameraProperties(
            CAMERA0_NAME,
            CAMERA0_TRANSFORM,
            CAMERA0_MATRIX,
            CAMERA0_DIST_COEFFS,
            CAMERA0_RESOLUTION_WIDTH,
            CAMERA0_RESOLUTION_HEIGHT,
            CAMERA0_STDDEV_FACTOR,
            CAMERA0_FOV,
            CAMERA0_FPS,
            CAMERA0_LATENCY,
            CAMERA0_LATENCY_STDDEV);

    // Target constants
    public final static String OBJECT0_NAME = "FUEL";
    public final static double OBJECT0_HEIGHT_METERS = 0.150114;
    // Initialize fixed array of sim targets
    public static VisionTargetSim[] SIM_TARGETS = new VisionTargetSim[] {
            new VisionTargetSim(new Pose3d(3, 2, OBJECT0_HEIGHT_METERS / 2, new Rotation3d()),
                new TargetModel(OBJECT0_HEIGHT_METERS)),
            new VisionTargetSim(new Pose3d(7, 6, OBJECT0_HEIGHT_METERS / 2, new Rotation3d()),
                new TargetModel(OBJECT0_HEIGHT_METERS)),
            new VisionTargetSim(new Pose3d(12, 7, OBJECT0_HEIGHT_METERS / 2, new Rotation3d()),
                new TargetModel(OBJECT0_HEIGHT_METERS)),
            new VisionTargetSim(new Pose3d(13, 4, 10 * OBJECT0_HEIGHT_METERS / 2, new Rotation3d()),
                new TargetModel(10 * OBJECT0_HEIGHT_METERS)),
            null,
    };
    // Dynamic supplier for moving sim targets
    public static Supplier<VisionTargetSim[]> visionTargetSimSupplier =
        () -> SIM_TARGETS = new VisionTargetSim[] {
                new VisionTargetSim(new Pose3d(3, 2, OBJECT0_HEIGHT_METERS / 2, new Rotation3d()),
                    new TargetModel(OBJECT0_HEIGHT_METERS)),
                new VisionTargetSim(new Pose3d(7, 6, OBJECT0_HEIGHT_METERS / 2, new Rotation3d()),
                    new TargetModel(OBJECT0_HEIGHT_METERS)),
                new VisionTargetSim(new Pose3d(12, 7, OBJECT0_HEIGHT_METERS / 2, new Rotation3d()),
                    new TargetModel(OBJECT0_HEIGHT_METERS)),
                new VisionTargetSim(
                    new Pose3d(13, 4, 10 * OBJECT0_HEIGHT_METERS / 2, new Rotation3d()),
                    new TargetModel(10 * OBJECT0_HEIGHT_METERS)),
                new VisionTargetSim(
                    new Pose3d(16, 3.5 * Math.sin(0.25 * Math.PI * Timer.getFPGATimestamp()) + 4.1,
                        OBJECT0_HEIGHT_METERS / 2, new Rotation3d()),
                    new TargetModel(OBJECT0_HEIGHT_METERS)),
        };

    /**
     * Creates and configures an ObjectDetector subsystem based on the current robot mode.
     * Selects the appropriate IO implementation (real hardware, simulation, or replay).
     * 
     * @return a configured ObjectDetector instance
     */
    public static ObjectDetector get()
    {
        RobotState robotState = RobotState.getInstance();
        switch (Constants.currentMode) {
            case REAL:
                // Real IO, inputs = PhotonVision implementation of ObjectDetectionIO
                return new ObjectDetector(new ObjectDetectionIOPhotonVision(CAMERA0_NAME));
            case SIM:
                // Sim IO, inputs = sim implementation of ObjectionDetectionIO
                return new ObjectDetector(new ObjectDetectionIOSim(CAMERA0,
                    () -> robotState.getEstimatedPose(), OBJECT0_NAME, visionTargetSimSupplier));
            case REPLAY:
                // Replayed robot, use logged data for IO
                return new ObjectDetector(new ObjectDetectionIO() {});
            default:
                throw new IllegalStateException("Unrecognized Robot Mode");
        }
    }
}
