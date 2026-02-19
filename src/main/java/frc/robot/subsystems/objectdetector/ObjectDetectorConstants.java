// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.objectdetector;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import frc.lib.devices.AprilTagCamera.CameraProperties;
import frc.lib.io.objectdetection.*;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.RobotSim;
import java.util.Comparator;
import java.util.function.Supplier;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionTargetSim;

/**
 * Configuration constants for the object detection subsystem.
 *
 * <p>Contains camera calibration data, mounting positions, and simulation targets for ML-based
 * object detection. Each camera has:
 *
 * <ul>
 *   <li>Extrinsics: Physical mounting transform (position and orientation on robot)
 *   <li>Intrinsics: Camera matrix and distortion coefficients from calibration
 *   <li>Performance: Resolution, FPS, latency, and standard deviation factors
 * </ul>
 *
 * <p>Used for detecting game pieces or other objects using PhotonVision's ML pipeline. In
 * simulation, uses configured target positions to test detection algorithms.
 */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class ObjectDetectorConstants {
    private static final RobotState robotState = RobotState.getInstance();
    // Object detection camera #0
    // Extrinsics
    // Transform sign convention: +X -> towards other alliance's station, +Y -> towards center of
    // field from starting starboard edge, +theta -> right-hand rule.
    public static final String CAMERA0_NAME = "Detection Camera #0";
    public static final Transform3d CAMERA0_TRANSFORM =
            new Transform3d(
                    Units.Inches.of(8),
                    Units.Inches.of(0),
                    Units.Inches.of(30),
                    new Rotation3d(
                            Units.Degrees.of(0.0), Units.Degrees.of(0.0), Units.Degrees.of(0.0)));

    // Intrinsics
    public static final int CAMERA0_RESOLUTION_WIDTH = 1600;
    public static final int CAMERA0_RESOLUTION_HEIGHT = 1304;
    // from Thrifty docs
    public static final Angle CAMERA0_FOV = Degrees.of(55);

    // ThriftyCam Default Calibrations
    public static final Matrix<N3, N3> CAMERA0_MATRIX =
            MatBuilder.fill(
                    Nat.N3(),
                    Nat.N3(),
                    2002.948392331919,
                    0.0,
                    783.9099067246102,
                    0.0,
                    1999.0390684862123,
                    662.7694019679813,
                    0.0,
                    0.0,
                    1.0);

    public static final Vector<N8> CAMERA0_DIST_COEFFS =
            VecBuilder.fill(
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
    public static final String OBJECT0_NAME = "FUEL";
    public static final double OBJECT0_HEIGHT_METERS = 0.150114;

    // Dynamic supplier for moving sim targets
    // Only publish FUEL to OBject Detection VisionSystemSim that are roughly within 8m of the
    // camera & its FOV -- publish nearest values first
    public static Supplier<VisionTargetSim[]> visionTargetSimSupplier =
            () -> {
                var robotPose = robotState.getEstimatedPose();
                var cameraPose =
                        robotPose.transformBy(
                                new Transform2d(
                                        CAMERA0_TRANSFORM.getX(),
                                        CAMERA0_TRANSFORM.getY(),
                                        CAMERA0_TRANSFORM.getRotation().toRotation2d()));
                var fuels = RobotSim.getInstance().getFuelSim().getFuelPoses();
                double halfFov = CAMERA0_FOV.in(Radians) / 2.0;
                return fuels.stream()
                        .filter(pose -> pose != null)
                        .filter(
                                pose ->
                                        pose.getTranslation()
                                                        .toTranslation2d()
                                                        .getDistance(cameraPose.getTranslation())
                                                < 8.0)
                        .filter(
                                pose -> {
                                    var fieldRel =
                                            pose.getTranslation()
                                                    .toTranslation2d()
                                                    .minus(cameraPose.getTranslation());
                                    var cameraRel =
                                            fieldRel.rotateBy(
                                                    cameraPose.getRotation().unaryMinus());
                                    if (cameraRel.getX() <= 0) {
                                        return false;
                                    }
                                    double yaw = Math.atan2(cameraRel.getY(), cameraRel.getX());
                                    return Math.abs(yaw) < halfFov;
                                })
                        .sorted(
                                Comparator.comparingDouble(
                                        pose ->
                                                pose.getTranslation()
                                                        .toTranslation2d()
                                                        .getDistance(cameraPose.getTranslation())))
                        .limit(15)
                        .map(
                                pose ->
                                        new VisionTargetSim(
                                                pose, new TargetModel(OBJECT0_HEIGHT_METERS)))
                        .toArray(VisionTargetSim[]::new);
            };

    /**
     * Creates and configures an ObjectDetector subsystem based on the current robot mode. Selects
     * the appropriate IO implementation (real hardware, simulation, or replay).
     *
     * @return a configured ObjectDetector instance
     */
    public static ObjectDetector get() {
        RobotState robotState = RobotState.getInstance();
        switch (Constants.currentMode) {
            case REAL:
                // Real IO, inputs = PhotonVision implementation of ObjectDetectionIO
                return new ObjectDetector(
                        CAMERA0_NAME, new ObjectDetectionIOPhotonVision(CAMERA0_NAME));
            case SIM:
                // Sim IO, inputs = sim implementation of ObjectionDetectionIO
                return new ObjectDetector(
                        CAMERA0_NAME,
                        new ObjectDetectionIOSim(
                                CAMERA0,
                                () -> robotState.getEstimatedPose(),
                                OBJECT0_NAME,
                                visionTargetSimSupplier));
            case REPLAY:
                // Replayed robot, use logged data for IO
                return new ObjectDetector(CAMERA0_NAME, new ObjectDetectionIO() {});
            default:
                throw new IllegalStateException("Unrecognized Robot Mode");
        }
    }
}
