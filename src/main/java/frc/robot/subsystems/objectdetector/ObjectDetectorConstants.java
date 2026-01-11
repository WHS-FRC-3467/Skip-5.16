// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.objectdetector;

import java.util.function.Supplier;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionTargetSim;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
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
    // Camera constants
    // Transform sign convention: +X -> towards other alliance's station, +Y -> towards center of
    // field from starting starboard edge, +theta -> right-hand rule. units: meters & degrees.
    // Object detection camera # 0
    public static final String CAMERA0_NAME = "Detection Camera #0";
    public static final Angle CAMERA0_ROLL = Units.Degrees.of(0.0);
    public static final Angle CAMERA0_PITCH = Units.Degrees.of(25.0);
    public static final Angle CAMERA0_YAW = Units.Degrees.of(0.0);
    public static final double CAMERA0_X = 0.30;
    public static final double CAMERA0_Y = -0.30;
    public static final double CAMERA0_Z = 1.0;
    public static Transform3d CAMERA0_TRANSFORM =
        new Transform3d(CAMERA0_X, CAMERA0_Y, CAMERA0_Z,
            new Rotation3d(CAMERA0_ROLL, CAMERA0_PITCH, CAMERA0_YAW));
    // Object detection camera # 1
    // ...

    // Sim constants
    // 2025 Simulated Algae Targets
    public final static String SIM_NAME = "Algae";
    public final static double algaeHeightMeters = 0.41;
    // Initialize fixed array of sim targets
    public static VisionTargetSim[] SIM_TARGETS = new VisionTargetSim[] {
            new VisionTargetSim(new Pose3d(3, 2, algaeHeightMeters / 2, new Rotation3d()),
                new TargetModel(algaeHeightMeters)),
            new VisionTargetSim(new Pose3d(7, 6, algaeHeightMeters / 2, new Rotation3d()),
                new TargetModel(algaeHeightMeters)),
            new VisionTargetSim(new Pose3d(12, 7, algaeHeightMeters / 2, new Rotation3d()),
                new TargetModel(algaeHeightMeters)),
            null,
    };
    // Dynamic supplier for moving sim targets
    public static Supplier<VisionTargetSim[]> visionTargetSimSupplier =
        () -> SIM_TARGETS = new VisionTargetSim[] {
                new VisionTargetSim(new Pose3d(3, 2, algaeHeightMeters / 2, new Rotation3d()),
                    new TargetModel(algaeHeightMeters)),
                new VisionTargetSim(new Pose3d(7, 6, algaeHeightMeters / 2, new Rotation3d()),
                    new TargetModel(algaeHeightMeters)),
                new VisionTargetSim(new Pose3d(12, 7, algaeHeightMeters / 2, new Rotation3d()),
                    new TargetModel(algaeHeightMeters)),
                new VisionTargetSim(
                    new Pose3d(16, 3.5 * Math.sin(0.25 * Math.PI * Timer.getFPGATimestamp()) + 4.1,
                        algaeHeightMeters / 2, new Rotation3d()),
                    new TargetModel(algaeHeightMeters)),
        };
    // 2026 Targets
    // ...

    // Robot runtime mode for use in roboRIO & AKit
    public static ObjectDetector get()
    {
        RobotState robotState = RobotState.getInstance();
        switch (Constants.currentMode) {
            case REAL:
                // Real IO, inputs = PhotonVision implementation of ObjectDetectionIO
                return new ObjectDetector(new ObjectDetectionIOPhotonVision(CAMERA0_NAME));
            case SIM:
                // Sim IO, inputs = sim implementation of ObjectionDetectionIO
                return new ObjectDetector(new ObjectDetectionIOSim(CAMERA0_NAME, CAMERA0_TRANSFORM,
                    () -> robotState.getEstimatedPose(), SIM_NAME, visionTargetSimSupplier));
            case REPLAY:
                // Replayed robot, use logged data for IO
                return new ObjectDetector(new ObjectDetectionIO() {});
            default:
                throw new IllegalStateException("Unrecognized Robot Mode");
        }
    }
}
