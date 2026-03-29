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

package frc.robot.subsystems.objectdetector;

import static edu.wpi.first.units.Units.Degrees;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.SimHooks;

import frc.lib.devices.ObjectDetection;
import frc.lib.devices.ObjectDetection.DetectionFrame;
import frc.lib.devices.ObjectDetection.DetectionTarget;
import frc.lib.devices.ObjectDetection.ObjectDetectionObservation;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Optional;

class ObjectDetectorTest {
    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));
        SimHooks.pauseTiming();
        SimHooks.restartTiming();
    }

    @AfterEach
    void tearDown() {
        SimHooks.resumeTiming();
    }

    @Test
    void acceptLatestFrameBuildsObjectObservationsAndSelectsPreferredTarget() {
        Pose2d bufferedPoseAtFrame = new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(30.0));
        ObjectDetector detector =
                new ObjectDetector(
                        ObjectDetectorConstants.CAMERA0_NAME,
                        () -> 1.0,
                        timestamp -> Optional.of(bufferedPoseAtFrame));
        DetectionTarget firstTarget = new DetectionTarget(17, -30.0, 5.0, 10.0);
        detector.acceptLatestFrame(
                new DetectionFrame(
                        1.0, List.of(firstTarget, new DetectionTarget(23, -20.0, -3.0, 40.0))));

        assertEquals(2, detector.getLatestObjectObservation().size());
        assertTrue(detector.getLatestObjectObservation().get(0).isPresent());
        assertEquals(17, detector.getLatestObjectObservation().get(0).get().objID());
        assertTrue(detector.getLatestObjectObservation().get(0).get().distance().isPresent());
        Pose2d expectedObjectPose =
                new ObjectDetection()
                        .getObjectObservation(
                                firstTarget,
                                ObjectDetectorConstants.CAMERA0_TRANSFORM,
                                ObjectDetectorConstants.OBJECT0_HEIGHT_METERS,
                                1,
                                0,
                                1,
                                0,
                                bufferedPoseAtFrame)
                        .flatMap(ObjectDetectionObservation::objectPose)
                        .orElseThrow();
        Pose2d actualObjectPose =
                detector.getLatestObjectObservation().get(0).get().objectPose().orElseThrow();
        assertEquals(expectedObjectPose.getX(), actualObjectPose.getX(), 1.0e-9);
        assertEquals(expectedObjectPose.getY(), actualObjectPose.getY(), 1.0e-9);
        assertTrue(detector.getLatestTargetObservation().isPresent());
        assertEquals(23, detector.getLatestTargetObservation().get().objID());
        assertEquals(40.0, detector.getLatestTargetObservation().get().area(), 1.0e-9);
        assertEquals(-3.0, detector.getLatestTargetObservation().get().yaw().in(Degrees), 1.0e-9);
    }

    @Test
    void acceptLatestFrameWithoutBufferedPoseLeavesObjectUnlocalized() {
        ObjectDetector detector =
                new ObjectDetector(
                        ObjectDetectorConstants.CAMERA0_NAME,
                        () -> 1.0,
                        timestamp -> Optional.empty());
        detector.acceptLatestFrame(
                new DetectionFrame(1.0, List.of(new DetectionTarget(31, -20.0, 4.0, 30.0))));

        assertEquals(1, detector.getLatestObjectObservation().size());
        assertTrue(detector.getLatestObjectObservation().get(0).isPresent());
        assertEquals(31, detector.getLatestObjectObservation().get(0).get().objID());
        assertTrue(detector.getLatestObjectObservation().get(0).get().distance().isEmpty());
        assertTrue(detector.getLatestObjectObservation().get(0).get().objectPose().isEmpty());
        assertTrue(detector.getLatestTargetObservation().isPresent());
    }

    @Test
    void periodicClearsStaleObservationsUsingFrameTimestamp() {
        double[] currentTimeSeconds = {1.30};
        ObjectDetector detector =
                new ObjectDetector(
                        ObjectDetectorConstants.CAMERA0_NAME,
                        () -> currentTimeSeconds[0],
                        timestamp -> Optional.of(Pose2d.kZero));
        detector.acceptLatestFrame(
                new DetectionFrame(1.0, List.of(new DetectionTarget(11, -20.0, 4.0, 30.0))));

        detector.periodic();

        assertTrue(detector.getLatestObjectObservation().isEmpty());
        assertTrue(detector.getLatestTargetObservation().isEmpty());
    }

    @Test
    void acceptLatestFrameIgnoresOutOfOrderOlderFrames() {
        ObjectDetector detector =
                new ObjectDetector(
                        ObjectDetectorConstants.CAMERA0_NAME,
                        () -> 2.0,
                        timestamp -> Optional.of(Pose2d.kZero));
        detector.acceptLatestFrame(
                new DetectionFrame(2.0, List.of(new DetectionTarget(5, -20.0, 4.0, 30.0))));
        detector.acceptLatestFrame(
                new DetectionFrame(1.0, List.of(new DetectionTarget(9, -10.0, -6.0, 50.0))));

        assertEquals(1, detector.getLatestObjectObservation().size());
        assertEquals(5, detector.getLatestObjectObservation().get(0).get().objID());
        assertTrue(detector.getLatestTargetObservation().isPresent());
        assertEquals(5, detector.getLatestTargetObservation().get().objID());
        assertEquals(4.0, detector.getLatestTargetObservation().get().yaw().in(Degrees), 1.0e-9);
    }
}
