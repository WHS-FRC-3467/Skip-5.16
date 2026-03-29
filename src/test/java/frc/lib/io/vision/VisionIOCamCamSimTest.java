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

package frc.lib.io.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.simulation.SimHooks;

import frc.lib.devices.CamCamCoproc;
import frc.robot.generated.flatbuffers.ConfiguredData;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.photonvision.simulation.VisionTargetSim;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Optional;
import java.util.UUID;

class VisionIOCamCamSimTest {
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
    void apriltagSimPublishesPoseObservations() {
        AprilTagFieldLayout layout =
                new AprilTagFieldLayout(
                        List.of(
                                new AprilTag(
                                        1,
                                        new Pose3d(
                                                3.0, 2.0, 1.0, new Rotation3d(0.0, 0.0, Math.PI)))),
                        10.0,
                        5.0);
        Pose2d robotPose = new Pose2d(1.0, 2.0, Rotation2d.kZero);
        CamCamCoproc.CameraProperties camera = cameraProperties();
        VisionIOCamCamSim io =
                new VisionIOCamCamSim(
                        new CamCamCoproc.CameraProperties[] {camera},
                        null,
                        layout,
                        () -> robotPose,
                        "fuel",
                        () -> new VisionTargetSim[0]);
        CamCamCoproc coprocessor =
                new CamCamCoproc(
                        "sim-test-" + UUID.randomUUID(),
                        io,
                        layout,
                        new CamCamCoproc.CameraProperties[] {camera},
                        null);

        Optional<ConfiguredData[]> unreadResults = Optional.empty();
        ConfiguredData posePacket = null;
        for (int i = 0; i < 50 && posePacket == null; i++) {
            SimHooks.stepTiming(0.05);
            unreadResults = coprocessor.getUnreadResults();
            if (unreadResults.isPresent()) {
                posePacket =
                        java.util.Arrays.stream(unreadResults.get())
                                .filter(packet -> packet.poseObservations() != null)
                                .findFirst()
                                .orElse(null);
            }
        }

        assertTrue(unreadResults.isPresent());
        assertTrue(unreadResults.get().length > 0);
        assertNotNull(posePacket);
        assertTrue(posePacket.poseObservations().observationsLength() > 0);
        assertTrue(posePacket.poseObservations().observations(0).averageDistance() > 0.0);
        assertNull(posePacket.objectDetectionObservations());
    }

    @Test
    void configuredSimPublishesHeartbeatPacketsWhenIdle() throws ReflectiveOperationException {
        AprilTagFieldLayout layout = new AprilTagFieldLayout(List.of(), 10.0, 5.0);
        Pose2d robotPose = new Pose2d(1.0, 2.0, Rotation2d.kZero);
        CamCamCoproc.CameraProperties camera = cameraProperties();
        VisionIOCamCamSim io =
                new VisionIOCamCamSim(
                        new CamCamCoproc.CameraProperties[] {camera},
                        null,
                        layout,
                        () -> robotPose,
                        "fuel",
                        () -> new VisionTargetSim[0]);
        CamCamCoproc coprocessor =
                new CamCamCoproc(
                        "sim-heartbeat-test-" + UUID.randomUUID(),
                        io,
                        layout,
                        new CamCamCoproc.CameraProperties[] {camera},
                        null);

        SimHooks.stepTiming(0.05);
        Optional<ConfiguredData[]> firstUnread = coprocessor.getUnreadResults();
        SimHooks.stepTiming(0.60);
        Optional<ConfiguredData[]> secondUnread = coprocessor.getUnreadResults();

        assertTrue(firstUnread.isPresent());
        assertEquals(1, firstUnread.get().length);
        assertNull(firstUnread.get()[0].poseObservations());
        assertNull(firstUnread.get()[0].objectDetectionObservations());

        assertTrue(secondUnread.isPresent());
        assertEquals(1, secondUnread.get().length);
        assertNull(secondUnread.get()[0].poseObservations());
        assertNull(secondUnread.get()[0].objectDetectionObservations());
        assertFalse(getAlert(coprocessor, "coprocessorDisconnectAlert").get());
    }

    private CamCamCoproc.CameraProperties cameraProperties() {
        return new CamCamCoproc.CameraProperties(
                "sim-apriltag-camera",
                new Transform3d(0.0, 0.0, 1.0, new Rotation3d()),
                MatBuilder.fill(
                        Nat.N3(), Nat.N3(), 600.0, 0.0, 640.0, 0.0, 600.0, 360.0, 0.0, 0.0, 1.0),
                VecBuilder.fill(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                1280,
                720,
                Degrees.of(90.0),
                20.0,
                Milliseconds.of(30.0),
                Milliseconds.of(5.0));
    }

    private Alert getAlert(CamCamCoproc coprocessor, String fieldName)
            throws ReflectiveOperationException {
        Field field = CamCamCoproc.class.getDeclaredField(fieldName);
        field.setAccessible(true);
        return (Alert) field.get(coprocessor);
    }
}
