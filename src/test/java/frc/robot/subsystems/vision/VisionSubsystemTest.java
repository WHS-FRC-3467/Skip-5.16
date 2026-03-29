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

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.google.flatbuffers.FlatBufferBuilder;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.simulation.SimHooks;

import frc.lib.devices.CamCamCoproc;
import frc.lib.io.vision.VisionIO;
import frc.robot.generated.flatbuffers.CamCamData;
import frc.robot.generated.flatbuffers.ConfiguredData;
import frc.robot.generated.flatbuffers.Data;
import frc.robot.generated.flatbuffers.PoseObservation;
import frc.robot.generated.flatbuffers.PoseObservations;
import frc.robot.generated.flatbuffers.YOLOObservation;
import frc.robot.generated.flatbuffers.YOLOObservations;
import frc.robot.subsystems.objectdetector.ObjectDetector;
import frc.robot.subsystems.objectdetector.ObjectDetectorConstants;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.List;

class VisionSubsystemTest {
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
    void periodicForwardsNewestObjectFrameToDetector() {
        ObjectDetector detector = new ObjectDetector(ObjectDetectorConstants.CAMERA0_NAME);
        CamCamCoproc coprocessor =
                new CamCamCoproc(
                        "vision-test",
                        queuedIo(configuredDataWithObjectFrames()),
                        emptyFieldLayout(),
                        new CamCamCoproc.CameraProperties[] {},
                        ObjectDetectorConstants.CAMERA0);
        VisionSubsystem vision = new VisionSubsystem(detector, coprocessor);

        vision.periodic();

        assertEquals(1, detector.getLatestObjectObservation().size());
        assertTrue(detector.getLatestObjectObservation().get(0).isPresent());
        assertEquals(7, detector.getLatestObjectObservation().get(0).get().objID());
        assertTrue(detector.getLatestTargetObservation().isPresent());
        assertEquals(7, detector.getLatestTargetObservation().get().objID());
        assertEquals(12.0, detector.getLatestTargetObservation().get().yaw().in(Degrees), 1.0e-9);
    }

    @Test
    void postFilterRejectsPoseAboveZTolerance() {
        assertTrue(VisionSubsystem.postFilter(new Pose3d(1.0, 1.0, 0.10, new Rotation3d())));
        assertFalse(VisionSubsystem.postFilter(new Pose3d(1.0, 1.0, 1.00, new Rotation3d())));
    }

    @Test
    void preFilterRejectsHighAmbiguitySingleTagObservation() {
        assertFalse(VisionSubsystem.preFilter(poseObservation(1.5, 0.25, 1)));
    }

    @Test
    void preFilterRejectsDistantObservation() {
        assertFalse(VisionSubsystem.preFilter(poseObservation(4.5, -1.0, 2)));
    }

    @Test
    void preFilterAcceptsReasonableMultiTagObservation() {
        assertTrue(VisionSubsystem.preFilter(poseObservation(1.5, -1.0, 2)));
    }

    @Test
    void toVisionPoseObservationUsesPoseQualityMetadata() {
        PoseObservation ambiguousSingleTag = poseObservation(1.5, 0.10, 1);
        PoseObservation strongerObservation = poseObservation(1.5, -1.0, 2);

        var ambiguousSingleTagVisionObservation =
                VisionSubsystem.toVisionPoseObservation(
                        ambiguousSingleTag, new Pose2d(1.0, 2.0, Rotation2d.kZero));
        var strongerVisionObservation =
                VisionSubsystem.toVisionPoseObservation(
                        strongerObservation, new Pose2d(1.0, 2.0, Rotation2d.kZero));

        assertEquals(1.5, ambiguousSingleTagVisionObservation.avgTagDistance(), 1.0e-9);
        assertEquals(1, ambiguousSingleTagVisionObservation.numTagsUsed());
        assertEquals(2, strongerVisionObservation.numTagsUsed());
        assertTrue(
                ambiguousSingleTagVisionObservation.linearStdDev()
                        > strongerVisionObservation.linearStdDev());
        assertTrue(
                ambiguousSingleTagVisionObservation.angularStdDev()
                        > strongerVisionObservation.angularStdDev());
    }

    private VisionIO queuedIo(byte[]... payloads) {
        return new VisionIO() {
            private int nextIndex = 0;

            @Override
            public void updateInputs(VisionIOInputs inputs) {
                if (nextIndex >= payloads.length) {
                    inputs.unreadData = new byte[][] {};
                    return;
                }

                inputs.unreadData = new byte[][] {payloads[nextIndex++]};
            }
        };
    }

    private byte[] configuredDataWithObjectFrames() {
        FlatBufferBuilder builder = new FlatBufferBuilder(0);
        int objectDetectionObservations = objectDetectionObservations(builder);
        int configuredData =
                ConfiguredData.createConfiguredData(builder, 0, objectDetectionObservations, 0);
        int camCamData = CamCamData.createCamCamData(builder, Data.ConfiguredData, configuredData);
        CamCamData.finishCamCamDataBuffer(builder, camCamData);
        return builder.sizedByteArray();
    }

    private PoseObservation poseObservation(double averageDistance, double ambiguity, int numTags) {
        FlatBufferBuilder builder = new FlatBufferBuilder(0);
        int poseObservations = poseObservations(builder, averageDistance, ambiguity, numTags);
        int configuredData = ConfiguredData.createConfiguredData(builder, poseObservations, 0, 0);
        int camCamData = CamCamData.createCamCamData(builder, Data.ConfiguredData, configuredData);
        CamCamData.finishCamCamDataBuffer(builder, camCamData);

        CamCamData data =
                CamCamData.getRootAsCamCamData(java.nio.ByteBuffer.wrap(builder.sizedByteArray()));
        ConfiguredData configuredDataView = new ConfiguredData();
        assertNotNull(data.data(configuredDataView));
        return configuredDataView.poseObservations().observations(0);
    }

    private int poseObservations(
            FlatBufferBuilder builder, double averageDistance, double ambiguity, int numTags) {
        PoseObservations.startObservationsVector(builder, 1);
        PoseObservation.createPoseObservation(
                builder,
                1.0,
                1.0,
                2.0,
                0.1,
                1.0,
                0.0,
                0.0,
                0.0,
                averageDistance,
                ambiguity,
                numTags);
        int observationsVector = builder.endVector();
        return PoseObservations.createPoseObservations(builder, observationsVector);
    }

    private int objectDetectionObservations(FlatBufferBuilder builder) {
        int olderObservation = yoloObservation(builder, 1.0, 4, 3.0, 5.0);
        int newerObservation = yoloObservation(builder, 2.0, 7, 12.0, 9.0);
        int observationsVector =
                YOLOObservations.createObservationsVector(
                        builder, new int[] {olderObservation, newerObservation});
        return YOLOObservations.createYOLOObservations(builder, observationsVector);
    }

    private int yoloObservation(
            FlatBufferBuilder builder, double timestamp, int id, double yawDegrees, double area) {
        YOLOObservation.startObjectsVector(builder, 1);
        frc.robot.generated.flatbuffers.ObjectDetection.createObjectDetection(
                builder, (short) id, -20.0, yawDegrees, area);
        int objectsVector = builder.endVector();
        return YOLOObservation.createYOLOObservation(builder, timestamp, objectsVector);
    }

    private AprilTagFieldLayout emptyFieldLayout() {
        return new AprilTagFieldLayout(List.of(), 16.5, 8.2);
    }
}
