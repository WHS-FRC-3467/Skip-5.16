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

package frc.lib.devices;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.google.flatbuffers.FlatBufferBuilder;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.simulation.SimHooks;

import frc.lib.io.vision.VisionIO;
import frc.robot.generated.flatbuffers.CamCamConfiguration;
import frc.robot.generated.flatbuffers.CamCamData;
import frc.robot.generated.flatbuffers.CamCamProblem;
import frc.robot.generated.flatbuffers.CamCamProblemCameraDisconnect;
import frc.robot.generated.flatbuffers.CamCamProblemObjectDetectionCameraUnconfigured;
import frc.robot.generated.flatbuffers.ConfiguredData;
import frc.robot.generated.flatbuffers.Data;
import frc.robot.generated.flatbuffers.PoseObservation;
import frc.robot.generated.flatbuffers.PoseObservations;
import frc.robot.generated.flatbuffers.Problems;
import frc.robot.generated.flatbuffers.YOLOObservation;
import frc.robot.generated.flatbuffers.YOLOObservations;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.lang.reflect.Field;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.UUID;

class CamCamCoprocTest {

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
    void getUnreadResultsIgnoresEmptyPayload() {
        CamCamCoproc coproc =
                new CamCamCoproc(
                        "coproc-test-" + UUID.randomUUID(), queuedIo(new byte[] {}), fieldLayout());

        assertDoesNotThrow(() -> assertTrue(coproc.getUnreadResults().isEmpty()));
    }

    @Test
    void getUnreadResultsRaisesDecodeFailureAlertForWrongIdentifier()
            throws ReflectiveOperationException {
        CamCamCoproc coproc =
                new CamCamCoproc(
                        "coproc-test-" + UUID.randomUUID(),
                        queuedIo(new byte[] {0, 0, 0, 0, 0, 0, 0, 0}),
                        fieldLayout());

        assertDoesNotThrow(() -> assertTrue(coproc.getUnreadResults().isEmpty()));
        assertTrue(getAlert(coproc, "coprocessorPacketDecodeFailureAlert").get());
        assertFalse(getAlert(coproc, "coprocessorDisconnectAlert").get());
    }

    @Test
    void getUnreadResultsReturnsPoseOnlyObjectOnlyAndCombinedPackets()
            throws ReflectiveOperationException {
        CamCamCoproc coproc =
                new CamCamCoproc(
                        "coproc-test-" + UUID.randomUUID(),
                        queuedIo(
                                configuredData(1.0, null, null, false),
                                configuredData(null, 2.0, null, false),
                                configuredData(3.0, 4.0, null, false)),
                        fieldLayout());

        Optional<ConfiguredData[]> unreadResults = coproc.getUnreadResults();

        assertTrue(unreadResults.isPresent());
        assertEquals(3, unreadResults.get().length);

        ConfiguredData poseOnly = unreadResults.get()[0];
        assertNotNull(poseOnly.poseObservations());
        assertEquals(1.0, poseOnly.poseObservations().observations(0).timestamp());
        assertEquals(1.4, poseOnly.poseObservations().observations(0).averageDistance());
        assertEquals(0.05, poseOnly.poseObservations().observations(0).ambiguity());
        assertNull(poseOnly.objectDetectionObservations());

        ConfiguredData objectOnly = unreadResults.get()[1];
        assertNull(objectOnly.poseObservations());
        assertNotNull(objectOnly.objectDetectionObservations());
        assertEquals(2.0, objectOnly.objectDetectionObservations().observations(0).timestamp());
        assertEquals(2, objectOnly.objectDetectionObservations().observations(0).objects(0).id());
        assertEquals(
                2.1, objectOnly.objectDetectionObservations().observations(0).objects(0).pitch());
        assertEquals(
                2.2, objectOnly.objectDetectionObservations().observations(0).objects(0).yaw());
        assertEquals(
                2.3, objectOnly.objectDetectionObservations().observations(0).objects(0).area());

        ConfiguredData both = unreadResults.get()[2];
        assertNotNull(both.poseObservations());
        assertEquals(3.0, both.poseObservations().observations(0).timestamp());
        assertNotNull(both.objectDetectionObservations());
        assertEquals(4.0, both.objectDetectionObservations().observations(0).timestamp());
        assertFalse(getAlert(coproc, "coprocessorPacketDecodeFailureAlert").get());
    }

    @Test
    void latestPacketControlsCameraProblemState() throws ReflectiveOperationException {
        CamCamCoproc coproc =
                new CamCamCoproc(
                        "coproc-test-" + UUID.randomUUID(),
                        queuedIo(
                                configuredData(1.0, null, 0, false),
                                configuredData(2.0, null, null, false)),
                        fieldLayout(),
                        cameraProperties("camera-0"));

        Optional<ConfiguredData[]> unreadResults = coproc.getUnreadResults();

        assertTrue(unreadResults.isPresent());
        assertEquals(2, unreadResults.get().length);
        assertTrue(getHasReceivedCameraReport(coproc));
        assertTrue(getReportedCameraConnected(coproc)[0]);
    }

    @Test
    void constructorPublishesApriltagOnlyConfiguration() {
        RecordingVisionIO io = queuedIo();

        new CamCamCoproc(
                "coproc-test-" + UUID.randomUUID(),
                io,
                fieldLayout(),
                cameraProperties("camera-0"));

        CamCamConfiguration configuration = configurationFrom(io.lastConfiguration);
        assertEquals(1, configuration.apriltagCamerasLength());
        assertEquals(16.5, configuration.apriltagLayout().field().length());
        assertEquals(8.2, configuration.apriltagLayout().field().width());
        assertEquals(2, configuration.apriltagLayout().tagsLength());
        assertEquals(3, configuration.apriltagLayout().tags(0).id());
        assertEquals(7, configuration.apriltagLayout().tags(1).id());
        assertNull(configuration.objectDetectionCamera());

        var camera = configuration.apriltagCameras(0);
        assertEquals(120.0, camera.intrinsics().cameraMatrix(0));
        assertEquals(322.0, camera.intrinsics().cameraMatrix(2));
        assertEquals(0.8, camera.intrinsics().distortionCoefficients(7));
        assertEquals(1.0, camera.extrinsics().translation().x());
        assertEquals(2.0, camera.extrinsics().translation().y());
        assertEquals(3.0, camera.extrinsics().translation().z());
    }

    @Test
    void constructorPublishesConfigurationWithObjectDetectionCamera() {
        RecordingVisionIO io = queuedIo();

        new CamCamCoproc(
                "coproc-test-" + UUID.randomUUID(),
                io,
                fieldLayout(),
                new CamCamCoproc.CameraProperties[] {cameraProperties("camera-0")},
                cameraProperties("object-detection-camera"));

        CamCamConfiguration configuration = configurationFrom(io.lastConfiguration);
        assertNotNull(configuration.objectDetectionCamera());
        assertEquals(1, configuration.apriltagCamerasLength());
        assertEquals(120.0, configuration.objectDetectionCamera().intrinsics().cameraMatrix(0));
        assertEquals(
                0.8, configuration.objectDetectionCamera().intrinsics().distortionCoefficients(7));
    }

    @Test
    void constructorOmitsObjectDetectionCameraWhenNull() {
        RecordingVisionIO io = queuedIo();

        new CamCamCoproc(
                "coproc-test-" + UUID.randomUUID(),
                io,
                fieldLayout(),
                new CamCamCoproc.CameraProperties[] {cameraProperties("camera-0")},
                null);

        CamCamConfiguration configuration = configurationFrom(io.lastConfiguration);
        assertNull(configuration.objectDetectionCamera());
    }

    @Test
    void objectDetectionCameraUnconfiguredTripsWarningAlertOnly()
            throws ReflectiveOperationException {
        CamCamCoproc coproc =
                new CamCamCoproc(
                        "coproc-test-" + UUID.randomUUID(),
                        queuedIo(configuredData(null, null, null, true)),
                        fieldLayout(),
                        cameraProperties("camera-0"));

        coproc.getUnreadResults();
        SimHooks.stepTiming(0.6);
        coproc.getUnreadResults();

        Alert objectDetectionWarning = getAlert(coproc, "objectDetectionCameraUnconfiguredAlert");
        Alert coprocessorUnconfigured = getAlert(coproc, "coprocessorUnconfiguredAlert");

        assertEquals(Alert.AlertType.kWarning, objectDetectionWarning.getType());
        assertTrue(objectDetectionWarning.get());
        assertFalse(coprocessorUnconfigured.get());
    }

    private RecordingVisionIO queuedIo(byte[]... payloads) {
        return new RecordingVisionIO(payloads);
    }

    private CamCamConfiguration configurationFrom(byte[] serializedConfiguration) {
        ByteBuffer buffer = ByteBuffer.wrap(serializedConfiguration);
        assertTrue(CamCamConfiguration.CamCamConfigurationBufferHasIdentifier(buffer));
        buffer.rewind();
        return CamCamConfiguration.getRootAsCamCamConfiguration(buffer);
    }

    private byte[] configuredData(
            Double poseTimestamp,
            Double yoloTimestamp,
            Integer disconnectedCameraId,
            boolean objectDetectionCameraUnconfigured) {
        FlatBufferBuilder builder = new FlatBufferBuilder(0);

        int poseObservations = poseTimestamp != null ? poseObservations(builder, poseTimestamp) : 0;
        int objectDetectionObservations =
                yoloTimestamp != null ? yoloObservations(builder, yoloTimestamp) : 0;
        int problems = problems(builder, disconnectedCameraId, objectDetectionCameraUnconfigured);
        int configuredData =
                ConfiguredData.createConfiguredData(
                        builder, poseObservations, objectDetectionObservations, problems);
        int camCamData = CamCamData.createCamCamData(builder, Data.ConfiguredData, configuredData);
        CamCamData.finishCamCamDataBuffer(builder, camCamData);

        return builder.sizedByteArray();
    }

    private int poseObservations(FlatBufferBuilder builder, double timestamp) {
        PoseObservations.startObservationsVector(builder, 1);
        PoseObservation.createPoseObservation(
                builder, timestamp, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, timestamp + 0.4, 0.05, 1);
        int observationsVector = builder.endVector();
        return PoseObservations.createPoseObservations(builder, observationsVector);
    }

    private int yoloObservations(FlatBufferBuilder builder, double timestamp) {
        YOLOObservation.startObjectsVector(builder, 1);
        frc.robot.generated.flatbuffers.ObjectDetection.createObjectDetection(
                builder, (short) timestamp, timestamp + 0.1, timestamp + 0.2, timestamp + 0.3);
        int objectsVector = builder.endVector();
        int yoloObservation =
                YOLOObservation.createYOLOObservation(builder, timestamp, objectsVector);
        int observationsVector =
                YOLOObservations.createObservationsVector(builder, new int[] {yoloObservation});
        return YOLOObservations.createYOLOObservations(builder, observationsVector);
    }

    private int problems(
            FlatBufferBuilder builder,
            Integer disconnectedCameraId,
            boolean objectDetectionCameraUnconfigured) {
        if (disconnectedCameraId == null && !objectDetectionCameraUnconfigured) {
            return 0;
        }

        ArrayList<Byte> problemTypes = new ArrayList<>();
        ArrayList<Integer> problemValues = new ArrayList<>();

        if (disconnectedCameraId != null) {
            int disconnectProblem =
                    CamCamProblemCameraDisconnect.createCamCamProblemCameraDisconnect(
                            builder, (byte) disconnectedCameraId.intValue());
            problemTypes.add(CamCamProblem.CamCamProblemCameraDisconnect);
            problemValues.add(disconnectProblem);
        }

        if (objectDetectionCameraUnconfigured) {
            CamCamProblemObjectDetectionCameraUnconfigured
                    .startCamCamProblemObjectDetectionCameraUnconfigured(builder);
            int objectDetectionProblem =
                    CamCamProblemObjectDetectionCameraUnconfigured
                            .endCamCamProblemObjectDetectionCameraUnconfigured(builder);
            problemTypes.add(CamCamProblem.CamCamProblemObjectDetectionCameraUnconfigured);
            problemValues.add(objectDetectionProblem);
        }

        byte[] serializedProblemTypes = new byte[problemTypes.size()];
        int[] serializedProblemValues = new int[problemValues.size()];
        for (int i = 0; i < problemTypes.size(); i++) {
            serializedProblemTypes[i] = problemTypes.get(i);
            serializedProblemValues[i] = problemValues.get(i);
        }

        int problemTypesVector = Problems.createProblemsTypeVector(builder, serializedProblemTypes);
        int problemValuesVector = Problems.createProblemsVector(builder, serializedProblemValues);
        return Problems.createProblems(builder, problemTypesVector, problemValuesVector);
    }

    private CamCamCoproc.CameraProperties cameraProperties(String name) {
        return new CamCamCoproc.CameraProperties(
                name,
                new Transform3d(1.0, 2.0, 3.0, new Rotation3d(0.1, 0.2, 0.3)),
                MatBuilder.fill(
                        Nat.N3(), Nat.N3(), 120.0, 0.0, 322.0, 0.0, 121.0, 241.0, 0.0, 0.0, 1.0),
                VecBuilder.fill(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8),
                0,
                0,
                null,
                0.0,
                null,
                null);
    }

    private AprilTagFieldLayout fieldLayout() {
        return new AprilTagFieldLayout(
                List.of(
                        new AprilTag(7, new Pose3d(7.0, 8.0, 9.0, new Rotation3d(0.0, 0.1, 0.2))),
                        new AprilTag(3, new Pose3d(3.0, 4.0, 5.0, new Rotation3d(0.3, 0.2, 0.1)))),
                16.5,
                8.2);
    }

    private boolean[] getReportedCameraConnected(CamCamCoproc coproc)
            throws ReflectiveOperationException {
        Field field = CamCamCoproc.class.getDeclaredField("reportedCameraConnected");
        field.setAccessible(true);
        return (boolean[]) field.get(coproc);
    }

    private boolean getHasReceivedCameraReport(CamCamCoproc coproc)
            throws ReflectiveOperationException {
        Field field = CamCamCoproc.class.getDeclaredField("hasReceivedCameraReport");
        field.setAccessible(true);
        return field.getBoolean(coproc);
    }

    private Alert getAlert(CamCamCoproc coproc, String fieldName)
            throws ReflectiveOperationException {
        Field field = CamCamCoproc.class.getDeclaredField(fieldName);
        field.setAccessible(true);
        return (Alert) field.get(coproc);
    }

    private static class RecordingVisionIO implements VisionIO {
        private final byte[][] unreadData;
        private byte[] lastConfiguration = new byte[] {};

        RecordingVisionIO(byte[]... unreadData) {
            this.unreadData = unreadData;
        }

        @Override
        public void configure(byte[] configuration) {
            lastConfiguration = configuration;
        }

        @Override
        public void updateInputs(VisionIOInputs inputs) {
            inputs.unreadData = unreadData;
        }
    }
}
