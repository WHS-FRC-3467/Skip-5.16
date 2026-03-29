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

import com.google.flatbuffers.FlatBufferBuilder;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import frc.lib.io.vision.VisionIO;
import frc.lib.io.vision.VisionIOInputsAutoLogged;
import frc.robot.FieldConstants;
import frc.robot.generated.flatbuffers.CamCamConfiguration;
import frc.robot.generated.flatbuffers.CamCamData;
import frc.robot.generated.flatbuffers.CamCamProblem;
import frc.robot.generated.flatbuffers.CamCamProblemCameraDisconnect;
import frc.robot.generated.flatbuffers.ConfiguredData;
import frc.robot.generated.flatbuffers.Data;
import frc.robot.generated.flatbuffers.Problems;
import frc.robot.generated.flatbuffers.UnconfiguredError;

import org.littletonrobotics.junction.Logger;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

/**
 * Represents one CamCam coprocessor instance on the robot.
 *
 * <p>Handles interfacing with the {@link VisionIO} hardware layer, publishing AprilTag and optional
 * object-detection camera calibration, and reading configured runtime packets.
 */
public class CamCamCoproc {
    private final String instanceName;
    private final CameraProperties[] apriltagCameraProperties;
    private final CameraProperties objectDetectionCameraProperties;

    /**
     * Intrinsic &amp; observed properties describing the camera.
     *
     * @param name Unique name for the camera
     * @param robotToCamera Transform from the robot frame to the camera frame
     * @param cameraMatrix Intrinsic camera matrix
     * @param distCoeffs Distortion coefficients for the camera
     * @param resolutionWidth Camera resolution width in pixels
     * @param resolutionHeight Camera resolution height in pixels
     * @param fov Estimated FOV of camera
     * @param fps Estimate FPS of camera
     * @param latency Average latency of the camera (exposure -> network tables)
     * @param latencyStdDev Standard deviation of the camera latency
     */
    public record CameraProperties(
            String name,
            Transform3d robotToCamera,
            Matrix<N3, N3> cameraMatrix,
            Matrix<N8, N1> distCoeffs,
            int resolutionWidth,
            int resolutionHeight,
            Angle fov,
            double fps,
            Time latency,
            Time latencyStdDev) {}

    private final String logPath;
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    private final Alert coprocessorDisconnectAlert;
    private final Debouncer coprocessorDisconnectDebouncer = new Debouncer(0.5);

    private final Alert coprocessorUnconfiguredAlert;
    private final Alert coprocessorPacketDecodeFailureAlert;

    private final Alert[] cameraDisconnectAlerts;
    private final Debouncer[] cameraDisconnectDebouncers;
    private final boolean[] reportedCameraConnected;
    private final Alert objectDetectionCameraUnconfiguredAlert;
    private final Debouncer objectDetectionCameraUnconfiguredDebouncer = new Debouncer(0.5);
    private boolean reportedObjectDetectionCameraUnconfigured = false;

    private final Alert allCamerasDisconnected;
    private final Debouncer allCamerasDisconnectedDebouncer = new Debouncer(0.5);

    private boolean hasReceivedCameraReport = false;

    /**
     * Creates a coprocessor using the default AprilTag layout and no object-detection camera.
     *
     * @param instanceName unique identifier for this CamCam instance
     * @param io transport used to communicate with the coprocessor
     * @param apriltagCameras AprilTag cameras hosted by the coprocessor
     */
    public CamCamCoproc(String instanceName, VisionIO io, CameraProperties... apriltagCameras) {
        this(
                instanceName,
                io,
                FieldConstants.DEFAULT_APRIL_TAG_TYPE.getLayout(),
                apriltagCameras,
                null);
    }

    /**
     * Creates a coprocessor with a custom AprilTag layout and no object-detection camera.
     *
     * @param instanceName unique identifier for this CamCam instance
     * @param io transport used to communicate with the coprocessor
     * @param fieldLayout AprilTag layout sent to the coprocessor
     * @param apriltagCameras AprilTag cameras hosted by the coprocessor
     */
    public CamCamCoproc(
            String instanceName,
            VisionIO io,
            AprilTagFieldLayout fieldLayout,
            CameraProperties... apriltagCameras) {
        this(instanceName, io, fieldLayout, apriltagCameras, null);
    }

    /**
     * Creates a coprocessor using the default AprilTag layout and an optional object-detection
     * camera.
     *
     * @param instanceName unique identifier for this CamCam instance
     * @param io transport used to communicate with the coprocessor
     * @param apriltagCameras AprilTag cameras hosted by the coprocessor
     * @param objectDetectionCamera optional object-detection camera hosted by the coprocessor
     */
    public CamCamCoproc(
            String instanceName,
            VisionIO io,
            CameraProperties[] apriltagCameras,
            CameraProperties objectDetectionCamera) {
        this(
                instanceName,
                io,
                FieldConstants.DEFAULT_APRIL_TAG_TYPE.getLayout(),
                apriltagCameras,
                objectDetectionCamera);
    }

    /**
     * Creates a coprocessor with explicit AprilTag and object-detection camera configuration.
     *
     * @param instanceName unique identifier for this CamCam instance
     * @param io transport used to communicate with the coprocessor
     * @param fieldLayout AprilTag layout sent to the coprocessor
     * @param apriltagCameras AprilTag cameras hosted by the coprocessor
     * @param objectDetectionCamera optional object-detection camera hosted by the coprocessor
     */
    public CamCamCoproc(
            String instanceName,
            VisionIO io,
            AprilTagFieldLayout fieldLayout,
            CameraProperties[] apriltagCameras,
            CameraProperties objectDetectionCamera) {
        this.instanceName = instanceName;
        logPath = "CamCamInstances/" + instanceName;
        this.io = io;
        apriltagCameraProperties =
                Arrays.copyOf(
                        Objects.requireNonNull(apriltagCameras, "apriltagCameras"),
                        apriltagCameras.length);
        objectDetectionCameraProperties = objectDetectionCamera;

        coprocessorDisconnectAlert =
                new Alert(
                        "CamCam Coprocessor " + instanceName + " is Disconnected!",
                        AlertType.kError);

        coprocessorUnconfiguredAlert =
                new Alert(
                        "CamCam Coprocessor " + instanceName + " is Not Configured!",
                        AlertType.kError);

        coprocessorPacketDecodeFailureAlert =
                new Alert(
                        "CamCam Coprocessor "
                                + instanceName
                                + " Published an Invalid or Unsupported Packet!",
                        AlertType.kError);

        cameraDisconnectAlerts = new Alert[apriltagCameraProperties.length];
        cameraDisconnectDebouncers = new Debouncer[apriltagCameraProperties.length];
        reportedCameraConnected = new boolean[apriltagCameraProperties.length];
        for (int i = 0; i < apriltagCameraProperties.length; i++) {
            cameraDisconnectAlerts[i] =
                    new Alert(
                            "Camera " + apriltagCameraProperties[i].name() + " is Disconnected!",
                            AlertType.kWarning);
            cameraDisconnectDebouncers[i] = new Debouncer(0.5);
        }

        objectDetectionCameraUnconfiguredAlert =
                new Alert(
                        "Object Detection Camera on CamCam Coprocessor "
                                + instanceName
                                + " is Unconfigured!",
                        AlertType.kWarning);

        allCamerasDisconnected =
                new Alert(
                        "All CamCam Cameras on Coprocessor " + instanceName + " are Disconnected!",
                        AlertType.kError);

        configure(fieldLayout);
    }

    /**
     * Retrieves unread vision results from the camera.
     *
     * <p>Updates inputs from the {@link VisionIO}, processes them through the logger, and returns
     * all configured packets if unread data is available. Returns an empty {@link Optional} if the
     * coprocessor is disconnected, unconfigured, or has no unread configured packets.
     *
     * @return unread configured data packets if available, otherwise {@link Optional#empty()}
     */
    public Optional<ConfiguredData[]> getUnreadResults() {
        io.updateInputs(inputs);
        Logger.processInputs(logPath, inputs);

        if (inputs.unreadData.length == 0) {
            clearUnconfiguredErrorReasonLog();
            updateAlerts(false, false);
            return Optional.empty();
        }

        ArrayList<ConfiguredData> unreadResults = new ArrayList<>(inputs.unreadData.length);
        boolean hadPacketDecodeFailure = false;
        Optional<CamCamData> latestPacket = Optional.empty();
        for (int i = 0; i < inputs.unreadData.length; i++) {
            Optional<CamCamData> parsedPacket = parseDataBuffer(inputs.unreadData[i]);
            if (i == inputs.unreadData.length - 1) {
                latestPacket = parsedPacket;
            }

            if (parsedPacket.isEmpty()) {
                hadPacketDecodeFailure = true;
                continue;
            }

            extractConfiguredData(parsedPacket.get()).ifPresent(unreadResults::add);
        }

        applyLatestPacketState(latestPacket);
        updateAlerts(true, hadPacketDecodeFailure);
        if (unreadResults.isEmpty()) {
            return Optional.empty();
        }

        return Optional.of(unreadResults.toArray(ConfiguredData[]::new));
    }

    /**
     * Returns the configured instance name for logging and transport lookups.
     *
     * @return CamCam instance name
     */
    public String getInstanceName() {
        return instanceName;
    }

    /**
     * Re-publishes the camera and field configuration used by this coprocessor.
     *
     * @param fieldLayout AprilTag layout to send to the coprocessor
     */
    public void configure(AprilTagFieldLayout fieldLayout) {
        io.configure(serializeConfiguration(Objects.requireNonNull(fieldLayout)));
    }

    private Optional<CamCamData> parseDataBuffer(byte[] rawData) {
        if (rawData == null || rawData.length < 8) {
            return Optional.empty();
        }

        ByteBuffer bb = ByteBuffer.wrap(rawData);
        if (!CamCamData.CamCamDataBufferHasIdentifier(bb)) {
            return Optional.empty();
        }

        try {
            bb.rewind();
            return Optional.of(CamCamData.getRootAsCamCamData(bb));
        } catch (RuntimeException ignored) {
            return Optional.empty();
        }
    }

    private Optional<ConfiguredData> extractConfiguredData(CamCamData data) {
        if (data.dataType() != Data.ConfiguredData) {
            return Optional.empty();
        }

        ConfiguredData configuredData = new ConfiguredData();
        if (data.data(configuredData) == null) {
            return Optional.empty();
        }

        return Optional.of(configuredData);
    }

    private void applyLatestPacketState(Optional<CamCamData> parsedData) {
        clearUnconfiguredErrorReasonLog();
        hasReceivedCameraReport = false;
        Arrays.fill(reportedCameraConnected, true);
        reportedObjectDetectionCameraUnconfigured = false;

        if (parsedData.isEmpty()) {
            return;
        }

        CamCamData data = parsedData.get();
        byte type = data.dataType();
        if (type == Data.UnconfiguredError) {
            coprocessorUnconfiguredAlert.set(true);
            logUnconfiguredErrorReason(data);
            return;
        }

        if (type != Data.ConfiguredData) {
            return;
        }

        ConfiguredData configuredData = new ConfiguredData();
        if (data.data(configuredData) == null) {
            return;
        }

        processProblems(configuredData.problems());
        hasReceivedCameraReport = true;
    }

    private void clearUnconfiguredErrorReasonLog() {
        coprocessorUnconfiguredAlert.set(false);
        Logger.recordOutput(logPath + "/HasUnconfiguredErrorReason", false);
        Logger.recordOutput(logPath + "/UnconfiguredErrorReason", "");
    }

    private void logUnconfiguredErrorReason(CamCamData data) {
        UnconfiguredError unconfiguredError = new UnconfiguredError();
        if (data.data(unconfiguredError) == null) {
            clearUnconfiguredErrorReasonLog();
            return;
        }

        String reason = unconfiguredError.reason();
        Logger.recordOutput(logPath + "/HasUnconfiguredErrorReason", reason != null);
        Logger.recordOutput(
                logPath + "/UnconfiguredErrorReason", Optional.ofNullable(reason).orElse(""));
    }

    private void processProblems(Problems problems) {
        if (problems == null) return;

        for (int i = 0; i < problems.problemsLength(); i++) {
            byte problemType = problems.problemsType(i);

            if (problemType == CamCamProblem.NONE) {
                continue;
            } else if (problemType == CamCamProblem.CamCamProblemCameraDisconnect) {
                CamCamProblemCameraDisconnect disconnect = new CamCamProblemCameraDisconnect();
                problems.problems(disconnect, i);

                processCameraDisconnect(disconnect);
            } else if (problemType
                    == CamCamProblem.CamCamProblemObjectDetectionCameraUnconfigured) {
                reportedObjectDetectionCameraUnconfigured = true;
            }
        }
    }

    private void processCameraDisconnect(CamCamProblemCameraDisconnect problem) {
        int cameraId = Byte.toUnsignedInt(problem.id());
        if (cameraId >= reportedCameraConnected.length) {
            return;
        }

        reportedCameraConnected[cameraId] = false;
    }

    private void updateAlerts(boolean rawDataAvailable, boolean packetDecodeFailure) {
        boolean coprocessorDisconnected =
                coprocessorDisconnectDebouncer.calculate(!rawDataAvailable);
        boolean coprocessorConnected = !coprocessorDisconnected;

        coprocessorDisconnectAlert.set(coprocessorDisconnected);
        coprocessorPacketDecodeFailureAlert.set(coprocessorConnected && packetDecodeFailure);

        boolean objectDetectionCameraUnconfigured =
                coprocessorConnected
                        && hasReceivedCameraReport
                        && reportedObjectDetectionCameraUnconfigured;
        objectDetectionCameraUnconfiguredAlert.set(
                objectDetectionCameraUnconfiguredDebouncer.calculate(
                        objectDetectionCameraUnconfigured));

        boolean allDisconnected =
                coprocessorConnected
                        && hasReceivedCameraReport
                        && cameraDisconnectAlerts.length > 0;
        for (int i = 0; i < cameraDisconnectAlerts.length; i++) {
            allDisconnected &= !reportedCameraConnected[i];
        }

        boolean allCamerasDisconnectedActive =
                allCamerasDisconnectedDebouncer.calculate(allDisconnected);
        allCamerasDisconnected.set(allCamerasDisconnectedActive);

        for (int i = 0; i < cameraDisconnectAlerts.length; i++) {
            if (allCamerasDisconnectedActive) {
                cameraDisconnectAlerts[i].set(false);
                continue;
            }

            boolean cameraDisconnected =
                    coprocessorConnected && hasReceivedCameraReport && !reportedCameraConnected[i];
            cameraDisconnectAlerts[i].set(
                    cameraDisconnectDebouncers[i].calculate(cameraDisconnected));
        }
    }

    private byte[] serializeConfiguration(AprilTagFieldLayout fieldLayout) {
        FlatBufferBuilder builder = new FlatBufferBuilder(0);
        int layoutOffset = serializeFieldLayout(builder, fieldLayout);
        int apriltagCamerasOffset = serializeApriltagCameras(builder);
        CamCamConfiguration.startCamCamConfiguration(builder);
        CamCamConfiguration.addApriltagCameras(builder, apriltagCamerasOffset);
        CamCamConfiguration.addApriltagLayout(builder, layoutOffset);
        if (objectDetectionCameraProperties != null) {
            int objectDetectionCamera = serializeCamera(builder, objectDetectionCameraProperties);
            CamCamConfiguration.addObjectDetectionCamera(builder, objectDetectionCamera);
        }
        int configuration = CamCamConfiguration.endCamCamConfiguration(builder);
        CamCamConfiguration.finishCamCamConfigurationBuffer(builder, configuration);
        return builder.sizedByteArray();
    }

    private int serializeApriltagCameras(FlatBufferBuilder builder) {
        CamCamConfiguration.startApriltagCamerasVector(builder, apriltagCameraProperties.length);
        for (int i = apriltagCameraProperties.length - 1; i >= 0; i--) {
            serializeCamera(builder, apriltagCameraProperties[i]);
        }
        return builder.endVector();
    }

    private int serializeCamera(FlatBufferBuilder builder, CameraProperties property) {
        Transform3d robotToCamera =
                Objects.requireNonNull(
                        property.robotToCamera(),
                        "CamCam camera " + property.name() + " is missing robotToCamera");
        Matrix<N3, N3> cameraMatrix =
                Objects.requireNonNull(
                        property.cameraMatrix(),
                        "CamCam camera " + property.name() + " is missing cameraMatrix");
        Matrix<N8, N1> distCoeffs =
                Objects.requireNonNull(
                        property.distCoeffs(),
                        "CamCam camera " + property.name() + " is missing distCoeffs");

        var quaternion = robotToCamera.getRotation().getQuaternion();
        return frc.robot.generated.flatbuffers.Camera.createCamera(
                builder,
                serializeDistortionCoefficients(distCoeffs),
                serializeCameraMatrix(cameraMatrix),
                robotToCamera.getX(),
                robotToCamera.getY(),
                robotToCamera.getZ(),
                quaternion.getW(),
                quaternion.getX(),
                quaternion.getY(),
                quaternion.getZ());
    }

    private double[] serializeCameraMatrix(Matrix<N3, N3> cameraMatrix) {
        double[] serialized = new double[9];
        int nextIndex = 0;
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                serialized[nextIndex++] = cameraMatrix.get(row, col);
            }
        }
        return serialized;
    }

    private double[] serializeDistortionCoefficients(Matrix<N8, N1> distCoeffs) {
        double[] serialized = new double[8];
        for (int row = 0; row < 8; row++) {
            serialized[row] = distCoeffs.get(row, 0);
        }
        return serialized;
    }

    private int serializeFieldLayout(FlatBufferBuilder builder, AprilTagFieldLayout fieldLayout) {
        List<edu.wpi.first.apriltag.AprilTag> tags = new ArrayList<>(fieldLayout.getTags());
        tags.sort(Comparator.comparingInt(tag -> tag.ID));

        frc.robot.generated.flatbuffers.AprilTagFieldLayout.startTagsVector(builder, tags.size());
        for (int i = tags.size() - 1; i >= 0; i--) {
            edu.wpi.first.apriltag.AprilTag tag = tags.get(i);
            frc.robot.generated.flatbuffers.AprilTag.createAprilTag(
                    builder,
                    tag.ID,
                    tag.pose.getX(),
                    tag.pose.getY(),
                    tag.pose.getZ(),
                    tag.pose.getRotation().getQuaternion().getW(),
                    tag.pose.getRotation().getQuaternion().getX(),
                    tag.pose.getRotation().getQuaternion().getY(),
                    tag.pose.getRotation().getQuaternion().getZ());
        }
        int tagsOffset = builder.endVector();

        frc.robot.generated.flatbuffers.AprilTagFieldLayout.startAprilTagFieldLayout(builder);
        frc.robot.generated.flatbuffers.AprilTagFieldLayout.addTags(builder, tagsOffset);
        int fieldDimensions =
                frc.robot.generated.flatbuffers.FieldDimensions.createFieldDimensions(
                        builder, fieldLayout.getFieldLength(), fieldLayout.getFieldWidth());
        frc.robot.generated.flatbuffers.AprilTagFieldLayout.addField(builder, fieldDimensions);
        return frc.robot.generated.flatbuffers.AprilTagFieldLayout.endAprilTagFieldLayout(builder);
    }
}
