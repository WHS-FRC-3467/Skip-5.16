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

import dsv0.CameraObservation;
import dsv0.CameraOutput;
import dsv0.Frame;
import dsv0.PoseSolution;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.TimestampedRaw;
import edu.wpi.first.util.WPIUtilJNI;

import frc.lib.devices.AprilTagCamera.CameraProperties;
import frc.robot.FieldConstants.AprilTagLayoutType;

import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PnpResult;
import org.photonvision.targeting.TargetCorner;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Real hardware implementation of {@link VisionIO} using slopstar.
 *
 * <p>Publishes the capture configuration expected by the slopstar coprocessor and converts its
 * per-camera flatbuffer output into {@link PhotonPipelineResult} instances so the rest of the
 * vision stack can stay unchanged.
 */
public class VisionIOC2 implements VisionIO {
    private static final String DEFAULT_DEVICE_ID = "dsv0";
    private static final String DEFAULT_CAMERA_ID = "0";
    private static final String FLATBUFFER_TYPE = "dsv0_fb";
    private static final int DEFAULT_CAMERA_EXPOSURE = 20;
    private static final int DEFAULT_CAMERA_GAIN = 0;
    private static final int DEFAULT_POLL_STORAGE_DEPTH = 32;
    private static final long DISCONNECT_TIMEOUT_US = 500_000L;
    private static final double DEFAULT_FIDUCIAL_SIZE_METERS = Units.inchesToMeters(6.5);
    private static final List<TargetCorner> ZERO_MIN_AREA_RECT_CORNERS =
            List.of(
                    new TargetCorner(0.0, 0.0),
                    new TargetCorner(0.0, 0.0),
                    new TargetCorner(0.0, 0.0),
                    new TargetCorner(0.0, 0.0));
    private static final List<TargetCorner> EMPTY_DETECTED_CORNERS = List.of();
    private static final ConcurrentHashMap<String, SlopstarDeviceContext> DEVICE_CONTEXTS =
            new ConcurrentHashMap<>();

    /**
     * Shared capture configuration for slopstar.
     *
     * @param deviceId NetworkTables device ID root, e.g. {@code dsv0}
     * @param cameraIndex Index of this camera's output topic
     * @param cameraId Capture device ID consumed by slopstar
     * @param exposure Exposure value sent to slopstar's config topic
     * @param gain Gain value sent to slopstar's config topic
     * @param fiducialSizeMeters AprilTag edge length used by slopstar solvePnP
     * @param tagLayout Field layout used both for config publishing and result reconstruction
     * @param tagLayoutJson Serialized field layout in slopstar's expected JSON format
     * @param pollStorageDepth NT queue depth for unread raw frames
     */
    public static record SlopstarConfig(
            String deviceId,
            int cameraIndex,
            String cameraId,
            int exposure,
            int gain,
            double fiducialSizeMeters,
            AprilTagFieldLayout tagLayout,
            String tagLayoutJson,
            int pollStorageDepth) {}

    private static record SharedDeviceConfig(
            String cameraId,
            int resolutionWidth,
            int resolutionHeight,
            int exposure,
            int gain,
            double fiducialSizeMeters,
            String tagLayoutJson) {}

    /**
     * Per-device shared slopstar config publishers. Camera outputs remain per VisionIO instance.
     */
    private static final class SlopstarDeviceContext {
        private final NetworkTableInstance ntInstance;
        private final String deviceId;
        private final StringPublisher cameraIdPublisher;
        private final IntegerPublisher resolutionWidthPublisher;
        private final IntegerPublisher resolutionHeightPublisher;
        private final IntegerPublisher exposurePublisher;
        private final IntegerPublisher gainPublisher;
        private final DoublePublisher fiducialSizePublisher;
        private final StringPublisher tagLayoutPublisher;
        private final Map<Integer, String> cameraNamesByIndex = new HashMap<>();

        private SharedDeviceConfig sharedConfig = null;
        private boolean hasPublishedConfig = false;
        private boolean lastNtConnected = false;

        private SlopstarDeviceContext(
                NetworkTableInstance ntInstance,
                CameraProperties cameraProperties,
                SlopstarConfig config) {
            this.ntInstance = ntInstance;
            this.deviceId = config.deviceId();

            NetworkTable configTable = ntInstance.getTable("/" + deviceId + "/config");
            cameraIdPublisher = configTable.getStringTopic("camera_id").publish();
            resolutionWidthPublisher =
                    configTable.getIntegerTopic("camera_resolution_width").publish();
            resolutionHeightPublisher =
                    configTable.getIntegerTopic("camera_resolution_height").publish();
            exposurePublisher = configTable.getIntegerTopic("camera_exposure").publish();
            gainPublisher = configTable.getIntegerTopic("camera_gain").publish();
            fiducialSizePublisher = configTable.getDoubleTopic("fiducial_size_m").publish();
            tagLayoutPublisher = configTable.getStringTopic("tag_layout").publish();

            registerCamera(cameraProperties, config);
        }

        public synchronized void registerCamera(
                CameraProperties cameraProperties, SlopstarConfig config) {
            String existingCameraName =
                    cameraNamesByIndex.putIfAbsent(config.cameraIndex(), cameraProperties.name());
            if (existingCameraName != null && !existingCameraName.equals(cameraProperties.name())) {
                throw new IllegalArgumentException(
                        "Duplicate slopstar cameraIndex "
                                + config.cameraIndex()
                                + " for deviceId \""
                                + deviceId
                                + "\". Cameras \""
                                + existingCameraName
                                + "\" and \""
                                + cameraProperties.name()
                                + "\" cannot share the same output topic.");
            }

            SharedDeviceConfig candidateSharedConfig = sharedConfigFor(cameraProperties, config);
            if (sharedConfig == null) {
                sharedConfig = candidateSharedConfig;
            } else if (!sharedConfig.equals(candidateSharedConfig)) {
                throw conflictingSharedConfig(cameraProperties, candidateSharedConfig);
            }

            publishConfigIfNeeded();
        }

        public synchronized void publishConfigIfNeeded() {
            if (sharedConfig == null) {
                return;
            }

            boolean connected = ntInstance.isConnected();
            boolean shouldPublish = !hasPublishedConfig || (connected && !lastNtConnected);
            lastNtConnected = connected;

            if (!shouldPublish) {
                return;
            }

            cameraIdPublisher.set(sharedConfig.cameraId());
            resolutionWidthPublisher.set(sharedConfig.resolutionWidth());
            resolutionHeightPublisher.set(sharedConfig.resolutionHeight());
            exposurePublisher.set(sharedConfig.exposure());
            gainPublisher.set(sharedConfig.gain());
            fiducialSizePublisher.set(sharedConfig.fiducialSizeMeters());
            tagLayoutPublisher.set(sharedConfig.tagLayoutJson());
            hasPublishedConfig = true;
        }

        private IllegalArgumentException conflictingSharedConfig(
                CameraProperties cameraProperties, SharedDeviceConfig candidateSharedConfig) {
            StringBuilder mismatches = new StringBuilder();
            appendMismatch(
                    mismatches,
                    "cameraId",
                    sharedConfig.cameraId(),
                    candidateSharedConfig.cameraId());
            appendMismatch(
                    mismatches,
                    "resolutionWidth",
                    sharedConfig.resolutionWidth(),
                    candidateSharedConfig.resolutionWidth());
            appendMismatch(
                    mismatches,
                    "resolutionHeight",
                    sharedConfig.resolutionHeight(),
                    candidateSharedConfig.resolutionHeight());
            appendMismatch(
                    mismatches,
                    "exposure",
                    sharedConfig.exposure(),
                    candidateSharedConfig.exposure());
            appendMismatch(mismatches, "gain", sharedConfig.gain(), candidateSharedConfig.gain());
            appendMismatch(
                    mismatches,
                    "fiducialSizeMeters",
                    sharedConfig.fiducialSizeMeters(),
                    candidateSharedConfig.fiducialSizeMeters());
            appendMismatch(
                    mismatches,
                    "tagLayoutJson",
                    sharedConfig.tagLayoutJson(),
                    candidateSharedConfig.tagLayoutJson());

            return new IllegalArgumentException(
                    "Conflicting shared slopstar config for deviceId \""
                            + deviceId
                            + "\" from camera \""
                            + cameraProperties.name()
                            + "\". All cameras publishing to /"
                            + deviceId
                            + "/config must agree on: "
                            + mismatches
                            + ".");
        }

        private static void appendMismatch(
                StringBuilder mismatches, String fieldName, Object expected, Object actual) {
            if (expected == null ? actual == null : expected.equals(actual)) {
                return;
            }

            if (!mismatches.isEmpty()) {
                mismatches.append(", ");
            }
            mismatches
                    .append(fieldName)
                    .append(" (expected ")
                    .append(summarizeValue(expected))
                    .append(", got ")
                    .append(summarizeValue(actual))
                    .append(")");
        }

        private static String summarizeValue(Object value) {
            String text = String.valueOf(value);
            return text.length() <= 80 ? text : text.substring(0, 77) + "...";
        }
    }

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final SlopstarConfig config;
    private final SlopstarDeviceContext deviceContext;
    private final RawSubscriber observationSubscriber;

    private long sequenceId = 0;

    /**
     * Constructs a slopstar camera interface with explicit slopstar configuration.
     *
     * @param cameraProperties Camera configuration including name and calibration
     * @param config slopstar capture and topic configuration
     */
    public VisionIOC2(CameraProperties cameraProperties, SlopstarConfig config) {
        this.config = validateConfig(config);
        this.deviceContext = getOrCreateDeviceContext(cameraProperties, this.config);

        NetworkTable outputTable =
                ntInstance.getTable(
                        "/"
                                + this.config.deviceId()
                                + "/output/camera_"
                                + this.config.cameraIndex());

        observationSubscriber =
                outputTable
                        .getRawTopic("observation")
                        .subscribe(
                                FLATBUFFER_TYPE,
                                new byte[0],
                                PubSubOption.sendAll(true),
                                PubSubOption.keepDuplicates(true),
                                PubSubOption.pollStorage(this.config.pollStorageDepth()));
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        publishConfigIfNeeded();

        long nowUs = WPIUtilJNI.now();
        long lastChangeUs = observationSubscriber.getLastChange();

        inputs.connected =
                ntInstance.isConnected()
                        && observationSubscriber.exists()
                        && lastChangeUs > 0
                        && nowUs - lastChangeUs <= DISCONNECT_TIMEOUT_US;

        TimestampedRaw[] unreadFrames = observationSubscriber.readQueue();
        if (unreadFrames.length == 0) {
            inputs.results = new PhotonPipelineResult[0];
            return;
        }

        ArrayList<PhotonPipelineResult> results = new ArrayList<>(unreadFrames.length);
        for (TimestampedRaw unreadFrame : unreadFrames) {
            PhotonPipelineResult result = decodeResult(unreadFrame);
            if (result != null) {
                results.add(result);
            }
        }

        inputs.results = results.toArray(PhotonPipelineResult[]::new);
    }

    private void publishConfigIfNeeded() {
        deviceContext.publishConfigIfNeeded();
    }

    private PhotonPipelineResult decodeResult(TimestampedRaw unreadFrame) {
        if (unreadFrame == null || unreadFrame.value == null || unreadFrame.value.length == 0) {
            return null;
        }

        Frame frame;
        try {
            frame = Frame.getRootAsFrame(ByteBuffer.wrap(unreadFrame.value));
        } catch (RuntimeException e) {
            return null;
        }

        CameraOutput cameraOutput = findCameraOutput(frame);
        long captureTimestampUs = unreadFrame.timestamp;
        long publishTimestampUs =
                unreadFrame.serverTime != 0 ? unreadFrame.serverTime : captureTimestampUs;

        if (cameraOutput == null) {
            return createEmptyResult(captureTimestampUs, publishTimestampUs);
        }

        CameraObservation observation = cameraOutput.cameraObservation();
        if (observation == null || observation.solution0() == null) {
            return createEmptyResult(captureTimestampUs, publishTimestampUs);
        }

        PoseSolution primarySolution = observation.solution0();
        Pose3d fieldToCamera = toWpilibPose(primarySolution);
        PoseSolution alternateSolution = observation.solution1();
        Pose3d fieldToCameraAlt =
                alternateSolution != null ? toWpilibPose(alternateSolution) : null;

        double ambiguity =
                alternateSolution != null
                        ? computeAmbiguity(primarySolution.error(), alternateSolution.error())
                        : 0.0;

        ArrayList<PhotonTrackedTarget> targets = new ArrayList<>(observation.tagIdsLength());
        for (int i = 0; i < observation.tagIdsLength(); i++) {
            int tagId = observation.tagIds(i);
            Optional<Pose3d> tagPose = config.tagLayout().getTagPose(tagId);
            if (tagPose.isEmpty()) {
                continue;
            }

            Transform3d bestCameraToTarget = new Transform3d(fieldToCamera, tagPose.get());
            Transform3d altCameraToTarget =
                    fieldToCameraAlt != null
                            ? new Transform3d(fieldToCameraAlt, tagPose.get())
                            : bestCameraToTarget;

            Translation3d translation = bestCameraToTarget.getTranslation();
            double yawDegrees = Math.toDegrees(Math.atan2(translation.getY(), translation.getX()));
            double pitchDegrees =
                    Math.toDegrees(
                            Math.atan2(
                                    translation.getZ(),
                                    Math.hypot(translation.getX(), translation.getY())));

            targets.add(
                    new PhotonTrackedTarget(
                            yawDegrees,
                            pitchDegrees,
                            0.0,
                            0.0,
                            tagId,
                            -1,
                            -1.0f,
                            bestCameraToTarget,
                            altCameraToTarget,
                            ambiguity,
                            ZERO_MIN_AREA_RECT_CORNERS,
                            EMPTY_DETECTED_CORNERS));
        }

        Optional<MultiTargetPNPResult> multitagResult = Optional.empty();
        if (targets.size() >= 2) {
            Transform3d fieldToCameraTransform = new Transform3d(Pose3d.kZero, fieldToCamera);
            List<Short> idsUsed =
                    targets.stream().map(target -> (short) target.getFiducialId()).toList();

            multitagResult =
                    Optional.of(
                            new MultiTargetPNPResult(
                                    new PnpResult(fieldToCameraTransform, primarySolution.error()),
                                    idsUsed));
        }

        return new PhotonPipelineResult(
                sequenceId++, captureTimestampUs, publishTimestampUs, 0, targets, multitagResult);
    }

    private CameraOutput findCameraOutput(Frame frame) {
        if (frame == null || frame.camerasLength() == 0) {
            return null;
        }

        for (int i = 0; i < frame.camerasLength(); i++) {
            CameraOutput candidate = frame.cameras(i);
            if (candidate != null && candidate.cameraIndex() == config.cameraIndex()) {
                return candidate;
            }
        }

        return null;
    }

    private PhotonPipelineResult createEmptyResult(
            long captureTimestampUs, long publishTimestampUs) {
        return new PhotonPipelineResult(
                sequenceId++,
                captureTimestampUs,
                publishTimestampUs,
                0,
                List.of(),
                Optional.empty());
    }

    private static Pose3d toWpilibPose(PoseSolution solution) {
        dsv0.Pose3d pose = solution.pose();
        dsv0.Vec3 translation = pose.translation();
        dsv0.Quaternion rotation = pose.rotation();
        return new Pose3d(
                new Translation3d(translation.x(), translation.y(), translation.z()),
                new Rotation3d(
                        new edu.wpi.first.math.geometry.Quaternion(
                                rotation.w(), rotation.x(), rotation.y(), rotation.z())));
    }

    private static double computeAmbiguity(double primaryError, double alternateError) {
        if (alternateError <= 0.0) {
            return 0.0;
        }
        return Math.max(0.0, Math.min(1.0, primaryError / alternateError));
    }

    public static SlopstarConfig defaultsFor(int cameraIndex) {
        return new SlopstarConfig(
                DEFAULT_DEVICE_ID,
                cameraIndex,
                DEFAULT_CAMERA_ID,
                DEFAULT_CAMERA_EXPOSURE,
                DEFAULT_CAMERA_GAIN,
                DEFAULT_FIDUCIAL_SIZE_METERS,
                AprilTagLayoutType.OFFICIAL.getLayout(),
                AprilTagLayoutType.OFFICIAL.getLayoutString(),
                DEFAULT_POLL_STORAGE_DEPTH);
    }

    private static SlopstarConfig validateConfig(SlopstarConfig config) {
        if (config == null) {
            throw new IllegalArgumentException("SlopstarConfig cannot be null");
        }
        if (config.deviceId() == null || config.deviceId().isBlank()) {
            throw new IllegalArgumentException("SlopstarConfig.deviceId cannot be blank");
        }
        if (config.cameraIndex() < 0) {
            throw new IllegalArgumentException("SlopstarConfig.cameraIndex must be non-negative");
        }
        if (config.cameraId() == null) {
            throw new IllegalArgumentException("SlopstarConfig.cameraId cannot be null");
        }
        if (config.tagLayout() == null) {
            throw new IllegalArgumentException("SlopstarConfig.tagLayout cannot be null");
        }
        if (config.tagLayoutJson() == null || config.tagLayoutJson().isBlank()) {
            throw new IllegalArgumentException("SlopstarConfig.tagLayoutJson cannot be blank");
        }
        if (config.pollStorageDepth() <= 0) {
            throw new IllegalArgumentException("SlopstarConfig.pollStorageDepth must be positive");
        }
        return config;
    }

    private static SlopstarDeviceContext getOrCreateDeviceContext(
            CameraProperties cameraProperties, SlopstarConfig config) {
        return DEVICE_CONTEXTS.compute(
                config.deviceId(),
                (deviceId, existingContext) -> {
                    if (existingContext == null) {
                        return new SlopstarDeviceContext(
                                NetworkTableInstance.getDefault(), cameraProperties, config);
                    }
                    existingContext.registerCamera(cameraProperties, config);
                    return existingContext;
                });
    }

    private static SharedDeviceConfig sharedConfigFor(
            CameraProperties cameraProperties, SlopstarConfig config) {
        return new SharedDeviceConfig(
                config.cameraId(),
                cameraProperties.resolutionWidth(),
                cameraProperties.resolutionHeight(),
                config.exposure(),
                config.gain(),
                config.fiducialSizeMeters(),
                config.tagLayoutJson());
    }
}
