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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
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

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Real hardware implementation of {@link VisionIO} using c2.
 *
 * <p>Publishes the capture configuration expected by the c2 coprocessor and reads its per-camera
 * flatbuffer output as raw bytes for downstream decoding.
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
    private static final ConcurrentHashMap<String, C2DeviceContext> DEVICE_CONTEXTS =
            new ConcurrentHashMap<>();

    /**
     * Shared capture configuration for c2.
     *
     * @param deviceId NetworkTables device ID root, e.g. {@code dsv0}
     * @param cameraIndex Index of this camera's output topic
     * @param cameraId Capture device ID consumed by c2
     * @param exposure Exposure value sent to c2's config topic
     * @param gain Gain value sent to c2's config topic
     * @param fiducialSizeMeters AprilTag edge length used by c2 solvePnP
     * @param tagLayout Field layout used both for config publishing and result reconstruction
     * @param tagLayoutJson Serialized field layout in c2's expected JSON format
     * @param pollStorageDepth NT queue depth for unread raw frames
     */
    public static record C2Config(
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

    /** Per-device shared c2 config publishers. Camera outputs remain per VisionIO instance. */
    private static final class C2DeviceContext {
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

        private C2DeviceContext(
                NetworkTableInstance ntInstance,
                CameraProperties cameraProperties,
                C2Config config) {
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
                CameraProperties cameraProperties, C2Config config) {
            String existingCameraName =
                    cameraNamesByIndex.putIfAbsent(config.cameraIndex(), cameraProperties.name());
            if (existingCameraName != null && !existingCameraName.equals(cameraProperties.name())) {
                throw new IllegalArgumentException(
                        "Duplicate c2 cameraIndex "
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
                    "Conflicting shared c2 config for deviceId \""
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
    private final C2Config config;
    private final C2DeviceContext deviceContext;
    private final RawSubscriber observationSubscriber;

    /**
     * Constructs a c2 camera interface with explicit c2 configuration.
     *
     * @param cameraProperties Camera configuration including name and calibration
     * @param config c2 capture and topic configuration
     */
    public VisionIOC2(CameraProperties cameraProperties, C2Config config) {
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
            inputs.rawResults = new byte[0][];
            inputs.captureTimestampsUs = new long[0];
            inputs.publishTimestampsUs = new long[0];
            return;
        }

        ArrayList<byte[]> results = new ArrayList<>(unreadFrames.length);
        ArrayList<Long> captureTimestampsUs = new ArrayList<>(unreadFrames.length);
        ArrayList<Long> publishTimestampsUs = new ArrayList<>(unreadFrames.length);
        for (TimestampedRaw unreadFrame : unreadFrames) {
            if (unreadFrame != null && unreadFrame.value != null && unreadFrame.value.length > 0) {
                results.add(unreadFrame.value);
                captureTimestampsUs.add(unreadFrame.timestamp);
                publishTimestampsUs.add(
                        unreadFrame.serverTime != 0
                                ? unreadFrame.serverTime
                                : unreadFrame.timestamp);
            }
        }

        inputs.rawResults = results.toArray(byte[][]::new);
        inputs.captureTimestampsUs =
                captureTimestampsUs.stream().mapToLong(Long::longValue).toArray();
        inputs.publishTimestampsUs =
                publishTimestampsUs.stream().mapToLong(Long::longValue).toArray();
    }

    private void publishConfigIfNeeded() {
        deviceContext.publishConfigIfNeeded();
    }

    public static C2Config defaultsFor(int cameraIndex) {
        return new C2Config(
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

    private static C2Config validateConfig(C2Config config) {
        if (config == null) {
            throw new IllegalArgumentException("c2Config cannot be null");
        }
        if (config.deviceId() == null || config.deviceId().isBlank()) {
            throw new IllegalArgumentException("c2Config.deviceId cannot be blank");
        }
        if (config.cameraIndex() < 0) {
            throw new IllegalArgumentException("c2Config.cameraIndex must be non-negative");
        }
        if (config.cameraId() == null) {
            throw new IllegalArgumentException("c2Config.cameraId cannot be null");
        }
        if (config.tagLayout() == null) {
            throw new IllegalArgumentException("c2Config.tagLayout cannot be null");
        }
        if (config.tagLayoutJson() == null || config.tagLayoutJson().isBlank()) {
            throw new IllegalArgumentException("c2Config.tagLayoutJson cannot be blank");
        }
        if (config.pollStorageDepth() <= 0) {
            throw new IllegalArgumentException("c2Config.pollStorageDepth must be positive");
        }
        return config;
    }

    private static C2DeviceContext getOrCreateDeviceContext(
            CameraProperties cameraProperties, C2Config config) {
        return DEVICE_CONTEXTS.compute(
                config.deviceId(),
                (deviceId, existingContext) -> {
                    if (existingContext == null) {
                        return new C2DeviceContext(
                                NetworkTableInstance.getDefault(), cameraProperties, config);
                    }
                    existingContext.registerCamera(cameraProperties, config);
                    return existingContext;
                });
    }

    private static SharedDeviceConfig sharedConfigFor(
            CameraProperties cameraProperties, C2Config config) {
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
