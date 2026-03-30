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

import dsv0.CameraObservation;
import dsv0.CameraOutput;
import dsv0.Frame;
import dsv0.PoseSolution;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import frc.lib.io.vision.VisionIO;
import frc.lib.io.vision.VisionIOInputsAutoLogged;

import lombok.Getter;

import org.littletonrobotics.junction.Logger;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PnpResult;
import org.photonvision.targeting.TargetCorner;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

/**
 * Represents a single AprilTag camera on the robot.
 *
 * <p>Handles interfacing with the {@link VisionIO} hardware layer, providing camera intrinsics,
 * mounting transforms, and reading vision results.
 */
public class AprilTagCamera {
    /** Magic prefix for identifying photon-encoded results */
    private static final byte[] PHOTON_RESULT_MAGIC = new byte[] {'P', 'H', 'O', 'T', 'O', 'N', 1};

    private static final List<TargetCorner> ZERO_MIN_AREA_RECT_CORNERS =
            List.of(
                    new TargetCorner(0.0, 0.0),
                    new TargetCorner(0.0, 0.0),
                    new TargetCorner(0.0, 0.0),
                    new TargetCorner(0.0, 0.0));
    private static final List<TargetCorner> EMPTY_DETECTED_CORNERS = List.of();

    /**
     * Intrinsic &amp; observed properties describing the camera.
     *
     * @param name Unique name for the camera
     * @param robotToCamera Transform from the robot frame to the camera frame
     * @param cameraMatrix Intrinsic camera matrix
     * @param distCoeffs Distortion coefficients for the camera
     * @param resolutionWidth Camera resolution width in pixels
     * @param resolutionHeight Camera resolution height in pixels
     * @param stdDevFactor Standard deviation factor used in vision pose estimation
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
            double stdDevFactor,
            Angle fov,
            double fps,
            Time latency,
            Time latencyStdDev) {}

    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs;
    private final AprilTagFieldLayout tagLayout;
    private final int cameraIndex;

    private final Alert disconnectAlert;
    private final Debouncer disconnectDebouncer = new Debouncer(0.25);
    private long c2SequenceId = 0;

    /** The camera's properties, including intrinsics and transform relative to the robot. */
    @Getter private final CameraProperties properties;

    /**
     * Constructs a new {@code AprilTagCamera}.
     *
     * <p>Initializes the camera properties and sets up logging of inputs.
     *
     * @param properties the camera properties
     * @param io the VisionIO interface for this camera
     */
    public AprilTagCamera(
            CameraProperties properties,
            VisionIO io,
            int cameraIndex,
            AprilTagFieldLayout tagLayout) {
        disconnectAlert =
                new Alert("Camera " + properties.name() + " is Disconnected!", AlertType.kError);

        this.io = io;
        this.cameraIndex = cameraIndex;
        this.tagLayout = tagLayout;
        this.inputs = new VisionIOInputsAutoLogged();

        this.properties =
                new CameraProperties(
                        properties.name(),
                        properties.robotToCamera(),
                        properties.cameraMatrix(),
                        properties.distCoeffs(),
                        properties.resolutionWidth(),
                        properties.resolutionHeight(),
                        properties.stdDevFactor(),
                        properties.fov(),
                        properties.fps(),
                        properties.latency(),
                        properties.latencyStdDev());
    }

    /**
     * Retrieves unread vision results from the camera.
     *
     * <p>Updates inputs from the {@link VisionIO}, processes them through the logger, and returns
     * any results if the camera is connected. Returns an empty {@link Optional} if the camera is
     * not connected.
     *
     * @return an {@link Optional} containing an array of {@link PhotonPipelineResult} if available,
     *     or {@link Optional#empty()} if the camera is disconnected
     */
    public Optional<PhotonPipelineResult[]> getUnreadResults() {
        io.updateInputs(inputs);
        Logger.processInputs(properties.name(), inputs);

        boolean disconnected = !inputs.connected;
        disconnectAlert.set(disconnectDebouncer.calculate(disconnected));
        if (disconnected) return Optional.empty();

        return Optional.of(decodeUnreadResults());
    }

    private PhotonPipelineResult[] decodeUnreadResults() {
        ArrayList<PhotonPipelineResult> decodedResults = new ArrayList<>(inputs.rawResults.length);
        for (int i = 0; i < inputs.rawResults.length; i++) {
            byte[] rawResult = inputs.rawResults[i];
            long captureTimestampUs =
                    i < inputs.captureTimestampsUs.length ? inputs.captureTimestampsUs[i] : 0;
            long publishTimestampUs =
                    i < inputs.publishTimestampsUs.length ? inputs.publishTimestampsUs[i] : 0;
            PhotonPipelineResult decodedResult =
                    decodeResult(rawResult, captureTimestampUs, publishTimestampUs);
            if (decodedResult != null) {
                decodedResults.add(decodedResult);
            }
        }
        return decodedResults.toArray(PhotonPipelineResult[]::new);
    }

    private PhotonPipelineResult decodeResult(
            byte[] rawResult, long captureTimestampUs, long publishTimestampUs) {
        if (rawResult == null || rawResult.length == 0) {
            return null;
        }
        if (isPhotonResult(rawResult)) {
            return decodePhotonResult(rawResult);
        }
        return decodeC2Result(rawResult, captureTimestampUs, publishTimestampUs);
    }

    private static boolean isPhotonResult(byte[] rawResult) {
        return rawResult.length > PHOTON_RESULT_MAGIC.length
                && Arrays.equals(
                        PHOTON_RESULT_MAGIC, Arrays.copyOf(rawResult, PHOTON_RESULT_MAGIC.length));
    }

    private static PhotonPipelineResult decodePhotonResult(byte[] rawResult) {
        try {
            byte[] serializedResult =
                    Arrays.copyOfRange(rawResult, PHOTON_RESULT_MAGIC.length, rawResult.length);
            return PhotonPipelineResult.photonStruct.unpack(new Packet(serializedResult));
        } catch (RuntimeException e) {
            return null;
        }
    }

    private PhotonPipelineResult decodeC2Result(
            byte[] rawResult, long captureTimestampUs, long publishTimestampUs) {
        if (tagLayout == null || cameraIndex < 0) {
            return null;
        }

        Frame frame;
        try {
            frame = Frame.getRootAsFrame(ByteBuffer.wrap(rawResult));
        } catch (RuntimeException e) {
            return null;
        }

        CameraOutput cameraOutput = findCameraOutput(frame, cameraIndex);
        if (cameraOutput == null) {
            long resolvedCaptureTimestampUs =
                    captureTimestampUs != 0 ? captureTimestampUs : frame.timestampUs();
            long resolvedPublishTimestampUs =
                    publishTimestampUs != 0 ? publishTimestampUs : resolvedCaptureTimestampUs;
            return createEmptyC2Result(resolvedCaptureTimestampUs, resolvedPublishTimestampUs);
        }

        long resolvedCaptureTimestampUs =
                captureTimestampUs != 0
                        ? captureTimestampUs
                        : (cameraOutput.timestampUs() != 0
                                ? cameraOutput.timestampUs()
                                : frame.timestampUs());
        long resolvedPublishTimestampUs =
                publishTimestampUs != 0 ? publishTimestampUs : resolvedCaptureTimestampUs;

        CameraObservation observation = cameraOutput.cameraObservation();
        if (observation == null || observation.solution0() == null) {
            return createEmptyC2Result(resolvedCaptureTimestampUs, resolvedPublishTimestampUs);
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
            Optional<Pose3d> tagPose = tagLayout.getTagPose(tagId);
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
                c2SequenceId++,
                resolvedCaptureTimestampUs,
                resolvedPublishTimestampUs,
                0,
                targets,
                multitagResult);
    }

    private CameraOutput findCameraOutput(Frame frame, int cameraIndex) {
        if (frame == null || frame.camerasLength() == 0) {
            return null;
        }

        for (int i = 0; i < frame.camerasLength(); i++) {
            CameraOutput candidate = frame.cameras(i);
            if (candidate != null && candidate.cameraIndex() == cameraIndex) {
                return candidate;
            }
        }

        return null;
    }

    private PhotonPipelineResult createEmptyC2Result(
            long captureTimestampUs, long publishTimestampUs) {
        return new PhotonPipelineResult(
                c2SequenceId++,
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
}
