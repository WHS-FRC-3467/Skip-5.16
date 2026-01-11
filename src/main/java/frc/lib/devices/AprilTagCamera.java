/*
 * Copyright (C) 2025 Windham Windup
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

import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.io.vision.VisionIO;
import frc.lib.io.vision.VisionIO.VisionIOInputs;
import lombok.Getter;

/**
 * Represents a single AprilTag camera on the robot.
 * 
 * <p>
 * Handles interfacing with the {@link VisionIO} hardware layer, providing camera intrinsics,
 * mounting transforms, and reading vision results.
 */
public class AprilTagCamera {

    /**
     * Immutable set of properties describing the camera.
     *
     * @param name Unique name for the camera
     * @param robotToCamera Transform from the robot frame to the camera frame
     * @param cameraMatrix Intrinsic camera matrix
     * @param distCoeffs Distortion coefficients for the camera
     * @param resolutionWidth Camera resolution width in pixels
     * @param resolutionHeight Camera resolution height in pixels
     * @param stdDevFactor Standard deviation factor used in vision pose estimation
     */
    public record CameraProperties(
        String name,
        Transform3d robotToCamera,
        Matrix<N3, N3> cameraMatrix,
        Matrix<N8, N1> distCoeffs,
        int resolutionWidth,
        int resolutionHeight,
        double stdDevFactor) {
    }

    private final VisionIO io;
    private final VisionIOInputs inputs;

    private final Alert mismatchedIntrinsicsAlert;

    /** The camera's properties, including intrinsics and transform relative to the robot. */
    @Getter
    private final CameraProperties properties;

    /**
     * Constructs a new {@code AprilTagCamera}.
     * <p>
     * Initializes the camera properties, sets up logging of inputs, and validates that supplied
     * intrinsics match those from the {@link VisionIO} inputs.
     * </p>
     *
     * @param properties the camera properties
     * @param io the VisionIO interface for this camera
     */
    public AprilTagCamera(CameraProperties properties, VisionIO io)
    {
        mismatchedIntrinsicsAlert = new Alert(
            "Camera "
                + properties.name()
                + "'s supplied intrinsics in code do not match intrinsics from replayed inputs! Defaulting to inputs!",
            AlertType.kWarning);

        this.io = io;
        inputs = new VisionIOInputs(properties.cameraMatrix(), properties.distCoeffs());

        // Get camera intrinsics from inputs to potentially pull from log if replaying
        Logger.processInputs(properties.name, inputs);

        Matrix<N3, N3> cameraMatrix = MatBuilder.fill(Nat.N3(), Nat.N3(), inputs.cameraMatrix);
        Matrix<N8, N1> distCoeffs = MatBuilder.fill(Nat.N8(), Nat.N1(), inputs.distCoeffs);

        if (!cameraMatrix.equals(properties.cameraMatrix)
            || !distCoeffs.equals(properties.distCoeffs)) {
            mismatchedIntrinsicsAlert.set(true);
        }

        this.properties =
            new CameraProperties(
                properties.name,
                properties.robotToCamera,
                cameraMatrix,
                distCoeffs,
                properties.resolutionWidth,
                properties.resolutionHeight,
                properties.stdDevFactor);
    }

    /**
     * Retrieves unread vision results from the camera.
     * <p>
     * Updates inputs from the {@link VisionIO}, processes them through the logger, and returns any
     * results if the camera is connected. Returns an empty {@link Optional} if the camera is not
     * connected.
     * </p>
     *
     * @return an {@link Optional} containing an array of {@link PhotonPipelineResult} if available,
     *         or {@link Optional#empty()} if the camera is disconnected
     */
    public Optional<PhotonPipelineResult[]> getUnreadResults()
    {
        io.updateInputs(inputs);
        Logger.processInputs(properties.name(), inputs);

        if (!inputs.connected)
            return Optional.empty();

        return Optional.of(inputs.results);
    }
}
