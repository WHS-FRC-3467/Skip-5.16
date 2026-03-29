# CamCam FlatBuffers

These schemas define the NetworkTables payloads exchanged with each `CamCam` coprocessor.

## Configuration

- `CamCamConfiguration` always includes the AprilTag camera list and AprilTag field layout.
- `object_detection_camera` is optional.
- If `object_detection_camera` is omitted, the coprocessor should not emit object-detection observations.

## Runtime Packets

- `CamCamData` packets carry either:
  - `ConfiguredData` when the coprocessor accepted configuration
  - `UnconfiguredError` when the coprocessor has not accepted configuration
- `ConfiguredData.pose_observations` and `ConfiguredData.object_detection_observations` are both optional and independent.
- A configured packet may contain only pose data, only object-detection data, both, or neither if it is only reporting runtime problems.

## Problems

- `CamCamProblemCameraDisconnect` reports a disconnected AprilTag camera.
- `CamCamProblemObjectDetectionCameraUnconfigured` is a runtime warning, not a fatal coprocessor-unconfigured state. This should only trip if the object detection camera *is* connected, but unconfigured
