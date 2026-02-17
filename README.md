# Skip-5.16
[![CI](https://github.com/WHS-FRC-3467/Skip-5.16/actions/workflows/build.yml/badge.svg)](https://github.com/WHS-FRC-3467/Skip-5.16/actions/workflows/build.yml)
[![Javadoc](https://github.com/WHS-FRC-3467/Skip-5.16/actions/workflows/javadoc.yml/badge.svg)](https://github.com/WHS-FRC-3467/Skip-5.16/actions/workflows/javadoc.yml)

Codebase for the 2026 season REBUILT

## Unique Features

Our robot code leverages several advanced software engineering practices and custom abstractions that distinguish it from typical FRC teams:

- **Hardware Abstraction Architecture**: A three-layer architecture (IO interfaces, Mechanism abstractions, and Subsystems) that completely isolates vendor-specific code from robot logic, enabling seamless hardware swapping and comprehensive simulation testing.

- **AdvantageKit Integration**: Full sensor data logging with `@AutoLog` annotations on all IO interfaces, allowing complete match replay and debugging capabilities through time-travel debugging in AdvantageScope.

- **Type-Safe Units System**: Consistent use of WPILib's units library throughout the codebase with typed measures (`AngularVelocity`, `Distance`, `Voltage`) instead of raw doubles, eliminating unit conversion errors and improving code clarity.

- **Physics-Based Simulation**: Separate Real and Sim implementations for all mechanisms with WPILib physics models (`FlywheelSim`, `ElevatorSim`, `SingleJointedArmSim`) that accurately model robot behavior for testing without hardware.

- **Runtime Hardware Selection**: Mode-based dependency injection through constants classes with `getMechanism()` factories that select appropriate implementations (Real, Sim, or Replay) at runtime based on `Constants.currentMode`.

- **Unified Mechanism Base Class**: Abstract `Mechanism<T extends MotorIO>` base class providing common functionality for motor control, PID tuning, logging, and visualization across all robot mechanisms (flywheels, linear actuators, rotary arms).

- **Device Wrapper Classes**: High-level device abstractions for sensors and actuators (BeamBreak, DistanceSensor, AprilTagCamera, Lights, Servo, AbsoluteEncoder) that provide consistent interfaces and automatic AdvantageKit logging.

- **Mechanism-Specific Visualizers**: Custom visualizers for each mechanism type (RotaryVisualizer, LinearMechanismVisualizer, FlywheelVisualizer, GamePieceVisualizer) that render mechanism state in AdvantageScope for real-time debugging.

- **Tunable PID Configuration**: `LoggedTunableNumber` integration throughout the codebase enabling live PID tuning through NetworkTables without code redeployment, with all changes automatically logged to match data.

- **Centralized Pose Estimation**: Singleton `RobotState` class with custom `PoseEstimator` that fuses odometry and vision measurements with configurable standard deviations and time windowing for optimal localization accuracy.

- **SteppableCommandGroup**: Custom command group implementation supporting manual step-through of autonomous routines using trigger inputs, enabling precise autonomous debugging and development.

- **Port Management System**: Strongly-typed `Device.CAN` class for all hardware ports defined in a central `Ports` class, preventing CAN ID conflicts and improving code organization with CANBus separation.

- **Custom Command Extensions**: Reusable command base classes (`AlignToPoseBase`, `DriveToPoseBase`) that provide common autonomous functionality while allowing subsystem-specific implementations.

- **Code Quality Tooling**: Automated enforcement of code standards using Spotless formatter and Google Error Prone static analysis integrated into the build process, ensuring consistent code quality across the team.

- **Advanced Vision Processing**: PhotonVision integration with multi-camera support, pose estimation, and custom filtering for AprilTags and object detection with automatic fallback strategies.

## Documentation

API documentation is automatically generated and published to GitHub Pages:
- **Javadoc**: [https://whs-frc-3467.github.io/Skip-5.16/](https://whs-frc-3467.github.io/Skip-5.16/)

The documentation is automatically updated on every push to the `dev` branch.
