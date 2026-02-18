package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.Ports;

/**
 * Configuration constants for the swerve drivetrain.
 *
 * <p>This class contains all hardware configuration for the swerve drive system, including:
 *
 * <ul>
 *   <li>PID gains for drive and steer motors
 *   <li>Physical dimensions (wheel radius, gear ratios, module positions)
 *   <li>Module-specific configurations (encoder offsets, motor IDs, inversions)
 *   <li>Simulation parameters (inertia, friction voltages)
 * </ul>
 *
 * <p>Generated using CTRE Tuner X Swerve Project Generator. Module encoder offsets should be
 * re-calibrated when encoders are changed or modules are rebuilt.
 *
 * @see <a href="https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html">
 *     CTRE Tuner X Swerve Documentation</a>
 */
public class DriveConstants {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with
    // the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains =
            new Slot0Configs()
                    .withKP(100)
                    .withKI(0)
                    .withKD(1.0)
                    .withKS(0.0)
                    .withKV(0.0)
                    .withKA(0)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains =
            new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKS(0).withKV(0.124);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The type of motor used for the drive motor
    private static final DriveMotorArrangement kDriveMotorType =
            DriveMotorArrangement.TalonFX_Integrated;
    // The type of motor used for the drive motor
    private static final SteerMotorArrangement kSteerMotorType =
            SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to
    // RemoteCANcoder
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.RemoteCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final Current kSlipCurrent = Amps.of(120.0);

    // Initial configs for the drive and steer motors and the azimuth encoder; these
    // cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API
    // documentation.
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs =
            new TalonFXConfiguration()
                    .withCurrentLimits(
                            new CurrentLimitsConfigs()
                                    // Swerve azimuth does not require much torque output, so we can
                                    // set a
                                    // relatively
                                    // low
                                    // stator current limit to help avoid brownouts without
                                    // impacting performance.
                                    .withStatorCurrentLimit(Amps.of(60))
                                    .withStatorCurrentLimitEnable(true));
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot
    public static final LinearVelocity kSpeedAt12Volts = FeetPerSecond.of(17.5);

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = (36.0 / 16.0);

    private static final double kDriveGearRatio = 6.0;
    private static final double kSteerGearRatio = 24.0;
    private static final Distance kWheelRadius = Inches.of(2);

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final int kPigeonId = 0;

    // These are only used for simulation
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.004);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.025);
    // Simulated voltage necessary to overcome friction
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    public static final SwerveDrivetrainConstants drivetrainConstants =
            new SwerveDrivetrainConstants()
                    .withCANBusName(Ports.DRIVETRAIN_BUS.getName())
                    .withPigeon2Id(kPigeonId)
                    .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            ConstantCreator =
                    new SwerveModuleConstantsFactory<
                                    TalonFXConfiguration,
                                    TalonFXConfiguration,
                                    CANcoderConfiguration>()
                            .withDriveMotorGearRatio(kDriveGearRatio)
                            .withSteerMotorGearRatio(kSteerGearRatio)
                            .withCouplingGearRatio(kCoupleRatio)
                            .withWheelRadius(kWheelRadius)
                            .withSteerMotorGains(steerGains)
                            .withDriveMotorGains(driveGains)
                            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                            .withSlipCurrent(kSlipCurrent)
                            .withSpeedAt12Volts(kSpeedAt12Volts)
                            .withDriveMotorType(kDriveMotorType)
                            .withSteerMotorType(kSteerMotorType)
                            .withFeedbackSource(kSteerFeedbackType)
                            .withDriveMotorInitialConfigs(driveInitialConfigs)
                            .withSteerMotorInitialConfigs(steerInitialConfigs)
                            .withEncoderInitialConfigs(encoderInitialConfigs)
                            .withSteerInertia(kSteerInertia)
                            .withDriveInertia(kDriveInertia)
                            .withSteerFrictionVoltage(kSteerFrictionVoltage)
                            .withDriveFrictionVoltage(kDriveFrictionVoltage);

    // Front Left
    private static final int kFrontLeftDriveMotorId = 5;
    private static final int kFrontLeftSteerMotorId = 6;
    private static final int kFrontLeftEncoderId = 11;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.384521484375);
    private static final boolean kFrontLeftSteerMotorInverted = false;
    private static final boolean kFrontLeftEncoderInverted = false;

    private static final Distance kFrontLeftXPos = Inches.of(10);
    private static final Distance kFrontLeftYPos = Inches.of(10);

    // Front Right
    private static final int kFrontRightDriveMotorId = 7;
    private static final int kFrontRightSteerMotorId = 8;
    private static final int kFrontRightEncoderId = 12;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(0.138427734375);
    private static final boolean kFrontRightSteerMotorInverted = false;
    private static final boolean kFrontRightEncoderInverted = false;

    private static final Distance kFrontRightXPos = Inches.of(10);
    private static final Distance kFrontRightYPos = Inches.of(-10);

    // Back Left
    private static final int kBackLeftDriveMotorId = 1;
    private static final int kBackLeftSteerMotorId = 2;
    private static final int kBackLeftEncoderId = 9;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.26806640625);
    private static final boolean kBackLeftSteerMotorInverted = false;
    private static final boolean kBackLeftEncoderInverted = false;

    private static final Distance kBackLeftXPos = Inches.of(-10);
    private static final Distance kBackLeftYPos = Inches.of(10);

    // Back Right
    private static final int kBackRightDriveMotorId = 3;
    private static final int kBackRightSteerMotorId = 4;
    private static final int kBackRightEncoderId = 10;
    private static final Angle kBackRightEncoderOffset = Rotations.of(0.1904296875);
    private static final boolean kBackRightSteerMotorInverted = false;
    private static final boolean kBackRightEncoderInverted = false;

    private static final Distance kBackRightXPos = Inches.of(-10);
    private static final Distance kBackRightYPos = Inches.of(-10);

    public static final SwerveModuleConstants<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            FrontLeft =
                    ConstantCreator.createModuleConstants(
                            kFrontLeftSteerMotorId,
                            kFrontLeftDriveMotorId,
                            kFrontLeftEncoderId,
                            kFrontLeftEncoderOffset,
                            kFrontLeftXPos,
                            kFrontLeftYPos,
                            kInvertLeftSide,
                            kFrontLeftSteerMotorInverted,
                            kFrontLeftEncoderInverted);
    public static final SwerveModuleConstants<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            FrontRight =
                    ConstantCreator.createModuleConstants(
                            kFrontRightSteerMotorId,
                            kFrontRightDriveMotorId,
                            kFrontRightEncoderId,
                            kFrontRightEncoderOffset,
                            kFrontRightXPos,
                            kFrontRightYPos,
                            kInvertRightSide,
                            kFrontRightSteerMotorInverted,
                            kFrontRightEncoderInverted);
    public static final SwerveModuleConstants<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            BackLeft =
                    ConstantCreator.createModuleConstants(
                            kBackLeftSteerMotorId,
                            kBackLeftDriveMotorId,
                            kBackLeftEncoderId,
                            kBackLeftEncoderOffset,
                            kBackLeftXPos,
                            kBackLeftYPos,
                            kInvertLeftSide,
                            kBackLeftSteerMotorInverted,
                            kBackLeftEncoderInverted);
    public static final SwerveModuleConstants<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            BackRight =
                    ConstantCreator.createModuleConstants(
                            kBackRightSteerMotorId,
                            kBackRightDriveMotorId,
                            kBackRightEncoderId,
                            kBackRightEncoderOffset,
                            kBackRightXPos,
                            kBackRightYPos,
                            kInvertRightSide,
                            kBackRightSteerMotorInverted,
                            kBackRightEncoderInverted);

    /**
     * Creates a CommandSwerveDrivetrain instance. This should only be called once in your robot
     * program,.
     */
    // public static CommandSwerveDrivetrain createDrivetrain() {
    // return new CommandSwerveDrivetrain(
    // DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
    // }

    /**
     * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types.
     */
    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         *
         * <p>This constructs the underlying hardware devices, so users should not construct the
         * devices themselves. If they need the devices, they can access them through getters in the
         * classes.
         *
         * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
         * @param modules Constants for each specific module
         */
        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         *
         * <p>This constructs the underlying hardware devices, so users should not construct the
         * devices themselves. If they need the devices, they can access them through getters in the
         * classes.
         *
         * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or
         *     set to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
         * @param modules Constants for each specific module
         */
        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(
                    TalonFX::new,
                    TalonFX::new,
                    CANcoder::new,
                    drivetrainConstants,
                    odometryUpdateFrequency,
                    modules);
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         *
         * <p>This constructs the underlying hardware devices, so users should not construct the
         * devices themselves. If they need the devices, they can access them through getters in the
         * classes.
         *
         * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or
         *     set to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
         * @param odometryStandardDeviation The standard deviation for odometry calculation in the
         *     form [x, y, theta]ᵀ, with units in meters and radians
         * @param visionStandardDeviation The standard deviation for vision calculation in the form
         *     [x, y, theta]ᵀ, with units in meters and radians
         * @param modules Constants for each specific module
         */
        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                Matrix<N3, N1> odometryStandardDeviation,
                Matrix<N3, N1> visionStandardDeviation,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(
                    TalonFX::new,
                    TalonFX::new,
                    CANcoder::new,
                    drivetrainConstants,
                    odometryUpdateFrequency,
                    odometryStandardDeviation,
                    visionStandardDeviation,
                    modules);
        }
    }

    /**
     * Creates and returns a Drive subsystem configured for the current robot mode.
     *
     * @return Drive subsystem instance with appropriate IO implementations
     */
    public static Drive get() {
        switch (Constants.currentMode) {
            case REAL:
                return new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(DriveConstants.FrontLeft),
                        new ModuleIOTalonFX(DriveConstants.FrontRight),
                        new ModuleIOTalonFX(DriveConstants.BackLeft),
                        new ModuleIOTalonFX(DriveConstants.BackRight));
            case SIM:
                return new Drive(
                        new GyroIO() {},
                        new ModuleIOSim("FrontLeft", DriveConstants.FrontLeft),
                        new ModuleIOSim("FrontRight", DriveConstants.FrontRight),
                        new ModuleIOSim("BackLeft", DriveConstants.BackLeft),
                        new ModuleIOSim("BackRight", DriveConstants.BackRight));
            case REPLAY:
                return new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {});
            default:
                throw new IllegalStateException("Unrecognized Robot Mode");
        }
    }

    /**
     * Maximum absolute pitch/roll angle at which the drivetrain is still considered level. When the
     * robot's pitch/roll exceeds this threshold, the drivetrain is treated as angled and
     * vision-based pose updates are ignored to prevent inaccurate pose estimation while the robot
     * is on an incline or bump.
     */
    public static final Angle ANGLED_TOLERANCE = Degrees.of(2.0);
}
