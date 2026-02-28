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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.Mechanism.TunablePidConfig;
import frc.lib.posestimator.SwerveOdometry.OdometryObservation;
import frc.lib.util.LoggedTunableBoolean;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggerHelper;
import frc.lib.util.PID;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import frc.robot.util.LocalADStarAK;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    public final RobotState robotState = RobotState.getInstance();

    // TunerConstants doesn't include these constants, so they are declared locally
    static final double ODOMETRY_FREQUENCY =
            new CANBus(DriveConstants.drivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;

    public static final double DRIVE_BASE_RADIUS =
            Math.max(
                    Math.max(
                            Math.hypot(
                                    DriveConstants.FrontLeft.LocationX,
                                    DriveConstants.FrontLeft.LocationY),
                            Math.hypot(
                                    DriveConstants.FrontRight.LocationX,
                                    DriveConstants.FrontRight.LocationY)),
                    Math.max(
                            Math.hypot(
                                    DriveConstants.BackLeft.LocationX,
                                    DriveConstants.BackLeft.LocationY),
                            Math.hypot(
                                    DriveConstants.BackRight.LocationX,
                                    DriveConstants.BackRight.LocationY)));

    public static final List<Translation2d> MODULE_TRANSLATIONS =
            List.of(
                    new Translation2d(
                            DriveConstants.FrontLeft.LocationX, DriveConstants.FrontLeft.LocationY),
                    new Translation2d(
                            DriveConstants.FrontRight.LocationX,
                            DriveConstants.FrontRight.LocationY),
                    new Translation2d(
                            DriveConstants.BackLeft.LocationX, DriveConstants.BackLeft.LocationY),
                    new Translation2d(
                            DriveConstants.BackRight.LocationX,
                            DriveConstants.BackRight.LocationY));

    // PathPlanner config constants
    private static final double ROBOT_MASS_KG = 63.5;
    private static final double ROBOT_MOI = 8.57;
    private static final double WHEEL_COF = 1.2;

    private static final RobotConfig PP_CONFIG =
            new RobotConfig(
                    ROBOT_MASS_KG,
                    ROBOT_MOI,
                    new ModuleConfig(
                            DriveConstants.FrontLeft.WheelRadius,
                            DriveConstants.kSpeedAt12Volts.in(MetersPerSecond),
                            WHEEL_COF,
                            DCMotor.getKrakenX60Foc(1)
                                    .withReduction(DriveConstants.FrontLeft.DriveMotorGearRatio),
                            DriveConstants.FrontLeft.SlipCurrent,
                            1),
                    MODULE_TRANSLATIONS.toArray(Translation2d[]::new));

    static final Lock odometryLock = new ReentrantLock();

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final Module[] modules = new Module[4]; // FL, FR, BL, BR

    private final SysIdRoutine sysId;

    private final Alert gyroDisconnectedAlert =
            new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    private final LoggedTunableBoolean enableSkidDetection =
            new LoggedTunableBoolean("Drive/Enable Skid Detection", true);

    private SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(MODULE_TRANSLATIONS.toArray(Translation2d[]::new));

    /*
     * Wheel skid detection for odometry: We compute a per-sample badWheels array and attach it to
     * the odometry observation. The odometry integrator can then ignore those wheels for that
     * integration step.
     *
     * The detection checks how consistent each wheel's translational velocity is compared to the
     * others. A wheel that is slipping is often an outlier.
     */
    private static final LoggedTunableNumber SKID_OUTLIER_SCALE =
            new LoggedTunableNumber("Drive/SkidOutlierScale", 1.35);

    /*
     * At very low translation speeds, encoder quantization and noise can cause false positives. We
     * skip skid detection if the median translational speed is below this threshold.
     */
    private static final LoggedTunableNumber SKID_MIN_TRANSLATION_MPS =
            new LoggedTunableNumber("Drive/SkidMinTranslationMPS", 0.15);

    @AutoLogOutput(key = "Drive/Skid/BadWheelsLatest")
    private boolean[] skidBadWheelsLatest = new boolean[] {false, false, false, false};

    @AutoLogOutput(key = "Drive/Skid/TransMagLatest")
    private double[] skidTranslationalSpeedMagnitudesLatest = new double[] {0.0, 0.0, 0.0, 0.0};

    // Reused buffers for skid detection to avoid per-sample allocations at high odometry rates.
    private final SwerveModuleState[] skidMeasuredStatesScratch =
            new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
            };
    private final double[] skidTranslationalSpeedScratch = new double[] {0.0, 0.0, 0.0, 0.0};
    private final boolean[] skidBadWheelsScratch = new boolean[] {false, false, false, false};

    private final TunablePidConfig driveTunablePID;
    private final TunablePidConfig steerTunablePID;

    public TunablePidConfig makeTunablePID(String prefix, PID defaultPid) {
        LoggedTunableNumber kp =
                new LoggedTunableNumber("Drive/PID/" + prefix + "/kP", defaultPid.P());
        LoggedTunableNumber ki =
                new LoggedTunableNumber("Drive/PID/" + prefix + "/kI", defaultPid.I());
        LoggedTunableNumber kd =
                new LoggedTunableNumber("Drive/PID/" + prefix + "/kD", defaultPid.D());
        LoggedTunableNumber ka =
                new LoggedTunableNumber("Drive/PID/" + prefix + "/kA", defaultPid.A());
        LoggedTunableNumber kv =
                new LoggedTunableNumber("Drive/PID/" + prefix + "/kV", defaultPid.V());
        LoggedTunableNumber kg =
                new LoggedTunableNumber("Drive/PID/" + prefix + "/kG", defaultPid.G());
        LoggedTunableNumber ks =
                new LoggedTunableNumber("Drive/PID/" + prefix + "/kS", defaultPid.S());
        int id = Objects.hash(this, prefix);
        return new TunablePidConfig(PIDSlot.SLOT_0, kp, ki, kd, ka, kv, kg, ks, id);
    }

    /**
     * Constructs a new Drive subsystem.
     *
     * @param gyroIO IO interface for the gyro
     * @param flModuleIO IO interface for the front left module
     * @param frModuleIO IO interface for the front right module
     * @param blModuleIO IO interface for the back left module
     * @param brModuleIO IO interface for the back right module
     */
    public Drive(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;

        modules[0] = new Module(flModuleIO, 0, DriveConstants.FrontLeft);
        modules[1] = new Module(frModuleIO, 1, DriveConstants.FrontRight);
        modules[2] = new Module(blModuleIO, 2, DriveConstants.BackLeft);
        modules[3] = new Module(brModuleIO, 3, DriveConstants.BackRight);

        // Usage reporting for swerve template
        HAL.report(
                tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        if (!AutoBuilder.isConfigured()) {
            AutoBuilder.configure(
                    robotState::getEstimatedPose,
                    robotState::resetPose,
                    this::getChassisSpeeds,
                    this::runVelocity,
                    new PPHolonomicDriveController(
                            new PIDConstants(6.0, 0.0, 0.1), new PIDConstants(8.0, 0.0, 0.1)),
                    PP_CONFIG,
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this);

            Pathfinding.setPathfinder(new LocalADStarAK());

            PathPlannerLogging.setLogActivePathCallback(
                    (activePath) -> {
                        Logger.recordOutput(
                                "Odometry/Trajectory",
                                activePath.toArray(new Pose2d[activePath.size()]));
                    });

            PathPlannerLogging.setLogTargetPoseCallback(
                    (targetPose) -> {
                        Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                    });
        }

        // Configure SysId
        sysId =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) ->
                                        Logger.recordOutput("Drive/SysIdState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

        driveTunablePID =
                makeTunablePID(
                        "Drive",
                        new PID(SlotConfigs.from(DriveConstants.FrontLeft.DriveMotorGains)));
        steerTunablePID =
                makeTunablePID(
                        "Steer",
                        new PID(SlotConfigs.from(DriveConstants.FrontLeft.SteerMotorGains)));
    }

    @Override
    @SuppressWarnings("LockNotBeforeTry")
    public void periodic() {
        LoggerHelper.recordCurrentCommand("Drive", this);

        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        LoggedTunableNumber.ifChanged(
                driveTunablePID.id,
                () -> {
                    for (Module module : modules) {
                        module.setDrivePID(
                                driveTunablePID.slot,
                                new PID(
                                        driveTunablePID.kp.get(),
                                        driveTunablePID.ki.get(),
                                        driveTunablePID.kd.get(),
                                        driveTunablePID.ka.get(),
                                        driveTunablePID.kv.get(),
                                        driveTunablePID.kg.get(),
                                        driveTunablePID.ks.get()));
                    }
                },
                driveTunablePID.kp,
                driveTunablePID.ki,
                driveTunablePID.kd,
                driveTunablePID.ka,
                driveTunablePID.kv,
                driveTunablePID.kg,
                driveTunablePID.ks);

        LoggedTunableNumber.ifChanged(
                steerTunablePID.id,
                () -> {
                    for (Module module : modules) {
                        module.setTurnPID(
                                steerTunablePID.slot,
                                new PID(
                                        steerTunablePID.kp.get(),
                                        steerTunablePID.ki.get(),
                                        steerTunablePID.kd.get(),
                                        steerTunablePID.ka.get(),
                                        steerTunablePID.kv.get(),
                                        steerTunablePID.kg.get(),
                                        steerTunablePID.ks.get()));
                    }
                },
                steerTunablePID.kp,
                steerTunablePID.ki,
                steerTunablePID.kd,
                steerTunablePID.ka,
                steerTunablePID.kv,
                steerTunablePID.kg,
                steerTunablePID.ks);

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

        double[] sampleTimestamps = modules[0].getOdometryTimestamps();
        int sampleCount = sampleTimestamps.length;

        for (int sampleIndex = 0; sampleIndex < sampleCount; sampleIndex++) {
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            for (int i = 0; i < 4; i++) {
                modulePositions[i] = modules[i].getOdometryPositions()[sampleIndex];
            }

            Optional<Rotation2d> gyroAngle = Optional.empty();
            if (gyroInputs.connected) {
                gyroAngle = Optional.of(gyroInputs.yawPosition);
            }

            boolean[] badWheels =
                    enableSkidDetection.get()
                            ? computeSkidMaskForSample(
                                    sampleIndex, sampleTimestamps, modulePositions)
                            : clearAndReturnFalseSkidMask();

            // Keep the latest sample easy to view in AdvantageScope.
            if (sampleIndex == sampleCount - 1) {
                System.arraycopy(badWheels, 0, skidBadWheelsLatest, 0, skidBadWheelsLatest.length);
            }

            robotState.addOdometryObservation(
                    new OdometryObservation(
                            Seconds.of(sampleTimestamps[sampleIndex]),
                            modulePositions,
                            gyroAngle,
                            badWheels));
        }

        // Update RobotState velocity
        robotState.setVelocity(getChassisSpeeds());
        robotState.setDrivetrainAngled(isAngled());

        Logger.recordOutput(
                "Drive/Speed",
                new Translation2d(
                                        getChassisSpeeds().vxMetersPerSecond,
                                        getChassisSpeeds().vyMetersPerSecond)
                                .getNorm()
                        * -1);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.kSpeedAt12Volts);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /**
     * Runs the drive in a straight line with the specified drive output.
     *
     * @param output Drive output voltage (-12 to 12)
     */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules
     * will return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = MODULE_TRANSLATIONS.get(i).getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /**
     * Returns a command to run a quasistatic test in the specified direction.
     *
     * @param direction Direction to run the test (forward or reverse)
     * @return Command that runs the quasistatic test
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction))
                .withName("SysId Quasistatic " + direction.toString());
    }

    /**
     * Returns a command to run a dynamic test in the specified direction.
     *
     * @param direction Direction to run the test (forward or reverse)
     * @return Command that runs the dynamic test
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.dynamic(direction))
                .withName("SysId Dynamic " + direction.toString());
    }

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the modules.
     *
     * @return Array of module positions for all four modules
     */
    protected SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /**
     * Returns the measured chassis speeds of the robot.
     *
     * @return Current chassis speeds in meters per second and radians per second
     */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Returns the position of each module in radians.
     *
     * @return Array of drive positions in radians for all four modules
     */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /**
     * Returns the average velocity of the modules in rotations/sec (Phoenix native units).
     *
     * @return Average drive velocity in rotations per second
     */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /**
     * Returns the maximum linear speed in meters per sec.
     *
     * @return Maximum linear speed in meters per second
     */
    public double getMaxLinearSpeedMetersPerSec() {
        return DriveConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    /**
     * Returns the maximum angular speed in radians per sec.
     *
     * @return Maximum angular speed in radians per second
     */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    /**
     * Returns the acceleration of the gyro in the X direction.
     *
     * @return Acceleration in the X direction in G's
     */
    public double getAccelerationX() {
        return gyroIO.getAccelerationX();
    }

    /**
     * Returns the acceleration of the gyro in the Y direction.
     *
     * @return Acceleration in the Y direction in G's
     */
    public double getAccelerationY() {
        return gyroIO.getAccelerationY();
    }

    /**
     * Returns whether the drivetrain is operating at a significant angle.
     *
     * <p>This checks the current pitch and roll reported by the gyro against the configured maximum
     * allowed angle ({@link DriveConstants#ANGLED_TOLERANCE}). It is used to detect when the robot
     * is on an incline or traversing a bump so that vision-based pose updates can be temporarily
     * ignored while the drivetrain is not level.
     *
     * @return {@code true} if the absolute pitch or roll exceeds the allowed threshold, indicating
     *     the drivetrain is sufficiently angled; {@code false} otherwise.
     */
    public boolean isAngled() {
        double pitch = MathUtil.inputModulus(gyroIO.getPitch(), -180.0, 180.0);
        double roll = MathUtil.inputModulus(gyroIO.getRoll(), -180.0, 180.0);
        double tolerance = DriveConstants.ANGLED_TOLERANCE.in(Degrees);

        double absPitch = Math.abs(pitch);
        double absRollFrom180 = Math.abs(roll) - 180.0;

        return absPitch > tolerance || Math.abs(absRollFrom180) > tolerance;
    }

    /**
     * Computes a per-odometry-sample skid mask.
     *
     * <p>The returned array is aligned to the odometry sample at {@code sampleIndex}. This matters
     * because {@link frc.lib.posestimator.SwerveOdometry} integrates pose using those exact sampled
     * positions.
     *
     * <p>Approach:
     *
     * <ul>
     *   <li>Estimate each wheel's velocity at this sample from position delta divided by dt.
     *   <li>Use kinematics to estimate the robot's angular velocity (omega) for that sample.
     *   <li>For each wheel, subtract the rotational velocity component caused by omega.
     *   <li>Compare the remaining translational speed magnitudes across wheels.
     * </ul>
     *
     * <p>Skid rule: A wheel is flagged as skidding if its translational speed magnitude is an
     * outlier compared to the median wheel translational speed. This catches common cases like one
     * wheel spinning freely or one wheel being dragged.
     *
     * <p>When translation is very small, encoder noise and quantization dominate. In that case we
     * skip skid detection and return all-false to avoid false positives.
     *
     * @param sampleIndex index into the odometry sample arrays for this cycle
     * @param sampleTimestamps timestamps for the odometry samples, in seconds
     * @param positionsNow module positions at {@code sampleIndex}
     * @return boolean[4] where true means "ignore this wheel for this odometry update"
     */
    private boolean[] computeSkidMaskForSample(
            int sampleIndex, double[] sampleTimestamps, SwerveModulePosition[] positionsNow) {
        boolean[] badWheels = clearAndReturnFalseSkidMask();
        SwerveModuleState[] measuredStates = skidMeasuredStatesScratch;

        if (sampleIndex <= 0) {
            /*
             * In sim we often only have one odometry sample per cycle. In that case we cannot
             * compute velocity from position delta, so we use the instantaneous measured module
             * states for this cycle. It is quite rare to not have multiple samples on a real robot,
             * but if that happens, this isn't a bad metric.
             */
            for (int i = 0; i < 4; i++) {
                measuredStates[i].speedMetersPerSecond = modules[i].getVelocityMetersPerSec();
                measuredStates[i].angle = modules[i].getAngle();
            }
        } else {
            double secondsBetweenSamples =
                    sampleTimestamps[sampleIndex] - sampleTimestamps[sampleIndex - 1];

            if (secondsBetweenSamples <= 1e-6) {
                setAllZeros(skidTranslationalSpeedMagnitudesLatest);
                return badWheels;
            }

            for (int i = 0; i < 4; i++) {
                SwerveModulePosition previousPosition =
                        modules[i].getOdometryPositions()[sampleIndex - 1];
                double deltaDistanceMeters =
                        positionsNow[i].distanceMeters - previousPosition.distanceMeters;

                double speedMetersPerSecond = deltaDistanceMeters / secondsBetweenSamples;

                measuredStates[i].speedMetersPerSecond = speedMetersPerSecond;
                measuredStates[i].angle = positionsNow[i].angle;
            }
        }

        final double angularVelocityRadiansPerSecond =
                kinematics.toChassisSpeeds(measuredStates).omegaRadiansPerSecond;

        double[] translationalSpeedMagnitudesMetersPerSecond = skidTranslationalSpeedScratch;

        for (int i = 0; i < 4; i++) {
            Translation2d moduleLocation = MODULE_TRANSLATIONS.get(i);

            double measuredVx =
                    measuredStates[i].speedMetersPerSecond * measuredStates[i].angle.getCos();
            double measuredVy =
                    measuredStates[i].speedMetersPerSecond * measuredStates[i].angle.getSin();

            double rotationalVx = -angularVelocityRadiansPerSecond * moduleLocation.getY();
            double rotationalVy = angularVelocityRadiansPerSecond * moduleLocation.getX();

            double translationalVx = measuredVx - rotationalVx;
            double translationalVy = measuredVy - rotationalVy;
            translationalSpeedMagnitudesMetersPerSecond[i] =
                    Math.hypot(translationalVx, translationalVy);
        }

        System.arraycopy(
                translationalSpeedMagnitudesMetersPerSecond,
                0,
                skidTranslationalSpeedMagnitudesLatest,
                0,
                skidTranslationalSpeedMagnitudesLatest.length);

        double medianTranslationalSpeedMetersPerSecond =
                medianOfFour(translationalSpeedMagnitudesMetersPerSecond);

        if (medianTranslationalSpeedMetersPerSecond < SKID_MIN_TRANSLATION_MPS.get()) {
            return badWheels;
        }

        double minimumAcceptableSpeedMetersPerSecond =
                medianTranslationalSpeedMetersPerSecond / SKID_OUTLIER_SCALE.get();
        double maximumAcceptableSpeedMetersPerSecond =
                medianTranslationalSpeedMetersPerSecond * SKID_OUTLIER_SCALE.get();

        for (int i = 0; i < 4; i++) {
            double translationalSpeedMetersPerSecond =
                    translationalSpeedMagnitudesMetersPerSecond[i];

            badWheels[i] =
                    (translationalSpeedMetersPerSecond < minimumAcceptableSpeedMetersPerSecond)
                            || (translationalSpeedMetersPerSecond
                                    > maximumAcceptableSpeedMetersPerSecond);
        }

        return badWheels;
    }

    private boolean[] clearAndReturnFalseSkidMask() {
        for (int i = 0; i < skidBadWheelsScratch.length; i++) {
            skidBadWheelsScratch[i] = false;
        }
        return skidBadWheelsScratch;
    }

    private static void setAllZeros(double[] values) {
        for (int i = 0; i < values.length; i++) {
            values[i] = 0.0;
        }
    }

    private static double medianOfFour(double[] values) {
        // 4-element sorting network; returns average of middle two with zero allocations.
        double a = values[0];
        double b = values[1];
        double c = values[2];
        double d = values[3];

        if (a > b) {
            double t = a;
            a = b;
            b = t;
        }
        if (c > d) {
            double t = c;
            c = d;
            d = t;
        }
        if (a > c) {
            double t = a;
            a = c;
            c = t;
        }
        if (b > d) {
            double t = b;
            b = d;
            d = t;
        }
        if (b > c) {
            double t = b;
            b = c;
            c = t;
        }

        return 0.5 * (b + c);
    }
}
