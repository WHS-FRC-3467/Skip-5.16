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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.posestimator.SwerveOdometry.OdometryObservation;
import frc.lib.util.LoggerHelper;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.LocalADStarAK;
import frc.robot.RobotState;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    public final RobotState robotState = RobotState.getInstance();

    // TunerConstants doesn't include these constants, so they are declared locally
    static final double ODOMETRY_FREQUENCY =
        new CANBus(DriveConstants.drivetrainConstants.CANBusName).isNetworkFD()
            ? 250.0
            : 100.0;
    public static final double DRIVE_BASE_RADIUS = Math.max(
        Math.max(
            Math.hypot(DriveConstants.FrontLeft.LocationX, DriveConstants.FrontLeft.LocationY),
            Math.hypot(DriveConstants.FrontRight.LocationX, DriveConstants.FrontRight.LocationY)),
        Math.max(
            Math.hypot(DriveConstants.BackLeft.LocationX, DriveConstants.BackLeft.LocationY),
            Math.hypot(DriveConstants.BackRight.LocationX, DriveConstants.BackRight.LocationY)));

    // PathPlanner config constants
    private static final double ROBOT_MASS_KG = 74.088;
    private static final double ROBOT_MOI = 6.883;
    private static final double WHEEL_COF = 1.2;
    private static final RobotConfig PP_CONFIG = new RobotConfig(
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
        getModuleTranslations());

    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;
    private final Alert gyroDisconnectedAlert =
        new Alert("Disconnected gyro, using kinematics as fallback.",
            AlertType.kError);

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());

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
        ModuleIO brModuleIO)
    {
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0, DriveConstants.FrontLeft);
        modules[1] = new Module(frModuleIO, 1, DriveConstants.FrontRight);
        modules[2] = new Module(blModuleIO, 2, DriveConstants.BackLeft);
        modules[3] = new Module(brModuleIO, 3, DriveConstants.BackRight);

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive,
            tInstances.kRobotDriveSwerve_AdvantageKit);

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
                    new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                PP_CONFIG,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);
            Pathfinding.setPathfinder(new LocalADStarAK());
            PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                        "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                });
            PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                });
        }

        // Configure SysId
        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    }

    @Override
    @SuppressWarnings("LockNotBeforeTry")
    public void periodic()
    {
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

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

        double[] sampleTimestamps = modules[0].getOdometryTimestamps();
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
            }

            Optional<Rotation2d> gyroAngle = Optional.empty();
            if (gyroInputs.connected) {
                gyroAngle = Optional.of(gyroInputs.yawPosition);
            }

            robotState.addOdometryObservation(
                new OdometryObservation(Seconds.of(sampleTimestamps[i]), modulePositions,
                    gyroAngle));
        }

        // Update RobotState velocity
        robotState.setVelocity(getChassisSpeeds());
        robotState.setDrivetrainAngled(isAngled());

        Logger.recordOutput("Drive/Speed", new Translation2d(getChassisSpeeds().vxMetersPerSecond,
            getChassisSpeeds().vyMetersPerSecond).getNorm() * -1);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds)
    {
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
    public void runCharacterization(double output)
    {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Stops the drive. */
    public void stop()
    {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules
     * will return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX()
    {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
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
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction)
    {
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
    public Command sysIdDynamic(SysIdRoutine.Direction direction)
    {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0)
            .andThen(sysId.dynamic(direction))
            .withName("SysId Dynamic " + direction.toString());
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates()
    {
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
    protected SwerveModulePosition[] getModulePositions()
    {
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
    public ChassisSpeeds getChassisSpeeds()
    {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Returns the position of each module in radians.
     *
     * @return Array of drive positions in radians for all four modules
     */
    public double[] getWheelRadiusCharacterizationPositions()
    {
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
    public double getFFCharacterizationVelocity()
    {
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
    public double getMaxLinearSpeedMetersPerSec()
    {
        return DriveConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    /**
     * Returns the maximum angular speed in radians per sec.
     *
     * @return Maximum angular speed in radians per second
     */
    public double getMaxAngularSpeedRadPerSec()
    {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    /**
     * Returns an array of module translations.
     *
     * @return Array of Translation2d objects representing module positions relative to robot center
     */
    public static Translation2d[] getModuleTranslations()
    {
        return new Translation2d[] {
                new Translation2d(DriveConstants.FrontLeft.LocationX,
                    DriveConstants.FrontLeft.LocationY),
                new Translation2d(DriveConstants.FrontRight.LocationX,
                    DriveConstants.FrontRight.LocationY),
                new Translation2d(DriveConstants.BackLeft.LocationX,
                    DriveConstants.BackLeft.LocationY),
                new Translation2d(DriveConstants.BackRight.LocationX,
                    DriveConstants.BackRight.LocationY)
        };
    }

    /**
     * Returns the acceleration of the gyro in the X direction.
     *
     * @return Acceleration in the X direction in G's
     */
    public double getAccelerationX()
    {
        return gyroIO.getAccelerationX();
    }

    /**
     * Returns the acceleration of the gyro in the Y direction.
     *
     * @return Acceleration in the Y direction in G's
     */
    public double getAccelerationY()
    {
        return gyroIO.getAccelerationY();
    }

    /**
     * Returns whether the drivetrain is operating at a significant angle.
     *
     * <p>This checks the current pitch and roll reported by the gyro against the
     * configured maximum allowed angle ({@link DriveConstants#ANGLED_TOLERANCE}).
     * It is used to detect when the robot is on an incline or traversing a bump
     * so that vision-based pose updates can be temporarily ignored while the
     * drivetrain is not level.
     *
     * @return {@code true} if the absolute pitch or roll exceeds the allowed
     *         threshold, indicating the drivetrain is sufficiently angled;
     *         {@code false} otherwise.
     */
    public boolean isAngled() {
        return Math.abs(gyroIO.getPitch()) > DriveConstants.ANGLED_TOLERANCE.in(Degrees)
            || Math.abs(gyroIO.getRoll()) > DriveConstants.ANGLED_TOLERANCE.in(Degrees);
    }
}
