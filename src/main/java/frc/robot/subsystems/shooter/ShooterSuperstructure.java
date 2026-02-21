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

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.behaviortree.ActionNode;
import frc.lib.behaviortree.BehaviorTree;
import frc.lib.behaviortree.ConditionNode;
import frc.lib.behaviortree.NodeStatus;
import frc.lib.behaviortree.SelectorNode;
import frc.lib.behaviortree.SequenceNode;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.mechanisms.rotary.RotaryMechanism;
import frc.lib.util.LoggedTrigger;
import frc.lib.util.LoggedTunableBoolean;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggerHelper;
import frc.robot.Constants;
import frc.robot.FieldConstants.Hub;
import frc.robot.RobotState;
import frc.robot.RobotState.FieldRegion;
import frc.robot.RobotState.Target;
import org.littletonrobotics.junction.Logger;

public class ShooterSuperstructure extends SubsystemBase implements AutoCloseable {

    /** Distance from hub in meters -> hood angle in degrees */
    private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    static {
        hoodAngleMap.put(1.05, 0.0);
        hoodAngleMap.put(1.66, 14.0);
        hoodAngleMap.put(2.09, 15.0);
        hoodAngleMap.put(2.87, 18.0);
        hoodAngleMap.put(3.83, 20.0);
        hoodAngleMap.put(4.67, 24.0);
    }

    /** Distance from hub in meters -> flywheel speed in rotations per second */
    private static final InterpolatingDoubleTreeMap hubFlywheelMap =
            new InterpolatingDoubleTreeMap();

    static {
        hubFlywheelMap.put(1.05, 25.0);
        hubFlywheelMap.put(1.66, 25.0);
        hubFlywheelMap.put(2.09, 27.0);
        hubFlywheelMap.put(2.87, 30.0);
        hubFlywheelMap.put(3.83, 32.0);
        hubFlywheelMap.put(4.67, 35.5);
    }

    /** Distance from feed pose in meters -> flywheel speed in rotations per second */
    private static final InterpolatingDoubleTreeMap feedFlywheelMap =
            new InterpolatingDoubleTreeMap();

    static {
        feedFlywheelMap.put(3.0, 17.0);
        feedFlywheelMap.put(4.0, 18.0);
        feedFlywheelMap.put(5.0, 20.0);
        feedFlywheelMap.put(6.0, 22.0);
        feedFlywheelMap.put(7.0, 25.0);
        feedFlywheelMap.put(8.0, 27.0);
        feedFlywheelMap.put(9.0, 29.0);
        feedFlywheelMap.put(10.0, 30.5);
        feedFlywheelMap.put(11.0, 32.0);
        feedFlywheelMap.put(12.0, 34.0);
        feedFlywheelMap.put(13.0, 35.0);
    }

    private final RobotState robotState = RobotState.getInstance();

    private final RotaryMechanism<?, ?> hoodIO;
    private final FlywheelMechanism<?> leftFlywheelIO;
    private final FlywheelMechanism<?> rightFlywheelIO;

    /**
     * Behavior tree that supervises shooter setpoints each robot loop.
     *
     * <p>The tree makes two decisions every tick:
     *
     * <ol>
     *   <li><b>SpinFlywheel</b> — always sets the flywheel to the velocity interpolated from the
     *       robot's current distance to its target.
     *   <li><b>HoodControl</b> — selects the hood angle using a priority selector:
     *       <ul>
     *         <li><em>HoodSafety</em>: if the hood is NOT safe to actuate (trench or neutral zone),
     *             retract it to zero degrees.
     *         <li><em>SetHoodAngle</em>: otherwise, set the hood to the interpolated angle for the
     *             current distance.
     *       </ul>
     * </ol>
     *
     * <p>Access this tree from outside the subsystem via {@link #getDecisionTree()} for
     * visualization or inspection.
     */
    private final BehaviorTree shooterDecisionTree;

    public final LoggedTrigger readyToShoot =
            new LoggedTrigger(
                    this.getName() + "/readyToShoot",
                    () -> {
                        double dist = robotState.getDistanceToTarget().in(Meters);
                        return isFlywheelAt(RotationsPerSecond.of(hubFlywheelMap.get(dist)))
                                && isHoodAt(Degrees.of(hoodAngleMap.get(dist)));
                    });

    public final LoggedTrigger atHubSetpoints =
            new LoggedTrigger(
                    this.getName() + "/atHubSetpoints",
                    () -> {
                        // Distance between robot and hub centers
                        // Assume the Robot is pressed against the Hub. Hardcoded as part of no
                        // vision fallback.
                        double dist = (Hub.WIDTH + Constants.FULL_ROBOT_LENGTH.in(Meters)) / 2.0;
                        return isFlywheelAt(RotationsPerSecond.of(hubFlywheelMap.get(dist)))
                                && isHoodAt(Degrees.of(hoodAngleMap.get(dist)));
                    });

    // Trigger determining whether hood is safe to actuate. Primarily for use in autos.
    public final LoggedTrigger hoodSafe =
            new LoggedTrigger(
                    this.getName() + "/hoodSafe",
                    () ->
                            robotState.getFieldRegion() == FieldRegion.ALLIANCE_ZONE
                                    && robotState.enteringTrench.negate().getAsBoolean());

    private final LoggedTunableBoolean tuningMode =
            new LoggedTunableBoolean(getName() + "/Tuning/Enable", false);
    private final LoggedTunableNumber tuningFlywheelSpeedRPS =
            new LoggedTunableNumber(getName() + "/Tuning/FlywheelSpeedRPS", 0.0);
    private final LoggedTunableNumber tuningHoodAngleDegrees =
            new LoggedTunableNumber(getName() + "/Tuning/HoodAngleDegrees", 0.0);

    /**
     * Constructs a new ShooterSuperstructure subsystem with the specified hood and flywheel
     * mechanisms.
     *
     * @param hoodIO the hood mechanism for adjusting shot angle
     * @param leftFlywheelIO the left flywheel mechanism for spinning up shots
     * @param rightFlywheelIO the right flywheel mechanism for spinning up shots
     */
    public ShooterSuperstructure(
            RotaryMechanism<?, ?> hoodIO,
            FlywheelMechanism<?> leftFlywheelIO,
            FlywheelMechanism<?> rightFlywheelIO) {
        this.hoodIO = hoodIO;
        this.leftFlywheelIO = leftFlywheelIO;
        this.rightFlywheelIO = rightFlywheelIO;

        // Build the shooter decision tree.
        //
        // Every tick the tree:
        //   1. Sets the flywheel velocity based on robot distance to target.
        //   2. Sets the hood angle — retracting it when the hood is not safe to
        //      raise (trench / neutral zone), otherwise using the target angle.
        //
        // The tree is ticked inside supervisorTreeCommand(), which can be used
        // as a drop-in replacement for spinUpShooter() anywhere that decision
        // logging / visualization is desired.
        shooterDecisionTree =
                new BehaviorTree(
                        getName() + "/Decision",
                        new SequenceNode(
                                "SpinUp",
                                // Step 1 — always update flywheel setpoint
                                new ActionNode(
                                        "SpinFlywheel",
                                        () -> {
                                            spinFlywheel(getDesiredFlywheelVelocity());
                                            return NodeStatus.SUCCESS;
                                        }),
                                // Step 2 — decide hood angle
                                new SelectorNode(
                                        "HoodControl",
                                        // Priority A: retract if hood cannot be safely raised
                                        new SequenceNode(
                                                "HoodSafety",
                                                new ConditionNode(
                                                        "HoodNotSafe",
                                                        () -> !hoodSafe.getAsBoolean()),
                                                new ActionNode(
                                                        "RetractHood",
                                                        () -> {
                                                            setHoodPosition(Degrees.zero());
                                                            return NodeStatus.SUCCESS;
                                                        })),
                                        // Priority B: set desired angle for current distance
                                        new ActionNode(
                                                "SetHoodAngle",
                                                () -> {
                                                    setHoodPosition(getDesiredHoodAngle());
                                                    return NodeStatus.SUCCESS;
                                                }))));
    }

    private void spinFlywheel(AngularVelocity velocity) {
        leftFlywheelIO.runVelocity(velocity, FlywheelConstants.MAX_ACCELERATION, PIDSlot.SLOT_0);
        rightFlywheelIO.runVelocity(velocity, FlywheelConstants.MAX_ACCELERATION, PIDSlot.SLOT_0);
    }

    private boolean isFlywheelAt(AngularVelocity velocity) {
        return MathUtil.isNear(
                        velocity.in(RotationsPerSecond),
                        leftFlywheelIO.getVelocity().in(RotationsPerSecond),
                        FlywheelConstants.TOLERANCE.in(RotationsPerSecond))
                && MathUtil.isNear(
                        velocity.in(RotationsPerSecond),
                        rightFlywheelIO.getVelocity().in(RotationsPerSecond),
                        FlywheelConstants.TOLERANCE.in(RotationsPerSecond));
    }

    // Hood

    private void setHoodPosition(Angle angle) {
        hoodIO.runPosition(angle, PIDSlot.SLOT_0);
    }

    private boolean isHoodAt(Angle angle) {
        return hoodIO.nearGoal(angle, HoodConstants.TOLERANCE);
    }

    /**
     * Gets the current angle of the hood.
     *
     * @return the hood's current position angle
     */
    public Angle getHoodAngle() {
        return hoodIO.getPosition();
    }

    /**
     * Gets the average angular velocity of both flywheels.
     *
     * @return the average velocity of left and right flywheels
     */
    public AngularVelocity getAverageFlywheelVelocity() {
        return RotationsPerSecond.of(
                (leftFlywheelIO.getVelocity().in(RotationsPerSecond)
                                + rightFlywheelIO.getVelocity().in(RotationsPerSecond))
                        / 2.0);
    }

    /**
     * Gets the average linear velocity at the edge of both flywheels. Converts angular velocity to
     * linear velocity using the flywheel radius.
     *
     * @return the average linear velocity at the flywheel edge in meters per second
     */
    public LinearVelocity getAverageLinearVelocity() {
        return MetersPerSecond.of(
                getAverageFlywheelVelocity().in(RotationsPerSecond)
                        * 2.0
                        * Math.PI
                        * FlywheelConstants.FLYWHEEL_RADIUS.in(Meters));
    }

    private AngularVelocity getDesiredFlywheelVelocity() {
        InterpolatingDoubleTreeMap flywheelMap =
                switch (robotState.getTarget()) {
                    case HUB -> hubFlywheelMap;
                    case FEED_LEFT, FEED_RIGHT -> feedFlywheelMap;
                };

        return RotationsPerSecond.of(flywheelMap.get(robotState.getDistanceToTarget().in(Meters)));
    }

    private Angle getDesiredHoodAngle() {
        if (robotState.getTarget() == Target.HUB) {
            return Degrees.of(hoodAngleMap.get(robotState.getDistanceToTarget().in(Meters)));
        }

        return Degrees.of(0.0);
    }

    // Gets ball trajectory exit angle relative to horizontal, accounting for hood angle and
    // physical offset of the hood from horizontal
    public Angle getExitAngle() {
        return Degrees.of(90).minus(HoodConstants.MIN_ANGLE_OFFSET).minus(hoodIO.getPosition());
    }

    /**
     * Statically spins the flywheel and actuates the hood to the proper values for a HUB SHOT given
     * a provided distance. ONLY valid for HUB shots in the CURRENT ALLIANCE ZONE. If called in the
     * trench or neutral zone, will spin flywheel to proper speed but keep the hood low to prevent
     * collision. Perpetual command -- never spins down. Therefore, to end, this should be
     * interrupted by a parent command group or timed-out. Primarily for use in autos.
     *
     * @param distance the distance from the desired robot shot position to the HUB.
     * @return Static non-updating HUB only shooter spin-up command.
     */
    public Command spinUpShooterToHubDistance(Distance distance) {
        return Commands.run(
                        () -> {
                            spinFlywheel(
                                    RotationsPerSecond.of(hubFlywheelMap.get(distance.in(Meters))));
                            if (hoodSafe.getAsBoolean()) {
                                setHoodPosition(Degrees.of(hoodAngleMap.get(distance.in(Meters))));
                            } else {
                                setHoodPosition(Degrees.zero());
                            }
                        },
                        this)
                .withName("Spin-Up Shooter to Distance");
    }

    /**
     * Dynamically spins the flywheel and actuates the hood to the proper values for ANY target shot
     * given current field-relative robot pose. Valid for ANY target. Perpetual command -- never
     * spins down. Therefore, to end, this should be interrupted by a parent command group or
     * timed-out.
     *
     * @return Dynamically-updating ALL TARGET shooter spin-up command.
     */
    public Command spinUpShooter() {
        return Commands.run(
                        () -> {
                            spinFlywheel(getDesiredFlywheelVelocity());
                            setHoodPosition(getDesiredHoodAngle());
                        },
                        this)
                .withName("Spin-Up Shooter");
    }

    /**
     * Prepares the subsystem to shoot at the HUB, and runs a command while it is ready.
     *
     * <p>Internally uses {@link #supervisorTreeCommand()} so shooter decisions are logged to
     * AdvantageKit and visible in the Behavior Tree Viewer in real time.
     *
     * @param whileAtPosition A command that runs while the shooter is ready to shoot at the HUB. If
     *     shooting is disrupted because shooter readiness drops, attempt a flywheel/hood adjustment
     *     and, if successful, re-commence shooting. Only valid for HUB shots. This is usually used
     *     for starting and stopping the indexer to ensure balls are not shot unless we are
     *     confident we will make the shot. Shooter remains spun-up at the end of this command.
     * @return The command sequence
     */
    public Command prepareShot(Command whileAtPosition) {
        return Commands.sequence(
                        Commands.parallel(
                                supervisorTreeCommand(),
                                Commands.repeatingSequence(
                                        Commands.waitUntil(readyToShoot),
                                        whileAtPosition.until(readyToShoot.negate()))))
                .withName("Prepare Shot");
    }

    /**
     * Creates a command to set the hood to a specific angle.
     *
     * @param angle the target angle for the hood
     * @return command that sets the hood angle
     */
    public Command setHoodAngle(Angle angle) {
        return Commands.runOnce(() -> setHoodPosition(angle)).withName("Set Hood Angle");
    }

    /**
     * Creates a command to set the flywheel velocity (alternate spelling).
     *
     * @param velocity the target angular velocity for both flywheels
     * @return command that sets the flywheel speed
     */
    public Command setFlywheelSpeed(AngularVelocity velocity) {
        return Commands.runOnce(() -> spinFlywheel(velocity)).withName("Set Flywheel Speed");
    }

    /**
     * Creates a command that holds the shooter subsystem without actuating anything.
     *
     * <p>Useful as the final step of a sequence that needs to prevent any other command from
     * changing the hood position (e.g., after forcing the hood down when entering the trench). The
     * hood is continuously re-commanded to {@code Degrees.zero()} on every execute() tick to hold
     * this subsystem requirement and resist any position drift or interference.
     *
     * @return a perpetual command that keeps the hood retracted
     */
    public Command idle() {
        return this.run(() -> setHoodPosition(Degrees.zero())).withName("Idle");
    }

    /**
     * Creates a command that runs the {@linkplain #shooterDecisionTree shooter decision tree} every
     * robot loop, providing a fully-logged, visualizable alternative to {@link #spinUpShooter()}.
     *
     * <p>On initialization the tree is reset so it starts from a clean state. On end (interrupted
     * or finished) the flywheel is stopped and the hood is retracted to zero for safety.
     *
     * <p>The tree makes decisions every 20 ms loop:
     *
     * <ol>
     *   <li>Set the flywheel to the interpolated velocity for the current distance-to-target.
     *   <li>Set the hood to the interpolated angle — or retract it when the hood is not safe to
     *       raise (trench / neutral zone).
     * </ol>
     *
     * <p>Real-time execution is visible in the Behavior Tree Viewer at {@code
     * http://localhost:5800/behaviortree/index.html} (simulation) or {@code
     * http://roboRIO-3467-FRC.local:5800/behaviortree/index.html} (real robot).
     *
     * @return a perpetual command that ticks the shooter decision tree
     */
    public Command supervisorTreeCommand() {
        return this.run(() -> shooterDecisionTree.tick())
                .beforeStarting(Commands.runOnce(shooterDecisionTree::reset))
                .finallyDo(
                        () -> {
                            spinFlywheel(RotationsPerSecond.zero());
                            setHoodPosition(Degrees.zero());
                        })
                .withName("Shooter Supervisor");
    }

    /**
     * Returns the shooter's behavior tree for external access.
     *
     * <p>Useful for wiring the tree into dashboard visualizations or for inspection during
     * development.
     *
     * @return the {@link BehaviorTree} instance that supervises shooter setpoints
     */
    public BehaviorTree getDecisionTree() {
        return shooterDecisionTree;
    }

    @Override
    public void periodic() {
        if (tuningMode.get()) {
            if (tuningMode.hasChanged(hashCode())
                    || tuningFlywheelSpeedRPS.hasChanged(hashCode())
                    || tuningHoodAngleDegrees.hasChanged(hashCode())) {
                spinFlywheel(RotationsPerSecond.of(tuningFlywheelSpeedRPS.get()));
                setHoodPosition(Degrees.of(tuningHoodAngleDegrees.get()));
            }

            Logger.recordOutput(
                    getName() + "/Tuning/DistanceToTargetMeters",
                    robotState.getDistanceToTarget().in(Meters));
        }
        LoggerHelper.recordCurrentCommand(this.getName(), this);

        leftFlywheelIO.periodic();
        rightFlywheelIO.periodic();
        hoodIO.periodic();
    }

    /** Closes all underlying mechanisms and releases resources. */
    @Override
    public void close() {
        leftFlywheelIO.close();
        rightFlywheelIO.close();
        hoodIO.close();
    }
}
