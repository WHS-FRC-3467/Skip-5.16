package frc.lib.mechanisms;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.units.measure.*;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.util.PID;
import org.littletonrobotics.junction.Logger;

/**
 * Wrapper around a rotational {@link Mechanism} that exposes linear (distance-based) control and
 * feedback.
 *
 * <p>This is useful for mechanisms such as elevators, winches, or drivetrains where linear motion
 * is derived from a rotating element with a known radius. The class transparently converts between
 * linear and angular units while delegating all standard motor control to the underlying mechanism.
 *
 * @param <T> The mechanism type being wrapped
 */
public class DistanceControlledMechanism<T extends Mechanism<?>> {

    /** Underlying rotational mechanism. */
    private final T mechanism;

    /** Radius used to convert between angular and linear motion (meters). */
    private final double radiusMeters;

    private String key;

    /**
     * Creates a distance-controlled wrapper around a rotational mechanism.
     *
     * @param mechanism The mechanism to wrap
     * @param radius Effective radius used for linear-to-angular conversion
     */
    public DistanceControlledMechanism(T mechanism, Distance radius) {
        this.mechanism = mechanism;
        this.radiusMeters = radius.in(Meters);
        this.key = mechanism.getName();

        if (this.radiusMeters <= 0.0) {
            throw new IllegalArgumentException("radius must be greater than 0 meters");
        }
    }

    /**
     * Sets an explicit key to log linear values with
     *
     * @param key The key to use
     * @return This object
     */
    public DistanceControlledMechanism<T> withKey(String key) {
        this.key = key;
        return this;
    }

    /**
     * Runs the mechanism to a target linear position.
     *
     * @param position Desired linear position
     * @param slot PID slot to use
     */
    public void runLinearPosition(Distance position, PIDSlot slot) {
        Angle angle = Radians.of(position.in(Meters) / radiusMeters);
        mechanism.runPosition(angle, slot);
    }

    /**
     * Runs the mechanism to a target linear position without a motion profile.
     *
     * @param position Desired linear position
     * @param slot PID slot to use
     */
    public void runUnprofiledLinearPosition(Distance position, PIDSlot slot) {
        Angle angle = Radians.of(position.in(Meters) / radiusMeters);
        mechanism.runUnprofiledPosition(angle, slot);
    }

    /**
     * Runs the mechanism at a target linear velocity with acceleration limiting.
     *
     * @param velocity Desired linear velocity
     * @param acceleration Maximum linear acceleration
     * @param slot PID slot to use
     */
    public void runLinearVelocity(
            LinearVelocity velocity, LinearAcceleration acceleration, PIDSlot slot) {

        AngularVelocity angularVelocity =
                RadiansPerSecond.of(velocity.in(MetersPerSecond) / radiusMeters);

        AngularAcceleration angularAcceleration =
                RadiansPerSecondPerSecond.of(
                        acceleration.in(MetersPerSecondPerSecond) / radiusMeters);

        mechanism.runVelocity(angularVelocity, angularAcceleration, slot);
    }

    /**
     * Gets the current linear position of the mechanism.
     *
     * @return Linear position
     */
    public Distance getLinearPosition() {
        double radians = mechanism.getPosition().in(Radians);
        return Meters.of(radians * radiusMeters);
    }

    /**
     * Gets the error between the target and current linear position.
     *
     * @return Linear position error
     */
    public Distance getLinearPositionError() {
        double radians = mechanism.getPositionError().in(Radians);
        return Meters.of(radians * radiusMeters);
    }

    /**
     * Gets the current linear velocity of the mechanism.
     *
     * @return Linear velocity
     */
    public LinearVelocity getLinearVelocity() {
        double radPerSec = mechanism.getVelocity().in(RadiansPerSecond);
        return MetersPerSecond.of(radPerSec * radiusMeters);
    }

    /**
     * Gets the error between the target and current linear velocity.
     *
     * @return Linear velocity error
     */
    public LinearVelocity getLinearVelocityError() {
        double radPerSec = mechanism.getVelocityError().in(RadiansPerSecond);
        return MetersPerSecond.of(radPerSec * radiusMeters);
    }

    /** Periodic update hook. Should be called regularly by the robot loop. */
    public void periodic() {
        mechanism.periodic();

        Logger.recordOutput(key + "/LinearPosition", getLinearPosition());
        Logger.recordOutput(key + "/LinearPositionError", getLinearPositionError());
        Logger.recordOutput(key + "/LinearVelocity", getLinearVelocity());
        Logger.recordOutput(key + "/LinearVelocityError", getLinearVelocityError());
    }

    /** Sets the mechanism to coast mode. */
    public void runCoast() {
        mechanism.runCoast();
    }

    /** Sets the mechanism to brake mode. */
    public void runBrake() {
        mechanism.runBrake();
    }

    /**
     * Runs the mechanism using direct voltage control.
     *
     * @param voltage Desired voltage output
     */
    public void runVoltage(Voltage voltage) {
        mechanism.runVoltage(voltage);
    }

    /**
     * Runs the mechanism using torque-producing current control.
     *
     * @param current Desired current
     */
    public void runCurrent(Current current) {
        mechanism.runCurrent(current);
    }

    /**
     * Runs the mechanism using torque-producing current control with a duty cycle limit.
     *
     * @param current Desired current
     * @param dutyCycle Desired dutycycle of current output, limiting top speed
     */
    public void runCurrent(Current current, double dutyCycle) {
        mechanism.runCurrent(current, dutyCycle);
    }

    /**
     * Runs the mechanism using duty cycle control.
     *
     * @param dutyCycle Fractional output between 0 and 1
     */
    public void runDutyCycle(double dutyCycle, boolean ignoringSoftLimits) {
        mechanism.runDutyCycle(dutyCycle, ignoringSoftLimits);
    }

    /**
     * Runs the mechanism to a specified angular position.
     *
     * @param position Target position
     * @param slot PID slot to use
     */
    public void runPosition(Angle position, PIDSlot slot) {
        mechanism.runPosition(position, slot);
    }

    /**
     * Runs the mechanism to a specific position without a motion profile.
     *
     * @param position Target position.
     * @param slot PID slot index.
     */
    public void runUnprofiledPosition(Angle position, PIDSlot slot) {
        mechanism.runUnprofiledPosition(position, slot);
    }

    /**
     * Runs the mechanism at a specified angular velocity.
     *
     * @param velocity Target velocity
     * @param acceleration Maximum acceleration
     * @param slot PID slot to use
     */
    public void runVelocity(
            AngularVelocity velocity, AngularAcceleration acceleration, PIDSlot slot) {

        mechanism.runVelocity(velocity, acceleration, slot);
    }

    /**
     * Updates the PID configuration for a slot.
     *
     * @param slot Slot to configure
     * @param pid PID values
     */
    public void setPID(PIDSlot slot, PID pid) {
        mechanism.setPID(slot, pid);
    }

    /**
     * Sets the encoder position.
     *
     * @param position Desired encoder position
     */
    public void setEncoderPosition(Angle position) {
        mechanism.setEncoderPosition(position);
    }

    /**
     * Gets the supply current draw.
     *
     * @return Supply current
     */
    public Current getSupplyCurrent() {
        return mechanism.getSupplyCurrent();
    }

    /**
     * Gets the applied voltage draw.
     *
     * @return Applied voltage
     */
    public Voltage getAppliedVoltage() {
        return mechanism.getAppliedVoltage();
    }

    /**
     * Gets the angular position.
     *
     * @return Current angle
     */
    public Angle getPosition() {
        return mechanism.getPosition();
    }

    /**
     * Gets the angular position error.
     *
     * @return Position error
     */
    public Angle getPositionError() {
        return mechanism.getPositionError();
    }

    /**
     * Gets the torque-producing current.
     *
     * @return Torque current
     */
    public Current getTorqueCurrent() {
        return mechanism.getTorqueCurrent();
    }

    /**
     * Gets the angular velocity.
     *
     * @return Angular velocity
     */
    public AngularVelocity getVelocity() {
        return mechanism.getVelocity();
    }

    /**
     * Gets the angular velocity error.
     *
     * @return Velocity error
     */
    public AngularVelocity getVelocityError() {
        return mechanism.getVelocityError();
    }

    /** Closes the mechanism and releases resources. */
    public void close() {
        mechanism.close();
    }
}
