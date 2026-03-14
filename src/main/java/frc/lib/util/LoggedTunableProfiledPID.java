// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * A ProfiledPIDController with tunable gains that can be adjusted from the dashboard.
 *
 * <p>This controller extends WPILib's ProfiledPIDController to add runtime tunability. PID gains
 * (kP, kI, kD) and motion profile constraints (max velocity and acceleration) can be adjusted
 * through NetworkTables when tuning mode is enabled.
 *
 * <p>Call {@link #updatePID()} periodically to check for and apply updated values from the
 * dashboard.
 *
 * <p>Example usage:
 *
 * <pre>{@code
 * LoggedTunableProfiledPID controller =
 *     new LoggedTunableProfiledPID("Arm/PID", 2.0, 0.0, 0.1, 2.0, 5.0);
 *
 * // In periodic():
 * controller.updatePID(); // Apply any changes from dashboard
 * double output = controller.calculate(position, setpoint);
 * }</pre>
 */
public class LoggedTunableProfiledPID extends ProfiledPIDController {

    // Tunable numbers
    private LoggedTunableNumber p;
    private LoggedTunableNumber i;
    private LoggedTunableNumber d;
    private LoggedTunableNumber maxVelocity;
    private LoggedTunableNumber maxAcceleration;

    /**
     * Constructs a tunable profiled PID controller with default period.
     *
     * @param name The logging key prefix for tunable values
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     * @param maxV Maximum velocity
     * @param maxA Maximum acceleration
     */
    public LoggedTunableProfiledPID(
            String name, double kP, double kI, double kD, double maxV, double maxA) {
        this(name, kP, kI, kD, maxV, maxA, .02);
    }

    /**
     * Constructs a tunable profiled PID controller with specified period.
     *
     * @param name The logging key prefix for tunable values
     * @param p Proportional gain
     * @param i Integral gain
     * @param d Derivative gain
     * @param maxVelocity Maximum velocity
     * @param maxAcceleration Maximum acceleration
     * @param period Loop period in seconds
     */
    public LoggedTunableProfiledPID(
            String name,
            double p,
            double i,
            double d,
            double maxVelocity,
            double maxAcceleration,
            double period) {
        super(p, i, d, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration), period);

        // Tunable numbers for PID and motion gain constants
        this.p = new LoggedTunableNumber(name + "/kP", p);
        this.i = new LoggedTunableNumber(name + "/kI", i);
        this.d = new LoggedTunableNumber(name + "/kD", d);

        this.maxVelocity = new LoggedTunableNumber(name + "/maxVelocity", maxVelocity);
        this.maxAcceleration = new LoggedTunableNumber(name + "/maxAcceleration", maxAcceleration);
    }

    /** Updates PID and motion profile constraints from tunable values if changed. */
    public void updatePID() {
        // If changed, update controller constants from Tuneable Numbers
        if (p.hasChanged(hashCode()) || i.hasChanged(hashCode()) || d.hasChanged(hashCode())) {
            this.setPID(p.get(), i.get(), d.get());
        }

        if (maxVelocity.hasChanged(hashCode()) || maxAcceleration.hasChanged(hashCode())) {
            this.setConstraints(
                    new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
        }
    }
}
