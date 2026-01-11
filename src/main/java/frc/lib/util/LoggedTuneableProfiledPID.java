// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class LoggedTuneableProfiledPID extends ProfiledPIDController {

    // Tunable numbers
    private LoggedTunableNumber p;
    private LoggedTunableNumber i;
    private LoggedTunableNumber d;
    private LoggedTunableNumber maxVelocity;
    private LoggedTunableNumber maxAcceleration;

    public LoggedTuneableProfiledPID(String name, double kP, double kI,
        double kD, double maxV, double maxA)
    {
        this(name, kP, kI, kD, maxV, maxA, .02);
    }

    public LoggedTuneableProfiledPID(String name, double p, double i,
        double d, double maxVelocity, double maxAcceleration, double period)
    {
        super(p, i, d, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration), period);

        // Tunable numbers for PID and motion gain constants
        this.p = new LoggedTunableNumber(name + "/kP", p);
        this.i = new LoggedTunableNumber(name + "/kI", i);
        this.d = new LoggedTunableNumber(name + "/kD", d);

        this.maxVelocity = new LoggedTunableNumber(name + "/maxVelocity", maxVelocity);
        this.maxAcceleration = new LoggedTunableNumber(name + "/maxAcceleration", maxAcceleration);
    }

    public void updatePID()
    {
        // If changed, update controller constants from Tuneable Numbers
        if (p.hasChanged(hashCode())
            || i.hasChanged(hashCode())
            || d.hasChanged(hashCode())) {
            this.setPID(p.get(), i.get(), d.get());
        }

        if (maxVelocity.hasChanged(hashCode())
            || maxAcceleration.hasChanged(hashCode())) {
            this.setConstraints(
                new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
        }
    }

}
