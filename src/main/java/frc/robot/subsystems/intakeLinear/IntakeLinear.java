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

package frc.robot.subsystems.intakeLinear;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.mechanisms.linear.LinearMechanism;
import frc.lib.util.LoggedTrigger;
import frc.lib.util.LoggedTunableNumber;

/**
 * Subsystem that controls the linear intake extension mechanism.
 * The intake can extend outward to collect game pieces or retract inward for storage.
 * Uses current-based control to detect when the mechanism has reached its limits.
 */
public class IntakeLinear extends SubsystemBase implements AutoCloseable {

    private static final LoggedTunableNumber INTAKE_CURRENT =
        new LoggedTunableNumber(IntakeLinearConstants.NAME + "/IntakeCurrent", 10.0);

    public final LoggedTrigger linearStopped;
    public final LoggedTrigger isExtended;
    public final LoggedTrigger isRetracted;

    private final LinearMechanism<?> io;

    /**
     * Constructs an IntakeLinear subsystem.
     * 
     * @param intakeLinearIO The linear mechanism for controlling the intake extension
     */
    public IntakeLinear(LinearMechanism<?> intakeLinearIO)
    {
        this.io = intakeLinearIO;
        this.linearStopped = new LoggedTrigger("IntakeLinear/isLinearStopped",() -> isLinearStopped());
        this.isExtended = new LoggedTrigger("IntakeLinear/isExtended", () -> io.getTorqueCurrent().in(Amps) > INTAKE_CURRENT.get() * 0.8).and(linearStopped);
        this.isRetracted = new LoggedTrigger("IntakeLinear/isRetracted", () -> io.getTorqueCurrent().in(Amps) < -INTAKE_CURRENT.get() * 0.8).and(linearStopped);
    }

    /**
     * Creates a command to extend the intake.
     * 
     * @return A command that extends the intake
     */
    public Command extend()
    {
        return this.runOnce(() -> io.runCurrent(Amps.of(INTAKE_CURRENT.get())));
    }

    /**
     * Creates a command to retract the intake.
     * 
     * @return A command that retracts the intake
     */
    public Command retract()
    {
        return this.runOnce(() -> io.runCurrent(Amps.of(-INTAKE_CURRENT.get())));
    }

    /**
     * Creates a command that repeatedly cycles the intake between extended and retracted positions.
     * Each cycle extends the intake, waits until stopped, retracts, then waits until stopped again.
     * 
     * @return A repeating command that cycles the intake
     */
    public Command cycle()
    {
        return Commands.repeatingSequence(
            this.extend(),
            Commands.deadline(Commands.waitSeconds(1), Commands.waitUntil(linearStopped)),
            this.retract(),
            Commands.deadline(Commands.waitSeconds(1), Commands.waitUntil(linearStopped)));
    }

    /**
     * Creates a command to stop the intake linear mechanism.
     * 
     * @return A command that stops the linear motion
     */
    public Command stop()
    {
        return this.runOnce(() -> io.runBrake());
    }

    private boolean isLinearStopped()
    {
        return Math.abs(io.getVelocity().in(RotationsPerSecond)) < 2.0;
    }


    @Override
    public void periodic()
    {
        Logger.recordOutput(this.getName() + "/State",
            this.isExtended.getAsBoolean() ? "EXTENDED"
                : this.isRetracted.getAsBoolean() ? "RETRACTED" : "MOVING");
        io.periodic();
    }

    @Override
    public void close()
    {
        io.close();
    }
}
