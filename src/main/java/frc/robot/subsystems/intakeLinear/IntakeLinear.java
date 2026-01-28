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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.mechanisms.linear.LinearMechanism;
import frc.lib.util.LoggedTunableNumber;

public class IntakeLinear extends SubsystemBase implements AutoCloseable {

    private static final LoggedTunableNumber INTAKE_CURRENT =
        new LoggedTunableNumber(IntakeLinearConstants.NAME + "/IntakeCurrent", 10.0);

        public final Trigger linearStopped;

    private final LinearMechanism io;

    public IntakeLinear(LinearMechanism intakeLinearIO)
    {
        this.io = intakeLinearIO;
        this.linearStopped = new Trigger(() -> isLinearStopped());
    }

    public Command extend() {
        return this.runOnce(() -> io.runCurrent(Amps.of(INTAKE_CURRENT.get())));
    }

    public Command retract() {
        return this.runOnce(() -> io.runCurrent(Amps.of(-INTAKE_CURRENT.get())));
    }

    //Repeating sequence to extend, wait until stopped, then retract
    public Command cycle() {
        return Commands.repeatingSequence(
            this.extend(),
            Commands.deadline(Commands.waitSeconds(1), Commands.waitUntil(linearStopped)),
            this.retract(),
            Commands.deadline(Commands.waitSeconds(1), Commands.waitUntil(linearStopped))
        );
    }

    public Command stop()
    {
        return this.runOnce(() -> io.runBrake());
    }

    private boolean isLinearStopped() {
        return Math.abs(io.getVelocity().in(RotationsPerSecond)) < 1;
    }


    @Override
    public void periodic()
    {
        io.periodic();
    }

    @Override
    public void close()
    {
        io.close();
    }
}
