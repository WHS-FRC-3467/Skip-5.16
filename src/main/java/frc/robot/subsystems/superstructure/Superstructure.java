/*
 * Copyright (C) 2025 Windham Windup
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

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import java.util.function.Supplier;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.linear.LinearMechanism;
import frc.lib.mechanisms.rotary.RotaryMechanism;
import frc.lib.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class Superstructure extends SubsystemBase implements AutoCloseable {

    private static final LoggedTunableNumber LINEAR_STOW_SETPOINT =
        new LoggedTunableNumber("Superstructure/LinearStowInches", 0.0);
    private static final LoggedTunableNumber LINEAR_RAISED_SETPOINT =
        new LoggedTunableNumber("Superstructure/LinearRaisedInches", 30.0);

    private static final LoggedTunableNumber ROTARY_STOW_SETPOINT =
        new LoggedTunableNumber("Superstructure/RotaryStowDegrees", 0.0);
    private static final LoggedTunableNumber ROTARY_RAISED_SETPOINT =
        new LoggedTunableNumber("Superstructure/RotaryRaisedDegrees", 90.0);

    private final RotaryMechanism rotaryIO;
    private final LinearMechanism linearIO;

    @RequiredArgsConstructor
    @SuppressWarnings("ImmutableEnumChecker")
    @Getter
    public enum Setpoint {
        STOW(() -> Inches.of(LINEAR_STOW_SETPOINT.get()),
            () -> Degrees.of(ROTARY_STOW_SETPOINT.get())),
        RAISED(() -> Inches.of(LINEAR_RAISED_SETPOINT.get()),
            () -> Degrees.of(ROTARY_RAISED_SETPOINT.get()));

        private final Supplier<Distance> linearSetpoint;
        private final Supplier<Angle> rotarySetpoint;

        public Angle getLinearAngle()
        {
            return LinearConstants.CONVERTER.toAngle(linearSetpoint.get());
        }
    }

    public Superstructure(RotaryMechanism rotaryIO, LinearMechanism linearIO)
    {
        this.rotaryIO = rotaryIO;
        this.linearIO = linearIO;

        setGoal(SuperstructureConstants.DEFAULT_SETPOINT).schedule();
    }

    public boolean nearSetpoint(Setpoint setpoint)
    {
        return rotaryIO.nearGoal(setpoint.rotarySetpoint.get(), RotaryConstants.TOLERANCE) &&
            linearIO.nearGoal(setpoint.linearSetpoint.get(), LinearConstants.TOLERANCE);
    }

    public boolean nearSetpoint(Angle rotarySetpoint, Distance linearSetpoint)
    {
        return rotaryIO.nearGoal(rotarySetpoint, RotaryConstants.TOLERANCE) &&
            linearIO.nearGoal(linearSetpoint, LinearConstants.TOLERANCE);
    }

    public Command setGoal(Setpoint setpoint)
    {
        return Commands.sequence(
            this.runOnce(() -> rotaryIO.runPosition(
                Setpoint.STOW.rotarySetpoint.get(),
                PIDSlot.SLOT_0)),
            Commands.waitUntil(
                () -> rotaryIO.nearGoal(
                    Setpoint.STOW.rotarySetpoint.get(),
                    RotaryConstants.TOLERANCE)),
            this.runOnce(() -> linearIO.runPosition(
                setpoint.getLinearAngle(),
                PIDSlot.SLOT_0)),
            Commands.waitUntil(
                () -> linearIO.nearGoal(
                    Setpoint.STOW.linearSetpoint.get(),
                    LinearConstants.TOLERANCE)),
            this.runOnce(() -> rotaryIO.runPosition(
                setpoint.rotarySetpoint.get(),
                PIDSlot.SLOT_0)))
            .withName("Go To " + setpoint.toString() + " Setpoint");
    }

    public Command setGoal(Angle rotarySetpoint, Distance linearSetpoint)
    {
        return Commands.sequence(
            this.runOnce(() -> rotaryIO.runPosition(
                Setpoint.STOW.rotarySetpoint.get(),
                PIDSlot.SLOT_0)),
            Commands.waitUntil(
                () -> rotaryIO.nearGoal(
                    Setpoint.STOW.rotarySetpoint.get(),
                    RotaryConstants.TOLERANCE)),
            this.runOnce(() -> linearIO.runPosition(
                LinearConstants.CONVERTER.toAngle(linearSetpoint),
                PIDSlot.SLOT_0)),
            Commands.waitUntil(
                () -> linearIO.nearGoal(
                    linearSetpoint,
                    LinearConstants.TOLERANCE)),
            this.runOnce(() -> rotaryIO.runPosition(
                rotarySetpoint,
                PIDSlot.SLOT_0)))
            .withName("Go To Custom Setpoint: " + rotarySetpoint.in(Degrees) + " deg, "
                + linearSetpoint.in(Inches) + " in");
    }

    public Command setGoalWithWait(Setpoint setpoint)
    {
        return Commands.sequence(
            setGoal(setpoint),
            Commands.waitUntil(() -> nearSetpoint(setpoint)))
            .withName("Go To " + setpoint.toString() + " Setpoint With Wait");
    }

    @Override
    public void periodic()
    {
        rotaryIO.periodic();
        linearIO.periodic();
    }

    @Override
    public void close()
    {
        rotaryIO.close();
        linearIO.close();
    }
}
