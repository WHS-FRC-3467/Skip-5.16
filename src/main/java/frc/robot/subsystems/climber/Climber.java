/* Copyright (C) 2026 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.linear.LinearMechanism;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggerHelper;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/** Add your docs here. */
public class Climber extends SubsystemBase {
    private final LinearMechanism<?> io;
    private Trigger homedTrigger;
    private Debouncer homeDebouncer = new Debouncer(0.1, DebounceType.kRising);

    private static final LoggedTunableNumber STOW_SETPOINT =
        new LoggedTunableNumber("Stow Height", 0.0);
    private static final LoggedTunableNumber RAISED_SETPOINT =
        new LoggedTunableNumber("Raised Height", 20.0);
    private static final LoggedTunableNumber DOWN_SETPOINT =
        new LoggedTunableNumber("Down Height", 2.0);

    @RequiredArgsConstructor
    @Getter
    public enum Setpoint {
        HOME(Inches.of(0.0)),
        STOW(Inches.of(STOW_SETPOINT.get())),
        RAISED(Inches.of(RAISED_SETPOINT.get())),
        DOWN(Inches.of(DOWN_SETPOINT.get()));

        private final Distance setpoint;

        public Angle getAngle()
        {
            return ClimberConstants.CONVERTER.toAngle(setpoint);
        }
    }

    public Climber(LinearMechanism<?> io)
    {
        this.io = io;
        homedTrigger =
            new Trigger(() -> homeDebouncer.calculate(io.getSupplyCurrent().gte(Amps.of(10))));
    }

    @Override
    public void periodic()
    {
        LoggerHelper.recordCurrentCommand(ClimberConstants.NAME, this);
        io.periodic();
    }

    public Command setGoal(Setpoint setpoint)
    {
        return this
            .runOnce(() -> io.runPosition(setpoint.getAngle(), PIDSlot.SLOT_0))
            .withName("Go To " + setpoint.toString() + " Setpoint");
    }

    public boolean nearGoal(Distance goalPosition)
    {
        System.out.println("Climber Position: " + ClimberConstants.CONVERTER.toDistance(io.getPosition()));
        System.out.println("Climber Goal: " + goalPosition.in(Inches));
        return io.nearGoal(goalPosition, ClimberConstants.TOLERANCE);
    }

    public Command waitUntilGoalCommand(Distance position)
    {
        return Commands.waitUntil(() -> {
            return nearGoal(position);
        });
    }

    public Command setGoalCommandWithWait(Setpoint setpoint)
    {
        return waitUntilGoalCommand(setpoint.getSetpoint())
            .deadlineFor(setGoal(setpoint))
            .withName("Go To " + setpoint.toString() + " Setpoint with wait");
    }

    public Command homeCommand()
    {
        return Commands.sequence(
            runOnce(() -> io.runVoltage(Volts.of(-2))),
            Commands.waitUntil(homedTrigger),
            runOnce(() -> io.setEncoderPosition(Setpoint.HOME.getAngle())),
            setGoal(Setpoint.STOW))
            .withName("Homing");
    }

    public AngularVelocity getVelocity()
    {
        return io.getVelocity();
    }

    public LinearVelocity getLinearVelocity()
    {
        return ClimberConstants.CONVERTER.toDistance(io.getVelocity().times(Seconds.of(1)))
            .div(Seconds.of(1));
    }
    
    public void close()
    {
        io.close();
    }
}
