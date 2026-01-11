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

package frc.robot.subsystems.servo1;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.devices.Servo;
import frc.lib.io.servo.ServoIO;
import frc.lib.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import static edu.wpi.first.units.Units.Degrees;
import org.littletonrobotics.junction.Logger;

public class Servo1 extends SubsystemBase {

    private static final LoggedTunableNumber RETRACTED_SETPOINT =
        new LoggedTunableNumber("Servo1/Retracted", 0.0);
    private static final LoggedTunableNumber EXTENDED_SETPOINT =
        new LoggedTunableNumber("Servo1/Extended", 180.0);

    @RequiredArgsConstructor
    @SuppressWarnings("Immutable")
    @Getter
    public enum Setpoint {
        IDLE(null),
        RETRACTED(Degrees.of(RETRACTED_SETPOINT.get())),
        EXTENDED(Degrees.of(EXTENDED_SETPOINT.get()));

        private final Angle output;
    }

    Setpoint setpoint = Setpoint.IDLE;

    private final Servo servo;

    public Servo1(ServoIO io)
    {
        servo = new Servo(io);
    }

    @Override
    public void periodic()
    {
        Logger.recordOutput(Servo1Constants.NAME + "/Setpoint", setpoint.toString());
    }

    public Command setGoal(Setpoint setpoint)
    {
        return this.runOnce(() -> {
            switch (setpoint) {
                case IDLE -> servo.stop();
                default -> servo.setAngle(setpoint.getOutput());
            }
            this.setpoint = setpoint;
        });
    }
}
