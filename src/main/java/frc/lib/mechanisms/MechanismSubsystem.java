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
package frc.lib.mechanisms;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import lombok.Getter;
import lombok.Setter;

public class MechanismSubsystem extends SubsystemBase {
  
    protected Mechanism<?> io;

    
   
protected MechanismSubsystem(Mechanism<?> io) {
    this.io = io;
}
   public LinearVelocity getLinearVelocity() {
        return io.getLinearVelocity();
    }
    public Angle getPosAlso() {
        return io.getPosition();
    }
}
