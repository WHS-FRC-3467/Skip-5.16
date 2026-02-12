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

import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class MechanismSubsystem extends SubsystemBase {
  
    protected Mechanism<?> io;

    
   
protected MechanismSubsystem(Mechanism<?> io) {
    this.io = io;
}
   public double getLinearVelocity() {
        return io.getLinearVelocity().in(MetersPerSecond);
    }

    public void setLinearVelocity(LinearVelocity velocity) {
        // TODO
    }
     public void setLinearPosition( AngleUnit angle) {
        // TODO
    }

    public double getLinearPosition() {
        return io.getLinearPosition();
    }

    
}
