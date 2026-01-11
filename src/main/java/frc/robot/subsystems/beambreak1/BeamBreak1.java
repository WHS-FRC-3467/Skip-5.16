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

package frc.robot.subsystems.beambreak1;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.devices.BeamBreak;
import frc.lib.io.beambreak.BeamBreakIO;

public class BeamBreak1 extends SubsystemBase {

    private final BeamBreak beamBreak;

    public final Trigger broken;

    public BeamBreak1(BeamBreakIO io)
    {
        beamBreak = new BeamBreak(io);

        broken = new Trigger(beamBreak::isBroken);
    }

    @Override
    public void periodic()
    {
        beamBreak.periodic();
    }
}
