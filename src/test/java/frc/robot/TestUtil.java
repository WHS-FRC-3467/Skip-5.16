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

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TestUtil {

    private static Timer timer = new Timer();
    private static Timer loopTimer = new Timer();

    public static void runTest(Command command, double duration, Subsystem subsystem) {
        command.initialize();

        timer.restart();
        loopTimer.restart();
        boolean finished = false;
        while (timer.get() < duration) {
            if (!finished) {
                command.execute();
                if (command.isFinished()) {
                    finished = true;
                }
            }

            subsystem.periodic();
            while (!loopTimer.advanceIfElapsed(0.02)) {
                // wait until 20ms has passed
            }
        }

        command.end(!finished);
        subsystem.periodic();
        timer.stop();
        loopTimer.stop();
    }
}
