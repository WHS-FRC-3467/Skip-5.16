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
package frc.lib.util;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.random.RandomGenerator;

public class CANdlePatterns {

    private static List<Boolean> reversed;
    private static List<Boolean> ableToReverse;

    public CANdlePatterns(int buffers) {
        reversed = new ArrayList<>(Collections.nCopies(buffers, false));
        ableToReverse = new ArrayList<>(Collections.nCopies(buffers, false));
    }

    public LEDPattern scrollFill(double velocity, Color color) {
        long now = RobotController.getTime();

        return (reader, writer) -> {
            int bufLen = reader.getLength();

            final double periodMicros = Hertz.of(velocity).asPeriod().in(Microseconds) * bufLen;
            double t = (now % (long) periodMicros) / periodMicros;

            int max = (int) (bufLen * t);

            for (int led = 0; led < max; led++) {
                writer.setLED(led, color);
            }

            for (int led = max; led < bufLen; led++) {
                writer.setLED(led, Color.kBlack);
            }
        };
    }

    public LEDPattern fireScroll(double frameRate, Distance ledSpacing) {
        double random = RandomGenerator.getDefault().nextDouble();
        LEDPattern fireScroll =
                LEDPattern.gradient(
                                LEDPattern.GradientType.kContinuous,
                                Color.kOrange,
                                Color.kOrangeRed)
                        .scrollAtAbsoluteSpeed(InchesPerSecond.of(frameRate), ledSpacing)
                        .blink(Seconds.of((random == 0.0 ? 5.0 : random) * 3), Milliseconds.of(8));
        fireScroll.breathe(Seconds.of(3));
        return fireScroll;
    }

    public LEDPattern larsonPattern(
            double velocity, Color color, int size, int buffer, String bounceMode) {

        return (reader, writer) -> {
            long now = RobotController.getTime();

            int bufLen = reader.getLength();
            double wereToBounce = 0;
            switch (bounceMode) {
                case "Front":
                    wereToBounce = bufLen - size;
                    break;
                case "Back":
                    wereToBounce = bufLen + size;
                    break;
                case "Center":
                    wereToBounce = Math.floor(bufLen / 2);
                    break;
                default:
                    break;
            }

            final double periodMicros =
                    Hertz.of(velocity).asPeriod().in(Microseconds) * wereToBounce;
            double t = (now % (long) periodMicros) / periodMicros;
            int max = (int) (wereToBounce * t);

            if (max == 0 && ableToReverse.get(buffer)) {
                reversed.set(buffer, Boolean.valueOf(!reversed.get(buffer)));
                ableToReverse.set(buffer, Boolean.valueOf(false));
            }

            for (int i = 0; i < bufLen - 1; i++) {

                if (max > bufLen) {
                    int inner_max = max - bufLen;
                    if (i < inner_max) {
                        if (reversed.get(buffer)) {
                            writer.setLED(i, color);
                        } else {
                            writer.setLED(bufLen - 2 - i, color);
                        }

                        continue;
                    }
                }
                if (i > max - 1 && i < max + size) {

                    if (reversed.get(buffer)) {
                        writer.setLED(bufLen - 2 - i, color);
                    } else {
                        writer.setLED(i, color);
                    }
                    continue;
                }
                if (max < bufLen) {
                    if (reversed.get(buffer)) {
                        writer.setLED(bufLen - 2 - i, Color.kBlack);

                    } else {
                        writer.setLED(i, Color.kBlack);
                    }
                }
            }

            if (max == wereToBounce - 1) {
                ableToReverse.set(buffer, Boolean.TRUE);
            }
        };
    }
}
