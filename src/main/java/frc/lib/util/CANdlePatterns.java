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

    // used only in larson animation
    private List<Boolean> reversed;
    private List<Boolean> ableToReverse;

    public CANdlePatterns(int buffers) {
        reversed = new ArrayList<>(Collections.nCopies(buffers, false));
        ableToReverse = new ArrayList<>(Collections.nCopies(buffers, false));
    }

    /**
     * @param velocity how long it takes for one LED to be filled
     * @param color
     * @return WPIlib LEDPattern
     */
    public LEDPattern scrollFill(double velocity, Color color) {
        long now = RobotController.getTime();

        return (reader, writer) -> {
            int bufLen = reader.getLength();

            final double periodMicros =
                    Hertz.of(velocity).asPeriod().in(Microseconds)
                            * bufLen; // needs to be multiplied by the length of the buffer because
            // CTRE defines the velocity as the amount of time it takes
            // for one LED to be filled and WPIlib defines this as the
            // amount of time it takes for all the LEDs to be filled
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

    /**
     * fire like LED pattern with randomized flickers
     *
     * @param frameRate
     * @param ledSpacing
     * @return WPIlib LEDPattern
     */
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

    /**
     * Moves a "Scanner" back and forth across a buffer, named after Glen A. Larson as the scanner
     * was featured in his shows Knight Rider and Battle Star Galactica
     *
     * @param velocity The amount of time it takes for a single LED to update, passed in as Hertz
     * @param color
     * @param size Size of the scrolling
     * @param buffer Which LED Buffer to operate on
     * @param bounceMode String representation of {@link
     *     com.ctre.phoenix6.signals.LarsonBounceValue}
     * @return WPIlib LEDPattern
     */
    public LEDPattern larsonPattern(
            double velocity, Color color, int size, int buffer, String bounceMode) {

        return (reader, writer) -> {
            long now = RobotController.getTime();

            int bufLen = reader.getLength();

            double wereToBounce = 0; // Where the scanner should bounce
            wereToBounce =
                    switch (bounceMode) {
                        case "Front" -> bufLen - size;
                        case "Back" -> bufLen + size;
                        case "Center" -> Math.floor(bufLen / 2);
                        default -> 0;
                    };

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
                        writer.setLED(reversed.get(buffer) ? i : bufLen - 2 - i, color);
                        continue;
                    }
                }
                if (i > max - 1 && i < max + size) {
                    writer.setLED(reversed.get(buffer) ? bufLen - 2 - i : i, color);
                    continue;
                }
                if (max < bufLen) {
                    writer.setLED(reversed.get(buffer) ? bufLen - 2 - i : i, Color.kBlack);
                }
            }

            if (max == wereToBounce - 1) {
                ableToReverse.set(buffer, Boolean.TRUE);
            }
        };
    }
}
