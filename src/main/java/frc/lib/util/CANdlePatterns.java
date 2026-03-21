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
import static edu.wpi.first.units.Units.Microseconds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class CANdlePatterns {

    private static boolean reverser = false;
    private static boolean allowedToReverse = false;

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

    public LEDPattern larsonPatternFix(
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

                if (i > max - 1 && i < max + size) {

                    if (reversed.get(buffer)) {
                        writer.setLED(bufLen - 2 - i, color);
                    } else {
                        writer.setLED(i, color);
                    }
                    continue;
                } else if (i < size && bounceMode == "Back") {
                    if (reversed.get(buffer)) {
                        writer.setLED(bufLen - 2 - i, color);
                    } else {
                        writer.setLED(i, color);
                    }
                    continue;
                }

                if (reversed.get(buffer)) {
                    writer.setLED(bufLen - 2 - i, Color.kBlack);

                } else {
                    writer.setLED(i, Color.kBlack);
                }
            }

            if (max == wereToBounce - 1) {
                ableToReverse.set(buffer, Boolean.TRUE);
            }
        };
    }

    public LEDPattern larsonPattern(double velocity, Color color, int size) {
        return (reader, writer) -> {
            long now = RobotController.getTime();

            int bufLen = reader.getLength();

            final double periodMicros =
                    Hertz.of(velocity).asPeriod().in(Microseconds) * (bufLen - size);
            double t = (now % (long) periodMicros) / periodMicros;
            int max = (int) ((bufLen - size) * t);

            int[] list = new int[bufLen - 1];
            boolean inner_reverse = reverser;
            if (max == bufLen - size - 1) {
                inner_reverse = !reverser;
            }
            for (int i = 0; i < bufLen - 1; i++) {

                if (i > max - 1 && i < max + size) {

                    if (inner_reverse) {
                        list[bufLen - 2 - i] = 1;
                    } else {
                        list[i] = 1;
                    }
                }
            }
            System.out.println(Arrays.toString(list));

            if (max == bufLen - size - 1 && allowedToReverse) {
                System.out.println("switched! " + String.valueOf(max));
                allowedToReverse = false;
                reverser = !reverser;
            }
            if (max == 0) {
                allowedToReverse = true;
            }
        };
    }
}

/*

  mmmm    m
 #"   " mm#mm   mmm    m mm   mmm    mmmm   mmm
 "#mmm    #    #" "#   #"  " "   #  #" "#  #"  #
     "#   #    #   #   #     m"""#  #   #  #""""
 "mmm#"   "mm  "#m#"   #     "mm"#  "#m"#  "#mm"
                                     m  #
                                      ""

bufLen = 8
size = 3

max = 0
1110000
max = 1
0111000
max = 2
0011100
max = 3
0001110
max = 4
0000111
max = 5
1110000
max = 6
1110000
max = 7
1110000

1110000
0111000

0001110
0000111
1110000
0000111


1
0001110
2
0011100
3
0111000
4
0000111
0
1110000

*/

            /*  if (max - size > 0) {

                if (reverser && max != bufLen - 1) {

                    for (int led = 0; led < max - size; led++) {
                        writer.setLED(bufLen - 1 - led, Color.kBlack);
                    }
                    for (int led = max - size; led < max; led++) {

                        writer.setLED(bufLen - 1 - led, color);
                    }

                } else {
                    for (int led = 0; led < max - size; led++) {
                        writer.setLED(led, Color.kBlack);
                    }
                    for (int led = max - size; led < max; led++) {
                        writer.setLED(led, color);
                    }
                }

            } else {

                for (int led = 0; led < max; led++) {
                    writer.setLED(led, color);
                }
            }
            if (reverser && max != bufLen - 1) {
                for (int led = max; led < bufLen; led++) {
                    writer.setLED(bufLen - 1 - led, Color.kBlack);
                }
            } else {
                for (int led = max; led < bufLen; led++) {
                    writer.setLED(led, Color.kBlack);
                }
            } */
