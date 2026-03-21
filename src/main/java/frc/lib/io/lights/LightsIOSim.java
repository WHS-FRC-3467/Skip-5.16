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

package frc.lib.io.lights;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.controls.ControlRequest;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

import frc.lib.util.CANdlePatterns;

import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.random.RandomGenerator;

/** A simulated lights implementation */
public class LightsIOSim implements LightsIO {

    final LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    final LEDPattern fire =
            LEDPattern.gradient(
                    LEDPattern.GradientType.kContinuous, Color.kOrange, Color.kOrangeRed);

    private Map<String, String> requestInfo;
    private AddressableLED led;
    private AddressableLEDBuffer buffer;
    private ArrayList<AddressableLEDBufferView> views;

    private static CANdlePatterns candlePatterns;

    /** Translates CTRE LED Commands To WPILIB Sim LED Commands */
    public LightsIOSim(List<LEDSegment> ledSegments) {
        candlePatterns = new CANdlePatterns(ledSegments.size());
        this.led = new AddressableLED(0);

        this.views = new ArrayList<>();
        int largestEndIndex = 2; // must be 2 because that is the minimum end index
        for (var ledSegment : ledSegments) {
            if (ledSegment.endIndex() > largestEndIndex) {
                largestEndIndex = ledSegment.endIndex();
            }
        }

        this.buffer = new AddressableLEDBuffer(largestEndIndex + 1); // ctre to wpilib offset
        for (var ledSegment : ledSegments) {
            this.views.add(buffer.createView(ledSegment.startIndex(), ledSegment.endIndex()));
        }
        led.setLength(buffer.getLength());
        led.setData(this.buffer);
        led.start();
    }

    @Override
    public void updateLedsSim() {
        led.setData(this.buffer);
    }

    /**
     * @param key Map Key
     * @return Value From Map Of Key
     */
    String checkAndGet(String key) {
        if (requestInfo.containsKey(key)) {
            return requestInfo.get(key);
        } else {
            Logger.recordOutput(
                    this.toString() + "/KeyNotFound", "Request Does Not Contain Key Of: " + key);
            return "EmptyKey";
        }
    }

    Color colorParse() {
        if (requestInfo.containsKey("Color")) {
            String colorString = requestInfo.get("Color");
            String numString = "";
            int colorOut[] = {0, 0, 0};
            int colorCount = 0;
            for (int i = 5; i < colorString.length(); i++) {

                char colorIter = colorString.charAt(i);
                numString += colorIter;
                if (colorString.charAt(i + 1) == ',') {

                    colorOut[colorCount] = Integer.valueOf(numString);
                    numString = "";
                    colorCount++;

                    i += 2;
                    if (i == colorString.length() || colorCount >= 3) {
                        break;
                    }
                }
            }
            return new Color(colorOut[0], colorOut[1], colorOut[2]);

        } else {
            return new Color("#C0FFEE");
        }
    }

    @Override
    public void setAnimation(ControlRequest request) {

        this.requestInfo = request.getControlInfo();

        double direction = "Backward".equals(checkAndGet("Direction")) ? -1.0 : 1.0;

        int slot = Integer.valueOf(checkAndGet("Slot"));
        Logger.recordOutput("LEDs/Slot" + slot, requestInfo.toString());

        final Distance kLedSpacing = // do not set to double, LEDPattern casts the input as an int
                Inches.of(1);
        switch (requestInfo.get("Name")) {
            case "RainbowAnimation": // scrolling rainbow animation
                rainbow.scrollAtAbsoluteSpeed(
                                InchesPerSecond.of(
                                        Double.valueOf(checkAndGet("FrameRate")) * direction),
                                kLedSpacing)
                        .applyTo(views.get(slot));
                break;

            case "FireAnimation": //  fire like animation
                double random = RandomGenerator.getDefault().nextDouble();

                final LEDPattern fireScroll =
                        fire.scrollAtAbsoluteSpeed(
                                        InchesPerSecond.of(
                                                Double.valueOf(checkAndGet("FrameRate"))
                                                        * (direction / 0.5)),
                                        kLedSpacing)
                                .blink(
                                        Seconds.of((random == 0.0 ? 5.0 : random) * 3),
                                        Milliseconds.of(8));
                fireScroll.breathe(Seconds.of(3));
                fireScroll.applyTo(views.get(slot));
                break;
            case "ColorFlowAnimation":
                candlePatterns
                        .scrollFill(
                                Double.valueOf(checkAndGet("FrameRate")) * direction, colorParse())
                        .applyTo(views.get(slot));
                break;
            case "EmptyAnimation":
                LEDPattern.kOff.applyTo(views.get(slot));
                break;
            case "LarsonAnimation":
                candlePatterns
                        .larsonPatternFix(
                                Double.valueOf(checkAndGet("FrameRate")) * direction,
                                colorParse(),
                                Integer.valueOf(checkAndGet("Size")),
                                slot,
                                checkAndGet("BounceMode"))
                        .applyTo(views.get(slot));
                break;
            default:
                return;
        }
        led.setData(this.buffer);
    }
}
