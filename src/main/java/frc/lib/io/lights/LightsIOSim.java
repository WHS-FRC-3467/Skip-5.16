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
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.random.RandomGenerator;
import org.littletonrobotics.junction.Logger;

/** A simulated lights implementation */
public class LightsIOSim implements LightsIO {

    private Map<String, String> requestInfo;
    AddressableLED led;
    AddressableLEDBuffer buffer;
    ArrayList<AddressableLEDBufferView> views;

    /** Constructs a {@link LightsIOSim} object. */
    public LightsIOSim(List<LEDSegment> ledSegments) {
        this.led = new AddressableLED(0);

        this.views = new ArrayList<>();
        int largestEndIndex = 2;
        for (var ledSegment : ledSegments) {
            if (ledSegment.endIndex() > largestEndIndex) {
                largestEndIndex = ledSegment.endIndex();
            }
        }

        this.buffer = new AddressableLEDBuffer(largestEndIndex + 1);
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

    @Override
    public void setAnimation(ControlRequest request) {

        this.requestInfo = request.getControlInfo();

        double direction = 1.0;
        if (requestInfo.containsKey("Direction")) {
            String directionString = requestInfo.get("Direction");
            if (directionString != "Forward") {
                direction = -1.0;
            }
        }
        int slot = 0;
        if (requestInfo.containsKey("Slot")) {
            slot = Integer.valueOf(requestInfo.get("Slot"));
            Logger.recordOutput("LEDs/Slot" + slot, requestInfo.toString());
        }
        final Distance kLedSpacing = // do not set to double, LEDPattern casts the input as an int
                Inches.of(1);
        switch (requestInfo.get("Name")) {
            case "RainbowAnimation":
                final LEDPattern rainbow = LEDPattern.rainbow(255, 128);
                final LEDPattern scrollingRainbow =
                        rainbow.scrollAtAbsoluteSpeed(
                                InchesPerSecond.of(
                                        Double.valueOf(requestInfo.get("FrameRate")) * direction),
                                kLedSpacing);
                scrollingRainbow.applyTo(views.get(slot));
                led.setData(this.buffer);
                break;
            case "FireAnimation":
                final LEDPattern fire =
                        LEDPattern.gradient(
                                LEDPattern.GradientType.kContinuous,
                                Color.kOrange,
                                Color.kOrangeRed);
                double random = RandomGenerator.getDefault().nextDouble();

                final LEDPattern fireScroll =
                        fire.scrollAtAbsoluteSpeed(
                                        InchesPerSecond.of(
                                                Double.valueOf(requestInfo.get("FrameRate"))
                                                        * (direction / 0.5)),
                                        kLedSpacing)
                                .blink(
                                        Seconds.of(random == 0 ? 5.0 : random * 3),
                                        Milliseconds.of(1))
                                .breathe(Seconds.of((random == 0 ? 1.0 : random) * 5));
                fireScroll.applyTo(views.get(slot));
                led.setData(this.buffer);
                break;
            case "EmptyAnimation":
                LEDPattern.kOff.applyTo(views.get(slot));
                led.setData(this.buffer);
            default:
                break;
        }
    }
}
