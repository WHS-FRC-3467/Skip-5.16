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

import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.random.RandomGenerator;

/** A simulated lights implementation */
public class LightsIOSim implements LightsIO {

    private Map<String, String> requestInfo;
    private AddressableLED led;
    private AddressableLEDBuffer buffer;
    private ArrayList<AddressableLEDBufferView> views;

    /** Translates CTRE LED Commands To WPILIB Sim LED Commands */
    public LightsIOSim(List<LEDSegment> ledSegments) {
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

    @Override
    public void setAnimation(ControlRequest request) {

        this.requestInfo = request.getControlInfo();

        double direction = "Forward".equals(checkAndGet("Direction")) ? 1.0 : -1.0;

        int slot = Integer.valueOf(checkAndGet("Slot"));
        Logger.recordOutput("LEDs/Slot" + slot, requestInfo.toString());

        final Distance kLedSpacing = // do not set to double, LEDPattern casts the input as an int
                Inches.of(1);
        switch (requestInfo.get("Name")) {
            case "RainbowAnimation": // scrolling rainbow animation
                final LEDPattern rainbow = LEDPattern.rainbow(255, 128);
                final LEDPattern scrollingRainbow =
                        rainbow.scrollAtAbsoluteSpeed(
                                InchesPerSecond.of(
                                        Double.valueOf(checkAndGet("FrameRate")) * direction),
                                kLedSpacing);
                scrollingRainbow.applyTo(views.get(slot));
                led.setData(this.buffer);
                break;
            case "FireAnimation": //  fire like animation
                final LEDPattern fire =
                        LEDPattern.gradient(
                                LEDPattern.GradientType.kContinuous,
                                Color.kOrange,
                                Color.kOrangeRed);
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
                led.setData(this.buffer);
                break;
            case "EmptyAnimation":
                LEDPattern.kOff.applyTo(views.get(slot));
                led.setData(this.buffer);
                break;
            default:
                break;
        }
    }
}
