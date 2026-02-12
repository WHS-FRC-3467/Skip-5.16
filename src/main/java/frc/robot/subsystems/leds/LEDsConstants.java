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

package frc.robot.subsystems.leds;

import java.util.List;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;

import edu.wpi.first.wpilibj.util.Color;
import frc.lib.io.lights.LightsIO;
import frc.lib.io.lights.LightsIOCandle;
import frc.lib.io.lights.LightsIOSim;
import frc.robot.Constants;
import frc.robot.Ports;

public class LEDsConstants {
    public static final String NAME = "MainLEDs";

    public static final LEDSegment CANDLE_LEDS = new LEDSegment(0, 7, 0);
    public static final LEDSegment FRONT_STRIP = new LEDSegment(8, 10, 1);

    public static final CANdleConfiguration CANDLE_CONFIG = new CANdleConfiguration()
        .withCANdleFeatures(new CANdleFeaturesConfigs()
            .withEnable5VRail(Enable5VRailValue.Enabled)
            .withVBatOutputMode(VBatOutputModeValue.On)
            .withStatusLedWhenActive(StatusLedWhenActiveValue.Disabled))
        .withLED(new LEDConfigs()
            .withBrightnessScalar(1.0)
            .withStripType(StripTypeValue.RGB)
            .withLossOfSignalBehavior(LossOfSignalBehaviorValue.DisableLEDs));

    /**
     * Creates a real hardware lights IO implementation for CANdle.
     *
     * @return A LightsIOCandle instance for real robot hardware
     */
    public static final LightsIOCandle getLightsIOReal()
    {
        return new LightsIOCandle(NAME, Ports.lights, CANDLE_CONFIG);
    }

    /**
     * Creates a simulated lights IO implementation.
     *
     * @return A LightsIOSim instance for simulation
     */
    public static final LightsIOSim getLightsIOSim()
    {
        return new LightsIOSim(NAME);
    }

    /**
     * Creates a replay lights IO implementation that does nothing. Used when replaying logs without
     * hardware.
     *
     * @return An empty LightsIO implementation for replay mode
     */
    public static final LightsIO getLightsIOReplay()
    {
        return new LightsIO() {};
    }

    /**
     * Factory method to create an LEDs subsystem instance. Creates the appropriate lights IO based
     * on the current robot mode (REAL, SIM, or REPLAY).
     *
     * @return A fully configured LEDs subsystem
     */
    public static LEDs get()
    {
        switch (Constants.currentMode) {
            case REAL:
                return new LEDs(new LightsIOCandle(NAME, Ports.lights, CANDLE_CONFIG));
            case SIM:
                return new LEDs(new LightsIOSim(NAME));
            case REPLAY:
                return new LEDs(new LightsIO() {});
            default:
                throw new IllegalStateException("Unrecognized Robot Mode");
        }
    }

    public record LEDSegment(int startIndex, int endIndex, int animationSlot) {
    };

    // Animations

    // Off
    public static final ControlRequest candleOff = new EmptyAnimation(CANDLE_LEDS.animationSlot);

    public static final ControlRequest frontOff = new EmptyAnimation(FRONT_STRIP.animationSlot);;

    public static final List<ControlRequest> offAnimation = List.of(candleOff, frontOff);

    // Disabled
    public static final ControlRequest candleDisabled =
        new RainbowAnimation(CANDLE_LEDS.startIndex, CANDLE_LEDS.endIndex)
            .withSlot(CANDLE_LEDS.animationSlot)
            .withFrameRate(10)
            .withBrightness(0.7)
            .withDirection(AnimationDirectionValue.Forward);

    public static final ControlRequest frontDisabled =
        new RainbowAnimation(FRONT_STRIP.startIndex, FRONT_STRIP.endIndex)
            .withSlot(FRONT_STRIP.animationSlot)
            .withFrameRate(10)
            .withBrightness(0.7)
            .withDirection(AnimationDirectionValue.Forward);

    public static final List<ControlRequest> disabledAnimation =
        List.of(candleDisabled, frontDisabled);

    // Auto
    public static final ControlRequest candleAuto =
        new FireAnimation(CANDLE_LEDS.startIndex, CANDLE_LEDS.endIndex)
            .withSlot(CANDLE_LEDS.animationSlot)
            .withFrameRate(10)
            .withBrightness(0.7)
            .withDirection(AnimationDirectionValue.Forward)
            .withCooling(0.1)
            .withSparking(1.0);

    public static final ControlRequest frontAuto =
        new FireAnimation(FRONT_STRIP.startIndex, FRONT_STRIP.endIndex)
            .withSlot(FRONT_STRIP.animationSlot)
            .withFrameRate(10)
            .withBrightness(0.7)
            .withDirection(AnimationDirectionValue.Forward)
            .withCooling(0.1)
            .withSparking(1.0);

    public static final List<ControlRequest> autoAnimation = List.of(candleAuto, frontAuto);

    // Flashing
    public static final ControlRequest candleFlash =
        new StrobeAnimation(CANDLE_LEDS.startIndex, CANDLE_LEDS.endIndex)
            .withSlot(CANDLE_LEDS.animationSlot)
            .withFrameRate(10)
            .withColor(new RGBWColor(Color.kRed));

}
