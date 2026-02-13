// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.mechanisms.rotary;

import java.util.Optional;
import frc.lib.io.absoluteencoder.AbsoluteEncoderIO;
import frc.lib.io.motor.MotorIO;

/**
 * A real implementation of the generic RotaryMechanism base class that interacts with a physical
 * motor through a MotorIO interface.
 */
public class RotaryMechanismReal extends RotaryMechanism<MotorIO, AbsoluteEncoderIO> {
    public RotaryMechanismReal(String name, MotorIO io, RotaryMechCharacteristics characteristics,
        Optional<AbsoluteEncoderIO> absoluteEncoder, String encoderName) {
        super(name, characteristics, io, absoluteEncoder, encoderName);
    }
}
