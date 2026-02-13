// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// https://github.com/frc1678/C2025-Public/blob/main/src/main/java/frc/lib/util/Util.java#L172

package frc.lib.util;

import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * Utility class for mechanism-related calculations and conversions.
 *
 * <p>
 * Provides tools for converting between linear and angular measurements, useful for mechanisms like
 * elevators, spools, and pulleys where rotation causes linear motion.
 */
public class MechanismUtil {

    /**
     * Converts between distance and angle measurements based on a fixed radius.
     *
     * <p>
     * This is useful for mechanisms where rotation causes linear motion (elevators, spools, drums)
     * or vice versa. For example, an elevator driven by a drum with a 2-inch radius: rotating the
     * drum by 1 radian will raise the elevator by 2 inches.
     */
    public static class DistanceAngleConverter {
        private final Distance radius;

        /**
         * Constructs a converter with the specified radius.
         *
         * @param radius The radius of the drum, pulley, or wheel
         */
        public DistanceAngleConverter(Distance radius) {
            this.radius = radius;
        }

        /**
         * Converts a distance measurement to an equal angle measurement based on radius initialized
         * with.
         *
         * @param distance Distance to convert to angle.
         * @return Angle distance is equivalent to.
         */
        public Angle toAngle(Distance distance) {
            return Radians.of(distance.in(BaseUnits.DistanceUnit) / radius.baseUnitMagnitude());
        }

        /**
         * Converts an angle measurement to an equal distance measurement based on radius
         * initialized with.
         *
         * @param angle to convert to distance.
         * @return Distance agle is equivalent to.
         */
        public Distance toDistance(Angle angle) {
            return BaseUnits.DistanceUnit.of(angle.in(Radians) * radius.baseUnitMagnitude());
        }

        /**
         * Gets an angle unit equivalent to a distance unit with the conversion of the radius
         * initialized with.
         *
         * @param unit The distance unit to convert.
         * @return The distance represented as an AngleUnit
         */
        public AngleUnit getDistanceUnitAsAngleUnit(DistanceUnit unit) {
            return Units.derive(BaseUnits.AngleUnit)
                .aggregate(toAngle(unit.one()).baseUnitMagnitude())
                .named(unit.name())
                .symbol(unit.symbol())
                .make();
        }

        /**
         * Gets a distance unit equivalent to a angle unit with the conversion of the radius
         * initialized with.
         *
         * @param unit The angle unit to convert.
         * @return The distance represented as a DistanceUnit
         */
        public DistanceUnit getAngleUnitAsDistanceUnit(AngleUnit unit) {
            return Units.derive(BaseUnits.DistanceUnit)
                .splitInto(toDistance(unit.one()).baseUnitMagnitude())
                .named(unit.name())
                .symbol(unit.symbol())
                .make();
        }

        /**
         * Gets the radius used for distance/angle conversions.
         *
         * @return The drum radius
         */
        public Distance getDrumRadius() {
            return radius;
        }
    }
}
