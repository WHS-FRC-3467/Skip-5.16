// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import frc.robot.Constants;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Class for a tunable measure with units. Gets value from dashboard in tuning mode, returns default
 * if not or value not in dashboard.
 *
 * <p>
 * Example usage:
 * 
 * <pre>{@code
 * // Create a tunable current with default of 30 Amps - no casting needed!
 * LoggedTunableMeasure<Current> intakeCurrent =
 *     new LoggedTunableMeasure<>("Intake/Current", Amps, 30.0);
 * 
 * // Use it in code - returns Current measure automatically
 * motor.runCurrent(intakeCurrent.get());
 * }</pre>
 * 
 * @param <M> The measure type (e.g., Current, Distance, Voltage)
 */
@SuppressWarnings("UnusedVariable")
public class LoggedTunableMeasure<M> implements Supplier<M> {
    private static final String tableKey = "Tuning";

    private final String key;
    private final Unit unit;
    private boolean hasDefault = false;
    private double defaultMagnitude;
    private LoggedNetworkNumber dashboardNumber;
    private Map<Integer, Double> lastHasChangedValues = new HashMap<>();

    /**
     * Create a new LoggedTunableMeasure
     *
     * @param dashboardKey Key on dashboard
     * @param unit The unit of measure (e.g., Units.Amps, Units.Meters)
     */
    public LoggedTunableMeasure(String dashboardKey, Unit unit)
    {
        this.key = tableKey + "/" + dashboardKey + " " + unit.name(); 
        this.unit = unit;
    }

    /**
     * Create a new LoggedTunableMeasure with the default value
     *
     * @param dashboardKey Key on dashboard
     * @param unit The unit of measure (e.g., Units.Amps, Units.Meters)
     * @param defaultMagnitude Default magnitude value
     */
    public LoggedTunableMeasure(String dashboardKey, Unit unit, double defaultMagnitude)
    {
        this(dashboardKey, unit);
        initDefault(defaultMagnitude);
    }

    /**
     * Set the default magnitude of the measure. The default value can only be set once.
     *
     * @param defaultMagnitude The default magnitude value
     */
    public void initDefault(double defaultMagnitude)
    {
        if (!hasDefault) {
            hasDefault = true;
            this.defaultMagnitude = defaultMagnitude;
            if (Constants.tuningMode) {
                dashboardNumber = new LoggedNetworkNumber(key, defaultMagnitude);
            }
        }
    }

    /**
     * Get the current value as a Measure, from dashboard if available and in tuning mode.
     *
     * @return The current value as a Measure with the specified unit
     */
    @Override
    @SuppressWarnings("unchecked")
    public M get()
    {
        if (!hasDefault) {
            return (M) unit.of(0.0);
        } else {
            double magnitude = Constants.tuningMode ? dashboardNumber.get() : defaultMagnitude;
            return (M) unit.of(magnitude);
        }
    }

    /**
     * Get the current magnitude value (without units), from dashboard if available and in tuning
     * mode.
     *
     * @return The current magnitude value
     */
    public double getMagnitude()
    {
        if (!hasDefault) {
            return 0.0;
        } else {
            return Constants.tuningMode ? dashboardNumber.get() : defaultMagnitude;
        }
    }

    /**
     * Get the unit of this measure
     *
     * @return The unit
     */
    public Unit getUnit()
    {
        return unit;
    }

    /**
     * Checks whether the measure has changed since our last check
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
     *        objects. Recommended approach is to pass the result of "hashCode()"
     * @return True if the measure has changed since the last time this method was called, false
     *         otherwise.
     */
    public boolean hasChanged(int id)
    {
        double currentValue = getMagnitude();
        Double lastValue = lastHasChangedValues.get(id);
        if (lastValue == null || currentValue != lastValue) {
            lastHasChangedValues.put(id, currentValue);
            return true;
        }

        return false;
    }

    /**
     * Runs action if any of the tunableMeasures have changed
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
     *        objects. Recommended approach is to pass the result of "hashCode()"
     * @param action Callback to run when any of the tunable measures have changed. Access tunable
     *        measures in order inputted in method
     * @param tunableMeasures All tunable measures to check
     */
    public static void ifChanged(
        int id, Consumer<Measure<?>[]> action, LoggedTunableMeasure<?>... tunableMeasures)
    {
        if (Arrays.stream(tunableMeasures)
            .anyMatch(tunableMeasure -> tunableMeasure.hasChanged(id))) {
            action.accept(
                Arrays.stream(tunableMeasures).map(LoggedTunableMeasure::get)
                    .toArray(Measure[]::new));
        }
    }

    /**
     * Runs action if any of the tunableMeasures have changed
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
     *        objects. Recommended approach is to pass the result of "hashCode()"
     * @param action Runnable to execute when any tunable measure changes
     * @param tunableMeasures All tunable measures to check
     */
    public static void ifChanged(int id, Runnable action,
        LoggedTunableMeasure<?>... tunableMeasures)
    {
        ifChanged(id, values -> action.run(), tunableMeasures);
    }
}
