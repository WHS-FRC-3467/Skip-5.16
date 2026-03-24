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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class for a tunable number. Gets value from dashboard regardless of tuning mode, returns default
 * if not or value not in dashboard.
 */
public class AlwaysTunableNumber {

    private static final String tableKey = "TunableNumbers";

    private String key;
    private double defaultValue;
    private boolean hasDefault = false;

    /**
     * Create a new TunableNumber
     *
     * @param dashboardKey Key on dashboard
     */
    public AlwaysTunableNumber(String dashboardKey) {
        this.key = tableKey + "/" + dashboardKey;
    }

    /**
     * Create a new TunableNumber with the default value
     *
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     */
    public AlwaysTunableNumber(String dashboardKey, double defaultValue) {
        this(dashboardKey);
        setDefault(defaultValue);
    }

    /**
     * Set the default value of the number. The default value can only be set once.
     *
     * @param defaultValue The default value
     */
    public void setDefault(double defaultValue) {
        if (!hasDefault) {
            hasDefault = true;
            this.defaultValue = defaultValue;
            SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
        }
    }

    /**
     * Get the default value for the number that has been set
     *
     * @return The default value
     */
    public double getDefault() {
        return defaultValue;
    }

    /**
     * Publishes a new value. Note that the value will not be returned by {@link #get()} until the
     * next cycle.
     */
    public void set(double value) {
        SmartDashboard.putNumber(key, value);
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode.
     *
     * @return The current value, or 0.0 if there is no default number
     */
    public double get() {
        if (!hasDefault) {
            return 0.0;
        } else {
            return SmartDashboard.getNumber(key, defaultValue);
        }
    }
}
