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

/** Helper that puts a double to the dashboard only when it changes beyond an absolute tolerance. */
public class DashboardDouble {
    private final String key;
    private final double tolerance;
    private Double last = null;

    /** No tolerance (exact change required). */
    public DashboardDouble(String key) {
        this(key, 0.0);
    }

    /**
     * Absolute tolerance only. Value is sent to the dashboard when |value - last| > tolerance.
     *
     * @param key logging key
     * @param tolerance minimum absolute delta required to trigger a log
     */
    public DashboardDouble(String key, double tolerance) {
        this.key = key;
        this.tolerance = Math.abs(tolerance);
    }

    /** Send the value if it differs from the last logged value by more than the tolerance. */
    public void put(double value) {
        if (last == null) {
            last = value;
            SmartDashboard.putNumber(key, value);
            return;
        }

        double delta = Math.abs(value - last);
        if (delta > tolerance) {
            last = value;
            SmartDashboard.putNumber(key, value);
        }
    }

    /** Force push to the dashboard regardless of previous value. */
    public void force(double value) {
        last = value;
        SmartDashboard.putNumber(key, value);
    }

    /** Get last sent value (may be null). */
    public Double getLast() {
        return last;
    }
}
