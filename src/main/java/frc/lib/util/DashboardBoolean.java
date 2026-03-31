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

/** Helper that puts a boolean to the dashboard only when it changes. */
public class DashboardBoolean {
    private final String key;
    private Boolean last = null;

    public DashboardBoolean(String key) {
        this.key = key;
    }

    /** Put the value if it differs from the last sent value. */
    public void put(boolean value) {
        if (last == null || value != last) {
            last = value;
            SmartDashboard.putBoolean(key, value);
        }
    }

    /** Force push to the dashboard regardless of previous value. */
    public void force(boolean value) {
        last = value;
        SmartDashboard.putBoolean(key, value);
    }

    /** Get last sent value (may be null). */
    public Boolean getLast() {
        return last;
    }
}
