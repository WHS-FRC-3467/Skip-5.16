package frc.lib.util;

import org.littletonrobotics.junction.Logger;

/** Helper that records a boolean to the logger only when it changes. */
public class LoggedBoolean {
    private final String key;
    private Boolean last = null;

    public LoggedBoolean(String key) {
        this.key = key;
    }

    /** Log the value if it differs from the last logged value. */
    public void log(boolean value) {
        if (last == null || value != last) {
            last = value;
            Logger.recordOutput(key, value);
        }
    }

    /** Force a log regardless of previous value. */
    public void force(boolean value) {
        last = value;
        Logger.recordOutput(key, value);
    }

    /** Get last recorded value (may be null). */
    public Boolean getLast() {
        return last;
    }
}
