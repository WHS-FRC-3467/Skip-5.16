package frc.lib.util;

import org.littletonrobotics.junction.Logger;

/** Helper that records an int to the logger only when it changes. */
public class LoggedInt {
    private final String key;
    private Integer last = null;

    public LoggedInt(String key) {
        this.key = key;
    }

    /**
     * Log the value if it differs from the last logged value.
     *
     * @param value The value to log.
     */
    public void log(int value) {
        if (last == null || value != last) {
            last = value;
            Logger.recordOutput(key, value);
        }
    }

    /** Force a log regardless of previous value. */
    public void force(int value) {
        last = value;
        Logger.recordOutput(key, value);
    }

    /** Get last recorded value (may be null). */
    public Integer getLast() {
        return last;
    }
}
