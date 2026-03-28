package frc.lib.util;

import org.littletonrobotics.junction.Logger;

/** Helper that records a double to the logger only when it changes beyond an absolute tolerance. */
public class LoggedDouble {
    private final String key;
    private final double tolerance;
    private Double last = null;

    /** No tolerance (exact change required). */
    public LoggedDouble(String key) {
        this(key, 0.0);
    }

    /**
     * Absolute tolerance only. Value is logged when |value - last| > tolerance.
     *
     * @param key logging key
     * @param tolerance minimum absolute delta required to trigger a log
     */
    public LoggedDouble(String key, double tolerance) {
        this.key = key;
        this.tolerance = Math.abs(tolerance);
    }

    /** Log the value if it differs from the last logged value by more than the tolerance. */
    public void log(double value) {
        if (last == null) {
            last = value;
            Logger.recordOutput(key, value);
            return;
        }

        double delta = Math.abs(value - last);
        if (delta > tolerance) {
            last = value;
            Logger.recordOutput(key, value);
        }
    }

    /** Force a log regardless of previous value. */
    public void force(double value) {
        last = value;
        Logger.recordOutput(key, value);
    }

    /** Get last recorded value (may be null). */
    public Double getLast() {
        return last;
    }
}
