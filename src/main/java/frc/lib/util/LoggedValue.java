package frc.lib.util;

import org.littletonrobotics.junction.Logger;

import java.util.Objects;

/**
 * Generic logged value that records an object to the logger only when it changes (equals).
 *
 * <p>Notes: - Uses {@link Objects#equals} to detect changes. For mutable objects, callers should
 * provide an immutable snapshot or a value type with a proper equals implementation. - Arrays do
 * not compare element-wise via this helper (Object arrays are compared by reference via
 * Objects.equals). For arrays, either wrap them in a List or use a specialized helper.
 */
public class LoggedValue<T> {
    private final String key;
    private T last = null;
    // Tracks whether we've ever recorded a value.
    private boolean hasLast = false;

    public LoggedValue(String key) {
        this.key = key;
    }

    /** Log the value if it differs from the last logged value according to equals(). */
    public void log(T value) {
        if (!hasLast || !Objects.equals(last, value)) {
            last = value;
            hasLast = true;
            // Log as string to keep this generic helper simple and avoid tight coupling
            // to the Logger's typed overloads. Callers who need typed/structured logging
            // should use Logger.recordOutput directly with a supported type.
            Logger.recordOutput(key, String.valueOf(value));
        }
    }

    /** Force a log regardless of previous value. */
    public void force(T value) {
        last = value;
        hasLast = true;
        Logger.recordOutput(key, String.valueOf(value));
    }

    /** Get last recorded value (may be null). */
    public T getLast() {
        return last;
    }

    /** Returns true if a value has ever been recorded (including a recorded null). */
    public boolean hasLast() {
        return hasLast;
    }
}
