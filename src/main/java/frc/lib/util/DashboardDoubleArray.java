/*
 * Copyright (C) 2026 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 */
package frc.lib.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;

/**
 * Helper that puts a double[] to the dashboard only when it changes beyond an absolute tolerance.
 * Comparison is element-wise; if array length changes or any element differs from the last by more
 * than the tolerance, the array is sent.
 */
public class DashboardDoubleArray {
    private final String key;
    private final double tolerance;
    private double[] last = null;

    /** No tolerance (exact change required). */
    public DashboardDoubleArray(String key) {
        this(key, 0.0);
    }

    /**
     * Absolute tolerance only. Value is sent to the dashboard when any element differs from the
     * last by more than tolerance or when lengths differ.
     *
     * @param key logging key
     * @param tolerance minimum absolute delta required to trigger a log per element
     */
    public DashboardDoubleArray(String key, double tolerance) {
        this.key = key;
        this.tolerance = Math.abs(tolerance);
    }

    /** Send the array if it differs from the last logged array by more than the tolerance. */
    public void put(double[] values) {
        if (values == null) return;

        if (last == null) {
            last = Arrays.copyOf(values, values.length);
            SmartDashboard.putNumberArray(key, last);
            return;
        }

        if (last.length != values.length) {
            last = Arrays.copyOf(values, values.length);
            SmartDashboard.putNumberArray(key, last);
            return;
        }

        for (int i = 0; i < values.length; i++) {
            if (Math.abs(values[i] - last[i]) > tolerance) {
                last = Arrays.copyOf(values, values.length);
                SmartDashboard.putNumberArray(key, last);
                return;
            }
        }
    }

    /** Force push to the dashboard regardless of previous value. */
    public void force(double[] values) {
        last = values == null ? null : Arrays.copyOf(values, values.length);
        SmartDashboard.putNumberArray(key, last == null ? new double[0] : last);
    }

    /** Get last sent array (may be null). */
    public double[] getLast() {
        return last == null ? null : Arrays.copyOf(last, last.length);
    }
}
