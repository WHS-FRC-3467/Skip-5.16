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

import java.util.function.BooleanSupplier;

/**
 * Detects falling edges (transitions from true to false) in a boolean signal.
 * Useful for triggering one-time actions when a condition becomes false.
 */
public class FallingEdge implements BooleanSupplier {
    private final BooleanSupplier source;

    private boolean previousState = true;

    /**
     * Constructs a FallingEdge detector.
     *
     * @param source The boolean supplier to detect falling edges from
     */
    public FallingEdge(BooleanSupplier source)
    {
        this.source = source;
    }

    /**
     * Creates a FallingEdge detector from a boolean supplier.
     *
     * @param source The boolean supplier to detect falling edges from
     * @return A new FallingEdge instance
     */
    public static FallingEdge of(BooleanSupplier source)
    {
        return new FallingEdge(source);
    }

    @Override
    public boolean getAsBoolean()
    {
        boolean currentState = source.getAsBoolean();
        boolean fallingEdge = previousState && !currentState;
        previousState = currentState;
        return fallingEdge;
    }

}
