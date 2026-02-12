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

import com.ctre.phoenix6.configs.SlotConfigs;
import lombok.With;

/**
 * Record representing PID and feedforward gains for motor control. Contains all necessary control
 * parameters for velocity and position control loops.
 *
 * @param P Proportional gain - corrects error proportionally
 * @param I Integral gain - corrects accumulated error over time
 * @param D Derivative gain - dampens oscillations by reacting to rate of change
 * @param A Acceleration feedforward - compensates for required acceleration
 * @param V Velocity feedforward - compensates for required velocity
 * @param G Gravity feedforward - compensates for gravity effects
 * @param S Static friction feedforward - overcomes static friction
 */
@With
public record PID(double P, double I, double D, double A, double V, double G, double S) {
    /**
     * Constructs a PID record with only P, I, and D gains. All feedforward gains are set to 0.0.
     *
     * @param P Proportional gain
     * @param I Integral gain
     * @param D Derivative gain
     */
    public PID(double P, double I, double D)
    {
        this(P, I, D, 0.0, 0.0, 0.0, 0.0);
    }

    /**
     * Constructs a PID record with all gains set to 0.0. Useful as a default starting point.
     */
    public PID()
    {
        this(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    /**
     * Converts this PID record to a CTRE Phoenix 6 SlotConfigs object.
     *
     * @return a SlotConfigs object with the gains from this PID
     */
    public SlotConfigs toSlotConfigs()
    {
        return new SlotConfigs()
            .withKP(P)
            .withKI(I)
            .withKD(D)
            .withKA(A)
            .withKV(V)
            .withKG(G)
            .withKS(S);
    }
}
