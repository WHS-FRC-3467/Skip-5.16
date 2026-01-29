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

@With
public record PID(double P, double I, double D, double A, double V, double G, double S) {
    public PID(double P, double I, double D)
    {
        this(P, I, D, 0.0, 0.0, 0.0, 0.0);
    }

    public PID()
    {
        this(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

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
