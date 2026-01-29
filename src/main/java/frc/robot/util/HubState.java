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

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.AccessLevel;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class HubState {

    private static double[] HUB_CHANGE_TIMES = {
            130.0,
            105.0,
            80.0,
            55.0,
            30.0,
    };

    private static double SECONDS_BEFORE = 5.0;

    @Getter(lazy = true)
    private static final HubState instance = new HubState();

    public Trigger hubChange = new Trigger(() -> isHubCloseToActive());
    public Trigger hubActive = new Trigger(this::isAllianceHubActive);

    public DriverStation.Alliance getActiveAlliance;

    public void checkActiveAlliance()
    {
        if (findClosestTime() >= HUB_CHANGE_TIMES[0]) {
            getActiveAlliance = switch (DriverStation.getGameSpecificMessage().charAt(0)) {
                case 'R' -> Alliance.Red;
                case 'B' -> Alliance.Blue;
                default -> Alliance.Blue;
            };
        } else if (getActiveAlliance == Alliance.Blue) {
            getActiveAlliance = Alliance.Red;
        } else if (getActiveAlliance == Alliance.Red) {
            getActiveAlliance = Alliance.Blue;
        }

    }

    private double findClosestTime()
    {
        double getTime = DriverStation.getMatchTime();

        for (int i = 0; i < HUB_CHANGE_TIMES.length; i++) {
            if (getTime > HUB_CHANGE_TIMES[i]) {
                return HUB_CHANGE_TIMES[i];
            }
        }
        return 0.0;
    }

    public boolean isHubCloseToActive()
    {
        if (DriverStation.getMatchTime() - findClosestTime() <= SECONDS_BEFORE) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isAllianceHubActive()
    {
        if (DriverStation.getMatchTime() <= findClosestTime()) {
            checkActiveAlliance();
            return getActiveAlliance == DriverStation.getAlliance().get() ? true : false;
        } else {
            return false;
        }
    }


}
