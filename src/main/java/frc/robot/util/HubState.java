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

    private static double SECONDS_BEFORE = 5.0; // const for the seconds before the hubChange trigger activates

    @Getter(lazy = true)
    private static final HubState instance = new HubState();

    public Trigger hubChange = new Trigger(this::isHubCloseToActive); // activates x seconds before the next hub change
    public Trigger hubActive = new Trigger(this::isAllianceHubActive); // tells the robot if the hub is active

    public DriverStation.Alliance getActiveAlliance = Alliance.Red;


    public void checkActiveAlliance()
    {
        if (DriverStation.getMatchTime() >= 130) { // init which only gets the data from the DS in the first 5 seconds of auto
            getActiveAlliance = switch (DriverStation.getGameSpecificMessage().charAt(0)) {
                case 'R' -> Alliance.Red;
                case 'B' -> Alliance.Blue;
                default -> Alliance.Blue;
            };
        } else if (getActiveAlliance == Alliance.Blue) { // simple switcher which alternates between R & B every time the fn is called
            getActiveAlliance = Alliance.Red;
        } else if (getActiveAlliance == Alliance.Red) {
            getActiveAlliance = Alliance.Blue;
        }

    }

    public double findClosestTime()
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


    private boolean toggle = true; // toggle turns off when the match time is 0.1 seconds away fro
    // the closest hub change times

    public boolean isAllianceHubActive()
    {

        if (toggle) {
            if (DriverStation.getMatchTime() - findClosestTime() <= 0.1) {
                checkActiveAlliance();
                toggle = false;
            }

        } else {
            if (DriverStation.getMatchTime() - findClosestTime() >= 0.1) {
                toggle = true;
            }
        }
        return getActiveAlliance == DriverStation.getAlliance().get() ? true : false;

    }


}
