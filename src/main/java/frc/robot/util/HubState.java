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

/**
 * Utility singleton that models the state of the game "hub" over the course of a match.
 * <p>
 * {@code HubState} uses {@link edu.wpi.first.wpilibj.DriverStation#getMatchTime()} together with a
 * fixed schedule of {@link #HUB_CHANGE_TIMES} to determine when the active hub changes between
 * alliances. It maintains an internal {@link DriverStation.Alliance} value that is updated via
 * {@link #checkActiveAlliance()} as the match time crosses each hub change boundary.
 * </p>
 * <p>
 * The class exposes two {@link edu.wpi.first.wpilibj2.command.button.Trigger} instances:
 * <ul>
 * <li>{@link #hubChange} – becomes active shortly before the next scheduled hub change, so
 * subsystems or commands can prepare for the transition.</li>
 * <li>{@link #hubActive} – indicates when the alliance hub tracked by this class is currently the
 * same as the robot's {@link DriverStation#getAlliance()}.</li>
 * </ul>
 * Typical usage is to obtain the singleton via {@code HubState.getInstance()} (generated from the
 * {@link #instance} field) and bind the triggers to commands that should run when the hub is about
 * to change or is active for the robot's alliance.
 */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class HubState {

    private static final double[] HUB_CHANGE_TIMES = {
            130.0,
            105.0,
            80.0,
            55.0,
            30.0,
    };

    private static final double SECONDS_BEFORE = 5.0; // const_for_the_seconds_before_the_hubChange_trigger_activates

    private static final double BUFFER = 0.1; // time offset so you dont overflow

    @Getter(lazy = true)
    private static final HubState instance = new HubState();
    @Getter
    public Trigger hubChange = new Trigger(this::isHubCloseToActive); // activates x seconds before
                                                                      // the next hub change
    public Trigger hubActive = new Trigger(this::isAllianceHubActive); // tells the robot if the hub
                                                                       // is active

    public DriverStation.Alliance getActiveAlliance = Alliance.Blue;


    public void checkActiveAlliance()
    {
        getActiveAlliance = getActiveAlliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
    }

    public void initActiveAlliance()
    {
        String gameSpecificMessage = DriverStation.getGameSpecificMessage();
        if (gameSpecificMessage != null && !gameSpecificMessage.isEmpty()) {
            getActiveAlliance = gameSpecificMessage.charAt(0) == 'B' ? Alliance.Blue : Alliance.Red;
        }
    }

    public double findClosestTime(double matchTime)
    {


        for (int i = 0; i < HUB_CHANGE_TIMES.length; i++) {
            if (matchTime > HUB_CHANGE_TIMES[i]) {
                return HUB_CHANGE_TIMES[i];
            }
        }
        return 0.0;
    }

    public boolean isHubCloseToActive()
    {
        double matchTime = DriverStation.getMatchTime();
        return matchTime - findClosestTime(matchTime) <= SECONDS_BEFORE;
    }


    private boolean toggle = true; // toggle turns off when the match time is 0.1 seconds away from
    // the closest hub change times

    public boolean isAllianceHubActive()
    {
        double matchTime = DriverStation.getMatchTime();

        if (toggle) {
            if (matchTime - findClosestTime(matchTime) <= BUFFER) {
                if (matchTime >= HUB_CHANGE_TIMES[0]) { // if match time is before the first
                                                        // transition time
                    initActiveAlliance();
                } else {
                    checkActiveAlliance();
                }
                toggle = false;
            }

        } else {
            if (matchTime - findClosestTime(matchTime) >= BUFFER) {
                toggle = true;
            }
        }
        if (matchTime < HUB_CHANGE_TIMES[4] + BUFFER | matchTime > HUB_CHANGE_TIMES[0]) {
            // if is endgame or is transition return true ^
            return true;
        }


        return getActiveAlliance != DriverStation.getAlliance().orElse(Alliance.Red);

    }


}
