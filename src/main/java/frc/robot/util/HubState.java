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
import frc.lib.util.LoggedTrigger;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.AccessLevel;

/**
 * Utility singleton that models the state of the game "hub" over the course of a match.
 * <p>
 * {@code HubState} uses {@link edu.wpi.first.wpilibj.DriverStation#getMatchTime()} together with a
 * fixed schedule of {@link #HUB_CHANGE_TIMES} to determine when the active hub changes between
 * alliances. It maintains an internal {@link DriverStation.Alliance} value that is updated as the
 * match time crosses each hub change boundary.
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

    /**
     * Returns a copy of the hub change times array.
     *
     * @return a defensive copy of the hub change times
     */
    public static double[] getHubChangeTimes()
    {
        return Arrays.copyOf(HUB_CHANGE_TIMES, HUB_CHANGE_TIMES.length);
    }

    @Getter
    private static final HubState instance = new HubState();

    @Getter
    private LoggedTrigger hubActive =
        new LoggedTrigger("Hub/Hub Active Trigger", () -> isOurHubActive());
    // tells the robot if the hub is active

    private static final double SECONDS_BEFORE = 5.0;
    /** activates {@link #SECONDS_BEFORE} seconds before the next hub change */
    @Getter
    private LoggedTrigger hubChange =
        new LoggedTrigger("Hub/Hub is about to change (within " + SECONDS_BEFORE + " seconds)",
            () -> isCloseToSwitching());
    // Time (in seconds) before a hub change that the hubChange trigger activates


    // Stores the alliance whose hub is currently active - default to our alliance during Auto - set
    // to Blue as a backup
    private volatile Alliance activeAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    // Stores the alliance whose hub is active in the 1st shift
    private volatile Alliance firstActiveAlliance = Alliance.Blue;

    /**
     * Checks the active alliance based on the game-specific message from the DriverStation. Meant
     * to only be called at the first possible change time.
     */
    public void setFirstActiveAlliance()
    {
        String gameSpecificMessage = DriverStation.getGameSpecificMessage();
        // The game-specific message indicates which alliance's hub is INACTIVE during the first
        // alliance shift. We therefore set firstActiveAlliance to the *other* alliance:
        // - If the first character is 'B', the Blue hub is inactive, so the Red hub is active.
        // - If the first character is 'R', the Red hub is inactive, so the Blue hub is active.
        if (gameSpecificMessage != null && !gameSpecificMessage.isEmpty()) {
            firstActiveAlliance =
                gameSpecificMessage.charAt(0) == 'B' ? Alliance.Red : Alliance.Blue;
        } else {
            // Fall back to the default alliance (Blue) and report the missing game-specific
            // message.
            DriverStation.reportError(
                "HubState: game specific message is null or empty; defaulting active hub alliance to BLUE.",
                false);
            firstActiveAlliance = Alliance.Blue;
        }
    }

    /**
     * Find the next time that the hub will change based on the current match time
     *
     * @param matchTime the current match time in seconds
     * @return the next hub change time, or 0.0 if no hub changes are scheduled
     */
    private double nextSwitchTime(double matchTime)
    {
        for (int i = 0; i < HUB_CHANGE_TIMES.length; i++) {
            if (matchTime > HUB_CHANGE_TIMES[i]) {
                return HUB_CHANGE_TIMES[i];
            }
        }
        return 0.0;
    }

    /**
     * Determines if the hub is close to changing. Uses {@link #SECONDS_BEFORE} as the threshold.
     */
    private boolean isCloseToSwitching()
    {
        double matchTime = DriverStation.getMatchTime();
        if (matchTime > HUB_CHANGE_TIMES[4]) {
            return matchTime - nextSwitchTime(matchTime) <= SECONDS_BEFORE;
        } else {
            // We know the hub won't change again in the last 30 seconds of the match
            return false;
        }
    }

    /**
     * Meant to be called regularly. Checks which hub is active and stores it in
     * {@link #activeAlliance}
     */
    public void periodic()
    {
        double matchTime = DriverStation.getMatchTime();
        if (matchTime <= HUB_CHANGE_TIMES[4] || matchTime > HUB_CHANGE_TIMES[0]) {
            // If the game is in the transition phase at the start of teleop (t>130),
            // or endgame (t<30), then our alliance hub is active, so set that
            // In auto, this method is not called.
            activeAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        } else if ((matchTime > HUB_CHANGE_TIMES[1]) && (matchTime <= HUB_CHANGE_TIMES[0])) {
            // First Shift
            activeAlliance = firstActiveAlliance;
        } else if (matchTime > HUB_CHANGE_TIMES[2] && matchTime <= HUB_CHANGE_TIMES[1]) {
            // Second Shift - switch from first shift
            activeAlliance = (firstActiveAlliance == Alliance.Blue) ? Alliance.Red : Alliance.Blue;
        } else if (matchTime > HUB_CHANGE_TIMES[3] && matchTime <= HUB_CHANGE_TIMES[2]) {
            // Third Shift - same as first shift
            activeAlliance = firstActiveAlliance;
        } else {
            // Fourth Shift - switch from third shift
            activeAlliance = (firstActiveAlliance == Alliance.Blue) ? Alliance.Red : Alliance.Blue;
        }

        hubActive.getAsBoolean();
        Logger.recordOutput(
            "Hub/Time Until Next Phase",
            matchTime - nextSwitchTime(matchTime));
    }

    /**
     * Returns whether our alliance hub is currently active
     */
    private boolean isOurHubActive()
    {
        return activeAlliance == DriverStation.getAlliance().orElse(Alliance.Blue);
    }
}
