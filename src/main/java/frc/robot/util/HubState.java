package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.LoggedTrigger;
import java.util.Arrays;
import lombok.AccessLevel;
import lombok.Getter;
import lombok.NoArgsConstructor;
import org.littletonrobotics.junction.Logger;

/**
 * Models the state of the game hub across match phases.
 *
 * <p>Updated for 2026 FRC timing behavior: - Robust against invalid match time (-1) - Parses FMS
 * message once - Cleaner shift calculation
 */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class HubState {

    /** Times (in seconds remaining) where the hub shifts alliances */
    private static final double[] HUB_CHANGE_TIMES = {130.0, 105.0, 80.0, 55.0, 30.0};

    private static final double SECONDS_BEFORE = 5.0;

    @Getter private static final HubState instance = new HubState();

    private volatile Alliance activeAlliance = Alliance.Blue;
    private volatile Alliance firstActiveAlliance = Alliance.Blue;

    private boolean firstAllianceInitialized = false;

    @Getter
    private final LoggedTrigger hubActive =
            new LoggedTrigger("Hub/Hub Active Trigger", this::isOurHubActive);

    @Getter
    private final LoggedTrigger hubChange =
            new LoggedTrigger(
                    "Hub/About To Change (" + SECONDS_BEFORE + "s)", this::isCloseToSwitching);

    public static double[] getHubChangeTimes() {
        return Arrays.copyOf(HUB_CHANGE_TIMES, HUB_CHANGE_TIMES.length);
    }

    /** Parse FMS message once per match */
    private void initializeFirstActiveAlliance() {
        if (firstAllianceInitialized) return;

        String message = DriverStation.getGameSpecificMessage();
        if (message != null && !message.isEmpty()) {
            firstActiveAlliance = message.charAt(0) == 'B' ? Alliance.Red : Alliance.Blue;
        } else {
            DriverStation.reportWarning(
                    "HubState: Missing game-specific message. Defaulting to BLUE.", false);
            firstActiveAlliance = Alliance.Blue;
        }

        firstAllianceInitialized = true;
    }

    private double nextSwitchTime(double matchTime) {
        for (double t : HUB_CHANGE_TIMES) {
            if (matchTime > t) {
                return t;
            }
        }
        return -1;
    }

    private boolean isCloseToSwitching() {
        double matchTime = DriverStation.getMatchTime();
        if (matchTime < 0) return false;

        double next = nextSwitchTime(matchTime);
        if (next < 0) return false;

        return (matchTime - next) <= SECONDS_BEFORE;
    }

    public void periodic() {
        double matchTime = DriverStation.getMatchTime();

        if (matchTime < 0) {
            // DS not connected or disabled
            return;
        }

        Alliance ourAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        // Auto or Endgame → our hub active
        if (matchTime > HUB_CHANGE_TIMES[0]
                || matchTime <= HUB_CHANGE_TIMES[HUB_CHANGE_TIMES.length - 1]) {
            activeAlliance = ourAlliance;
        } else {
            initializeFirstActiveAlliance();

            // Determine which shift we are in
            int shiftIndex = 0;
            for (int i = 0; i < HUB_CHANGE_TIMES.length; i++) {
                if (matchTime <= HUB_CHANGE_TIMES[i]) {
                    shiftIndex = i;
                }
            }

            // Even shifts → firstActiveAlliance
            // Odd shifts → opposite
            boolean evenShift = shiftIndex % 2 == 0;
            activeAlliance = evenShift ? firstActiveAlliance : opposite(firstActiveAlliance);
        }

        SmartDashboard.putBoolean(
                "Can Shoot!", hubActive.getAsBoolean() || hubChange.getAsBoolean());

        double next = nextSwitchTime(matchTime);
        if (next > 0) {
            Logger.recordOutput("Hub/Time Until Next Phase", matchTime - next);
        }
    }

    private Alliance opposite(Alliance alliance) {
        return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
    }

    private boolean isOurHubActive() {
        return activeAlliance == DriverStation.getAlliance().orElse(Alliance.Blue);
    }
}
