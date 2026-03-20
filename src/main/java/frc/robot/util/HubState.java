package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.lib.util.LoggedTrigger;
import frc.robot.RobotState;

import lombok.AccessLevel;
import lombok.Getter;
import lombok.NoArgsConstructor;

import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

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

    /**
     * Estimated time (in seconds) it takes for the fuel to be scored in the hub once it is inside
     */
    private static final double TIME_TO_PROCESS = 2.0;

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

    @Getter
    private final LoggedTrigger isTimeToShoot =
            new LoggedTrigger("Hub/Time To Shoot", this::isTimeToShoot);

    public static double[] getHubChangeTimes() {
        return Arrays.copyOf(HUB_CHANGE_TIMES, HUB_CHANGE_TIMES.length);
    }

    /**
     * Distance from hub in meters -> time for projectile to reach hub, but not fully enter such
     * that it is scored
     */
    private static final InterpolatingDoubleTreeMap hubTrajectoryTimeMap =
            new InterpolatingDoubleTreeMap();

    // TODO: Test time values. Currently *2'ed the theoretical values to account for velocity drop
    static {
        hubTrajectoryTimeMap.put(1.03, 0.134 * 2);
        hubTrajectoryTimeMap.put(1.30, 0.134 * 2);
        hubTrajectoryTimeMap.put(1.72, 0.169 * 2);
        hubTrajectoryTimeMap.put(2.1, 0.202 * 2);
        hubTrajectoryTimeMap.put(3.05, 0.269 * 2);
        hubTrajectoryTimeMap.put(3.54, 0.3 * 2);
        hubTrajectoryTimeMap.put(4.6, 0.396 * 2);
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

    /**
     * Run this periodically. This method: initializes first active alliance, updates the active
     * alliance, and reports to Elastic and AScope.
     */
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

        // If no more hub switches, count down to end of match
        if (next < 0) {
            next = 0;
        }

        Logger.recordOutput("Hub/Time Until Next Phase", matchTime - next);
    }

    private Alliance opposite(Alliance alliance) {
        return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
    }

    private boolean isOurHubActive() {
        return activeAlliance == DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    /**
     * Calculates whether it is time to shoot based on robot's pose. For signaling or gating shots
     * during the appropriate pre or post firing. Automatically returns true if feeding.
     * Precondition: the robot is facing the target.
     *
     * @return whether the robot, if it were to shoot now, would score when it is our shift.
     */
    private boolean isTimeToShoot() {
        RobotState robotState = RobotState.getInstance();
        if (robotState.getTarget() != RobotState.Target.HUB) {
            return true;
        }
        // Calculate time to shoot based on robot's pose, and add time for the hub to process the
        // shot.
        // Will use velocity and distance (component directed towards the hub) to create theoretical
        // LUT values.
        double totalShotTime =
                hubTrajectoryTimeMap.get(robotState.getDistanceToTarget().in(Meters))
                        + TIME_TO_PROCESS;

        // Compare current time to time of next switch.
        double matchTime = DriverStation.getMatchTime();
        // Also need to figure out how long we have after our shift to score
        // don't gate buzzer beating shots until knowing this for sure.
        return isOurHubActive() || (matchTime - totalShotTime < nextSwitchTime(matchTime));
    }
}
