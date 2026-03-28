package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * A helper class for logging various types of data related to FRC subsystems by Team 604 Quixilver
 */
public class LoggerHelper {
    // Cache of last recorded command name per subsystem to avoid redundant logger writes
    private static final Map<String, String> lastCommandBySubsystem = new ConcurrentHashMap<>();

    /**
     * Records the current command running on a subsystem to the logger.
     *
     * @param name The logging key prefix for this subsystem
     * @param subsystem The subsystem to log the current command for
     */
    public static void recordCurrentCommand(String name, SubsystemBase subsystem) {
        final var currentCommand = subsystem.getCurrentCommand();
        final String currentName = currentCommand == null ? "None" : currentCommand.getName();

        // Only write to the logger when the current command changes to reduce NT/log traffic
        String last = lastCommandBySubsystem.get(name);
        if (currentName.equals(last)) {
            return;
        }
        lastCommandBySubsystem.put(name, currentName);
        Logger.recordOutput(name + "/Current Command", currentName);
    }

    /**
     * Records a list of Pose2d objects to the logger with the specified key.
     *
     * @param key the key under which the list of Pose2d objects will be recorded
     * @param list the list of Pose2d objects to be recorded
     */
    public static void recordPose2dList(String key, List<Pose2d> list) {
        Pose2d[] array = new Pose2d[list.size()];
        Logger.recordOutput(key, list.toArray(array));
    }
}
