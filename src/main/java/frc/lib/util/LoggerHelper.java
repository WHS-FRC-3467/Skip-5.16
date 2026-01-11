package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * A helper class for logging various types of data related to FRC subsystems by Team 604 Quixilver
 */
public class LoggerHelper {
    public static void recordCurrentCommand(String name, SubsystemBase subsystem)
    {
        final var currentCommand = subsystem.getCurrentCommand();
        Logger.recordOutput(
            name + "/Current Command",
            currentCommand == null ? "None" : currentCommand.getName());
    }

    /**
     * Records a list of Pose2d objects to the logger with the specified key.
     *
     * @param key the key under which the list of Pose2d objects will be recorded
     * @param list the list of Pose2d objects to be recorded
     */
    public static void recordPose2dList(String key, List<Pose2d> list)
    {
        Pose2d[] array = new Pose2d[list.size()];
        list.toArray(array);
        Logger.recordOutput(key, array);
    }
}
