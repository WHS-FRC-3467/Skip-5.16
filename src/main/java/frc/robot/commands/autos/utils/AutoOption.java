package frc.robot.commands.autos.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;
import java.util.function.Supplier;

/**
 * Dashboard-facing autonomous option.
 *
 * <p>The chooser stores one of these instead of a raw command so the selected auto still exposes
 * preview poses and an expected start pose before the command is instantiated.
 */
public record AutoOption(
        Supplier<Command> commandSupplier, List<Pose2d> previewPoses, Pose2d startingPose) {
    /** Builds a fresh command for the selected autonomous option. */
    public Command command() {
        return commandSupplier.get();
    }
}
