package frc.robot.commands.autos;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.commands.autos.utils.AutoCommands;
import frc.robot.commands.autos.utils.AutoContext;
import frc.robot.commands.autos.utils.AutoOption;
import frc.robot.commands.autos.utils.AutoUtil;
import frc.robot.generated.ChoreoTraj;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;

import java.util.List;
import java.util.Optional;
import java.util.Set;

/** Native Choreo routine for the neutral-zone multi-piece autonomous variants. */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public final class NeutralAuto {
    private static final Alert TRAJECTORIES_MISSING =
            new Alert("Neutral Auto Trajectories Missing, Auto(s) Unavailable", AlertType.kError);

    /**
     * Builds the selected neutral auto variant.
     *
     * @param shouldMirror Whether to mirror the route for the opposite starting side
     * @param isSafe Whether to use the safer first segment instead of the aggressive one
     */
    public static Optional<AutoOption> create(
            AutoContext ctx, boolean shouldMirror, boolean isSafe) {
        List<String> names =
                isSafe
                        ? List.of(
                                ChoreoTraj.NeutralSafe1.name(),
                                ChoreoTraj.NeutralSafe2.name(),
                                ChoreoTraj.Neutral2.name())
                        : List.of(
                                ChoreoTraj.Neutral1.name(),
                                ChoreoTraj.NeutralSafe2.name(),
                                ChoreoTraj.Neutral2.name());
        List<Trajectory<SwerveSample>> trajectories =
                AutoUtil.loadTrajectories(names, shouldMirror).orElse(null);
        if (trajectories == null) {
            TRAJECTORIES_MISSING.set(true);
            return Optional.empty();
        }

        Optional<Trajectory<SwerveSample>> tunnelTrajectory =
                AutoUtil.loadTrajectory(ChoreoTraj.TunnelPath.name(), shouldMirror);

        return Optional.of(
                AutoUtil.trajectoryOption(
                        trajectories,
                        () -> {
                            AutoRoutine routine =
                                    ctx.autoFactory()
                                            .newRoutine(
                                                    "Neutral"
                                                            + (isSafe ? "Safe" : "Aggressive")
                                                            + (shouldMirror ? "Right" : "Left"));
                            AutoTrajectory first = routine.trajectory(trajectories.get(0));
                            AutoTrajectory second = routine.trajectory(trajectories.get(1));
                            AutoTrajectory third = routine.trajectory(trajectories.get(2));
                            Optional<AutoTrajectory> tunnel =
                                    tunnelTrajectory.map(routine::trajectory);
                            AutoUtil.bindEvents(ctx, first, second, third);
                            routine.active()
                                    .onTrue(
                                            Commands.sequence(
                                                    Commands.runOnce(
                                                            ctx.drive()
                                                                    ::resetTrajectoryControllers),
                                                    first.resetOdometry(),
                                                    Commands.defer(
                                                            () ->
                                                                    Commands.waitSeconds(
                                                                            AutoCommands
                                                                                    .getAutoDelay()),
                                                            Set.of()),
                                                    first.spawnCmd()));

                            first.done().onTrue(AutoCommands.shootThenFollow(ctx, 3.0, second));
                            AutoCommands.retryTrigger(routine, first)
                                    .onTrue(
                                            AutoCommands.recoverThenFollow(
                                                    ctx, first, tunnel, 3.0, second));

                            second.done().onTrue(AutoCommands.shootThenFollow(ctx, 10.0, third));
                            AutoCommands.retryTrigger(routine, second)
                                    .onTrue(
                                            AutoCommands.recoverThenFollow(
                                                    ctx, second, tunnel, 10.0, third));

                            third.done().onTrue(AutoCommands.shootThenFollow(ctx, 10.0, second));
                            AutoCommands.retryTrigger(routine, third)
                                    .onTrue(
                                            AutoCommands.recoverThenFollow(
                                                    ctx, third, tunnel, 10.0, second));

                            return routine;
                        }));
    }
}
