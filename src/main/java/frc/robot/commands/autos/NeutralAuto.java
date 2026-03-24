package frc.robot.commands.autos;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.commands.autos.utils.AutoCommands;
import frc.robot.commands.autos.utils.AutoContext;
import frc.robot.commands.autos.utils.AutoOption;
import frc.robot.commands.autos.utils.AutoUtil;
import frc.robot.generated.ChoreoTraj;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;

import java.util.List;
import java.util.Set;

/** Native Choreo routine for the neutral-zone multi-piece autonomous variants. */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public final class NeutralAuto {
    /**
     * Builds the selected neutral auto variant.
     *
     * @param shouldMirror Whether to mirror the route for the opposite starting side
     * @param isSafe Whether to use the safer first segment instead of the aggressive one
     */
    public static AutoOption create(AutoContext ctx, boolean shouldMirror, boolean isSafe) {
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
                AutoUtil.loadTrajectories(names, shouldMirror);
        Trajectory<SwerveSample> tunnelTrajectory =
                AutoUtil.loadTrajectories(List.of(ChoreoTraj.TunnelPath.name()), shouldMirror)
                        .stream()
                        .findFirst()
                        .orElseThrow();

        return AutoUtil.trajectoryOption(
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
                    AutoTrajectory tunnel = routine.trajectory(tunnelTrajectory);
                    AutoUtil.bindEvents(ctx, first, second, third);
                    routine.active()
                            .onTrue(
                                    Commands.sequence(
                                            Commands.runOnce(
                                                    ctx.drive()::resetTrajectoryControllers),
                                            first.resetOdometry(),
                                            Commands.defer(
                                                    () ->
                                                            Commands.waitSeconds(
                                                                    AutoCommands.getAutoDelay()),
                                                    Set.of()),
                                            Commands.defer(
                                                    () ->
                                                            AutoCommands.safeFollowTrajectory(
                                                                    ctx.drive(),
                                                                    first,
                                                                    tunnel,
                                                                    ctx.intake().retractIntake()),
                                                    Set.of(ctx.drive())),
                                            AutoCommands.shootCommand(
                                                    ctx.drive(),
                                                    ctx.intake(),
                                                    ctx.indexer(),
                                                    ctx.tower(),
                                                    ctx.shooter(),
                                                    3.0),
                                            Commands.repeatingSequence(
                                                            AutoCommands.stowHood(ctx.shooter()),
                                                            ctx.intake()
                                                                    .retractIntake()
                                                                    .asProxy()
                                                                    .withTimeout(0.5),
                                                            Commands.defer(
                                                                    () ->
                                                                            AutoCommands
                                                                                    .safeFollowTrajectory(
                                                                                            ctx
                                                                                                    .drive(),
                                                                                            second,
                                                                                            tunnel,
                                                                                            ctx.intake()
                                                                                                    .retractIntake()),
                                                                    Set.of(ctx.drive())),
                                                            AutoCommands.shootCommand(
                                                                    ctx.drive(),
                                                                    ctx.intake(),
                                                                    ctx.indexer(),
                                                                    ctx.tower(),
                                                                    ctx.shooter(),
                                                                    10),
                                                            AutoCommands.stowHood(ctx.shooter()),
                                                            ctx.intake()
                                                                    .retractIntake()
                                                                    .asProxy()
                                                                    .withTimeout(0.5),
                                                            Commands.defer(
                                                                    () ->
                                                                            AutoCommands
                                                                                    .safeFollowTrajectory(
                                                                                            ctx
                                                                                                    .drive(),
                                                                                            third,
                                                                                            tunnel,
                                                                                            ctx.intake()
                                                                                                    .retractIntake()),
                                                                    Set.of(ctx.drive())),
                                                            AutoCommands.shootCommand(
                                                                    ctx.drive(),
                                                                    ctx.intake(),
                                                                    ctx.indexer(),
                                                                    ctx.tower(),
                                                                    ctx.shooter(),
                                                                    10))
                                                    .finallyDo(
                                                            () ->
                                                                    CommandScheduler.getInstance()
                                                                            .schedule(
                                                                                    ctx.intake()
                                                                                            .stopRoller()
                                                                                            .ignoringDisable(
                                                                                                    true)))));
                    return routine;
                });
    }
}
