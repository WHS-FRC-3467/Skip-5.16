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

import lombok.AccessLevel;
import lombok.NoArgsConstructor;

import java.util.List;
import java.util.Set;

/** Native Choreo routine for the Anti-Nashoba autonomous variant. */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public final class AntiNashobaAuto {
    /** Builds the left or right mirrored Anti-Nashoba autonomous routine. */
    public static AutoOption create(AutoContext ctx, boolean shouldMirror) {
        List<Trajectory<SwerveSample>> trajectories =
                AutoUtil.loadTrajectories(List.of("Neutral1Nashoba", "NeutralSafe2"), shouldMirror);
        return AutoUtil.trajectoryOption(
                trajectories,
                () -> {
                    AutoRoutine routine =
                            ctx.autoFactory()
                                    .newRoutine("AntiNashoba" + (shouldMirror ? "Right" : "Left"));
                    AutoTrajectory first = routine.trajectory(trajectories.get(0));
                    AutoTrajectory second = routine.trajectory(trajectories.get(1));
                    AutoUtil.bindEvents(ctx, first, second);
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
                                            first.cmd(),
                                            AutoCommands.shootCommand(
                                                    ctx.drive(),
                                                    ctx.intake(),
                                                    ctx.indexer(),
                                                    ctx.tower(),
                                                    ctx.shooter(),
                                                    2.5),
                                            AutoCommands.stowHood(ctx.shooter()),
                                            ctx.intake().retractIntake().asProxy().withTimeout(0.5),
                                            second.cmd(),
                                            AutoCommands.shootCommand(
                                                            ctx.drive(),
                                                            ctx.intake(),
                                                            ctx.indexer(),
                                                            ctx.tower(),
                                                            ctx.shooter(),
                                                            10)
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
