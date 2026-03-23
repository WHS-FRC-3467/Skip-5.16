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

/** Native Choreo routine for the depot-side multi-piece autonomous variants. */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public final class DepotShootAuto {
    /** Builds the safe or aggressive depot autonomous routine. */
    public static AutoOption create(AutoContext ctx, boolean isSafe) {
        List<String> names =
                isSafe
                        ? List.of(
                                ChoreoTraj.NeutralSafe1.name(),
                                ChoreoTraj.Depot1.name(),
                                ChoreoTraj.NeutralSafe2.name(),
                                ChoreoTraj.Neutral2.name())
                        : List.of(
                                ChoreoTraj.Neutral1.name(),
                                ChoreoTraj.Depot1.name(),
                                ChoreoTraj.NeutralSafe2.name(),
                                ChoreoTraj.Neutral2.name());
        List<Trajectory<SwerveSample>> trajectories = AutoUtil.loadTrajectories(names, false);
        return AutoUtil.trajectoryOption(
                trajectories,
                () -> {
                    AutoRoutine routine =
                            ctx.autoFactory()
                                    .newRoutine("Depot" + (isSafe ? "Safe" : "Aggressive"));
                    AutoTrajectory first = routine.trajectory(trajectories.get(0));
                    AutoTrajectory second = routine.trajectory(trajectories.get(1));
                    AutoTrajectory third = routine.trajectory(trajectories.get(2));
                    AutoTrajectory fourth = routine.trajectory(trajectories.get(3));
                    AutoUtil.bindEvents(ctx, first, second, third, fourth);
                    routine.active()
                            .onTrue(
                                    Commands.sequence(
                                            Commands.runOnce(
                                                    ctx.drive()::resetTrajectoryControllers),
                                            first.resetOdometry(),
                                            first.cmd(),
                                            AutoCommands.shootCommand(
                                                    ctx.drive(),
                                                    ctx.intake(),
                                                    ctx.indexer(),
                                                    ctx.tower(),
                                                    ctx.shooter(),
                                                    2.5),
                                            second.cmd(),
                                            AutoCommands.shootCommand(
                                                    ctx.drive(),
                                                    ctx.intake(),
                                                    ctx.indexer(),
                                                    ctx.tower(),
                                                    ctx.shooter(),
                                                    2.5),
                                            AutoCommands.stowHood(ctx.shooter()),
                                            Commands.repeatingSequence(
                                                            AutoCommands.stowHood(ctx.shooter()),
                                                            ctx.intake()
                                                                    .retractIntake()
                                                                    .asProxy()
                                                                    .withTimeout(0.5),
                                                            third.cmd(),
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
                                                            fourth.cmd(),
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
