package frc.robot.commands.autos;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
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

/** Native Choreo routine for the depot-side multi-piece autonomous variants. */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public final class DepotShootAuto {
    private static final Alert TRAJECTORIES_MISSING =
            new Alert("Depot Auto Trajectories Missing, Auto(s) Unavailable", AlertType.kError);

    /** Builds the safe or aggressive depot autonomous routine. */
    public static Optional<AutoOption> create(AutoContext ctx, boolean isSafe) {
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
        List<Trajectory<SwerveSample>> trajectories =
                AutoUtil.loadTrajectories(names, false).orElse(null);
        if (trajectories == null) {
            TRAJECTORIES_MISSING.set(true);
            return Optional.empty();
        }

        Optional<Trajectory<SwerveSample>> tunnelTrajectory =
                AutoUtil.loadTrajectory(ChoreoTraj.TunnelPath.name(), false);

        return Optional.of(
                AutoUtil.trajectoryOption(
                        trajectories,
                        () -> {
                            AutoRoutine routine =
                                    ctx.autoFactory()
                                            .newRoutine("Depot" + (isSafe ? "Safe" : "Aggressive"));
                            AutoTrajectory first = routine.trajectory(trajectories.get(0));
                            AutoTrajectory second = routine.trajectory(trajectories.get(1));
                            AutoTrajectory third = routine.trajectory(trajectories.get(2));
                            AutoTrajectory fourth = routine.trajectory(trajectories.get(3));
                            Optional<AutoTrajectory> tunnel =
                                    tunnelTrajectory.map(routine::trajectory);
                            AutoUtil.bindEvents(ctx, first, second, third, fourth);
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

                            first.done().onTrue(shootThenFollow(ctx, 2.5, second));
                            AutoCommands.retryTrigger(routine, first)
                                    .onTrue(recoverThenFollow(ctx, first, tunnel, 2.5, second));

                            second.done().onTrue(shootThenFollow(ctx, 2.5, third));
                            AutoCommands.retryTrigger(routine, second)
                                    .onTrue(recoverThenFollow(ctx, second, tunnel, 2.5, third));

                            third.done().onTrue(shootThenFollow(ctx, 10.0, fourth));
                            AutoCommands.retryTrigger(routine, third)
                                    .onTrue(recoverThenFollow(ctx, third, tunnel, 10.0, fourth));

                            fourth.done().onTrue(shootThenFollow(ctx, 10.0, third));
                            AutoCommands.retryTrigger(routine, fourth)
                                    .onTrue(recoverThenFollow(ctx, fourth, tunnel, 10.0, third));

                            return routine;
                        }));
    }

    private static Command shootThenFollow(
            AutoContext ctx, double timeoutSeconds, AutoTrajectory next) {
        return Commands.sequence(
                shootOnly(ctx, timeoutSeconds),
                AutoCommands.stowHood(ctx.shooter()),
                retractIntake(ctx),
                next.spawnCmd());
    }

    private static Command recoverThenFollow(
            AutoContext ctx,
            AutoTrajectory failedTrajectory,
            Optional<AutoTrajectory> tunnel,
            double timeoutSeconds,
            AutoTrajectory next) {
        return Commands.sequence(
                AutoCommands.recoverTrajectory(
                        ctx.drive(), failedTrajectory, tunnel, retractIntake(ctx)),
                shootThenFollow(ctx, timeoutSeconds, next));
    }

    private static Command shootOnly(AutoContext ctx, double timeoutSeconds) {
        return AutoCommands.shootCommand(
                ctx.drive(),
                ctx.intake(),
                ctx.indexer(),
                ctx.tower(),
                ctx.shooter(),
                timeoutSeconds);
    }

    private static Command retractIntake(AutoContext ctx) {
        return ctx.intake().retractIntake().asProxy().withTimeout(0.5);
    }
}
