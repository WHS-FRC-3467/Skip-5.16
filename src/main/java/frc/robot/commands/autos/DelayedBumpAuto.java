/*
 * Copyright (C) 2026 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If
 * not, see <https://www.gnu.org/licenses/>.
 */
package frc.robot.commands.autos;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

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
public final class DelayedBumpAuto {
    /** Builds the safe or aggressive delayedBump autonomous routine. */
    public static Optional<AutoOption> create(AutoContext ctx, boolean isSafe) {
        List<String> names =
                List.of(
                        ChoreoTraj.DelayedBump1.name(),
                        isSafe
                                ? ChoreoTraj.DelayedBumpSafe2.name()
                                : ChoreoTraj.DelayedBump2.name());

        Optional<Trajectory<SwerveSample>> bumpTrajectory =
                AutoUtil.loadTrajectory(ChoreoTraj.BumpPath.name(), false);

        List<Trajectory<SwerveSample>> trajectories =
                AutoUtil.loadTrajectories(names, false).orElse(null);
        return Optional.of(
                AutoUtil.trajectoryOption(
                        trajectories,
                        () -> {
                            AutoRoutine routine =
                                    ctx.autoFactory()
                                            .newRoutine(
                                                    "DelayedBump"
                                                            + (isSafe ? "Safe" : "Aggressive"));
                            AutoTrajectory first = routine.trajectory(trajectories.get(0));
                            AutoTrajectory second = routine.trajectory(trajectories.get(1));
                            Optional<AutoTrajectory> bump = bumpTrajectory.map(routine::trajectory);
                            AutoUtil.bindEvents(ctx, first, second);
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
                                                    ctx, first, bump, 3.0, second));

                            second.done().onTrue(AutoCommands.shootThenFollow(ctx, 10.0, second));
                            AutoCommands.retryTrigger(routine, second)
                                    .onTrue(
                                            AutoCommands.recoverThenFollow(
                                                    ctx, second, bump, 10.0, second));
                            return routine;
                        }));
    }
}
