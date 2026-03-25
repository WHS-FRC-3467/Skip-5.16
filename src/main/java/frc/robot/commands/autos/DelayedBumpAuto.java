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
public final class DelayedBumpAuto {
    /** Builds the safe or aggressive depot autonomous routine. */
    public static AutoOption create(AutoContext ctx) {
        List<String> names =
                List.of(ChoreoTraj.DelayedBump1.name(), ChoreoTraj.DelayedBump2.name());
        List<Trajectory<SwerveSample>> trajectories = AutoUtil.loadTrajectories(names, false);
        return AutoUtil.trajectoryOption(
                trajectories,
                () -> {
                    AutoRoutine routine = ctx.autoFactory().newRoutine("DelayedBump");
                    AutoTrajectory first = routine.trajectory(trajectories.get(0));
                    AutoTrajectory second = routine.trajectory(trajectories.get(1));
                    AutoUtil.bindEvents(ctx, first, second);
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
                                            Commands.repeatingSequence(
                                                            second.cmd(),
                                                            AutoCommands.shootCommand(
                                                                    ctx.drive(),
                                                                    ctx.intake(),
                                                                    ctx.indexer(),
                                                                    ctx.tower(),
                                                                    ctx.shooter(),
                                                                    2.5))
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
