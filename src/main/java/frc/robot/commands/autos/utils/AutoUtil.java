package frc.robot.commands.autos.utils;

import static edu.wpi.first.units.Units.Meters;

import choreo.Choreo;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.FieldConstants;
import frc.robot.generated.ChoreoVars;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Shared helpers for native Choreo autonomous routines.
 *
 * <p>This utility handles the repetitive plumbing around wrapping chooser options, loading Choreo
 * trajectories, mirroring them for the alternate side, and binding common event markers.
 */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public final class AutoUtil {
    /** Wraps a simple command-based auto in an {@link AutoOption} with no preview metadata. */
    public static AutoOption commandOption(Supplier<Command> commandSupplier) {
        return new AutoOption(commandSupplier, List.of(), new Pose2d());
    }

    /**
     * Wraps a trajectory-based Choreo routine in an {@link AutoOption}.
     *
     * <p>The preview poses are flattened here so the chooser can render the full route on the field
     * widget without reinstantiating the routine.
     */
    public static AutoOption trajectoryOption(
            List<Trajectory<SwerveSample>> trajectories, Supplier<AutoRoutine> supplier) {
        if (trajectories.isEmpty()) {
            return commandOption(Commands::none);
        }

        List<Pose2d> previewPoses = new ArrayList<>();
        for (Trajectory<SwerveSample> trajectory : trajectories) {
            previewPoses.addAll(List.of(trajectory.getPoses()));
        }
        Pose2d start =
                trajectories.isEmpty()
                        ? new Pose2d()
                        : trajectories.get(0).getInitialPose(false).orElse(new Pose2d());
        return new AutoOption(
                () -> {
                    AutoRoutine routine = supplier.get();
                    return routine.cmd().finallyDo(routine::kill);
                },
                List.copyOf(previewPoses),
                start);
    }

    /**
     * Loads a group of named Choreo trajectories, mirroring them across the field width when the
     * routine needs the opposite lane.
     */
    public static Optional<List<Trajectory<SwerveSample>>> loadTrajectories(
            List<String> names, boolean shouldMirror) {
        var optionalTrajectories =
                names.stream().map(name -> loadTrajectory(name, shouldMirror)).toList();
        if (optionalTrajectories.stream().anyMatch(Optional::isEmpty)) {
            return Optional.empty();
        }

        return Optional.of(optionalTrajectories.stream().map(Optional::get).toList());
    }

    /**
     * Binds the shared event markers used by the current Choreo trajectories.
     *
     * <p>These markers live in the trajectory files themselves. Binding them here keeps the
     * handling local to each routine instead of registering them globally on the factory.
     */
    public static void bindEvents(AutoContext ctx, AutoTrajectory... trajectories) {
        double distanceFromHubMeters =
                ChoreoVars.Poses.NeutralShoot.getTranslation()
                        .minus(FieldConstants.Hub.INNER_CENTER_POINT.toTranslation2d())
                        .getNorm();

        for (AutoTrajectory trajectory : trajectories) {
            trajectory.atTime("ExtendIntake").onTrue(ctx.intake().intake());
            trajectory.atTime("RetractIntake").onTrue(ctx.intake().retractIntake());
            trajectory
                    .atTime("Spinup")
                    .onTrue(
                            ctx.shooter()
                                    .spinUpShooterToHubDistance(Meters.of(distanceFromHubMeters)));
        }
    }

    /** Loads a single Choreo trajectory and mirrors it when the caller requests it. */
    public static Optional<Trajectory<SwerveSample>> loadTrajectory(
            String name, boolean shouldMirror) {
        Trajectory<SwerveSample> trajectory =
                Choreo.<SwerveSample>loadTrajectory(name).orElse(null);
        if (trajectory == null) return Optional.empty();

        return Optional.of(shouldMirror ? mirror(trajectory) : trajectory);
    }

    /**
     * Mirrors a trajectory across the field width while preserving its event markers and splits.
     */
    private static Trajectory<SwerveSample> mirror(Trajectory<SwerveSample> trajectory) {
        return new Trajectory<>(
                trajectory.name(),
                trajectory.samples().stream().map(AutoUtil::mirrorSample).toList(),
                trajectory.splits(),
                trajectory.events());
    }

    /** Mirrors an individual sample across the field width for right-side autonomous variants. */
    private static SwerveSample mirrorSample(SwerveSample sample) {
        return new SwerveSample(
                sample.t,
                sample.x,
                FieldConstants.FIELD_WIDTH - sample.y,
                -sample.heading,
                sample.vx,
                -sample.vy,
                -sample.omega,
                sample.ax,
                -sample.ay,
                -sample.alpha,
                sample.moduleForcesX(),
                negate(sample.moduleForcesY()));
    }

    /** Returns a copy of the supplied array with all values negated. */
    private static double[] negate(double[] values) {
        double[] negated = new double[values.length];
        for (int i = 0; i < values.length; i++) {
            negated[i] = -values[i];
        }
        return negated;
    }
}
