package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class Intake extends SubsystemBase implements AutoCloseable {

    private static final LoggedTunableNumber INTAKE_SETPOINT =
        new LoggedTunableNumber("Intake/IntakeRPS", 0.0);
    private static final LoggedTunableNumber EJECT_SETPOINT =
        new LoggedTunableNumber("Intake/EjectRPS", 0.0);

    private final FlywheelMechanism io;

    @RequiredArgsConstructor
    @SuppressWarnings("ImmutableEnumChecker")
    @Getter
    public enum Setpoint {
        INTAKE(() -> RotationsPerSecond.of(INTAKE_SETPOINT.get())),
        EJECT(() -> RotationsPerSecond.of(EJECT_SETPOINT.get())),
        STOP(() -> RotationsPerSecond.of(0.0));

        private final Supplier<AngularVelocity> setpoint;

    }

    /** Constructor for the Intake subsystem - accepts a FlywheelMechanism. */
    public Intake(FlywheelMechanism intakeIO) {
        this.io = intakeIO;
    }

    /** Run the intake. Static speed torque control. */
    public Command intake() {
        return Commands.runOnce(() -> io.runVelocity(Setpoint.INTAKE.getSetpoint().get(), IntakeConstants.MAX_ACCELERATION,
            PIDSlot.SLOT_0));
    }

    /** Eject the intake. Static speed torque control. */
    public Command eject() {
        return Commands.runOnce(() -> io.runVelocity(Setpoint.EJECT.getSetpoint().get(), IntakeConstants.MAX_ACCELERATION,
            PIDSlot.SLOT_0));
    }

    /** Stop the intake as fast as possible. */
    public Command stop() {
        return Commands.runOnce(() -> io.runVelocity(RotationsPerSecond.of(0), IntakeConstants.MAX_ACCELERATION,
            PIDSlot.SLOT_0));
    }

    /* Checks to see if the intake is near the setpoint */
    public boolean nearSetpoint(Setpoint setpoint) {
        return MathUtil.isNear(
            setpoint.getSetpoint().get().in(RotationsPerSecond),
            io.getVelocity().in(RotationsPerSecond),
            IntakeConstants.TOLERANCE.in(RotationsPerSecond));
    }

    @Override
    public void periodic() {
        io.periodic();
    }

    @Override
    public void close() {
        io.close();
    }
}
