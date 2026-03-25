package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.commands.autos.utils.AutoOption;
import frc.robot.commands.autos.utils.AutoUtil;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
public final class NoneAuto {
    public static AutoOption create() {
        return AutoUtil.commandOption(Commands::none);
    }
}
