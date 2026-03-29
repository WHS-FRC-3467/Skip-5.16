package frc.robot.commands.autos.tuning;

import frc.robot.commands.DriveCommands;
import frc.robot.commands.autos.utils.AutoContext;
import frc.robot.commands.autos.utils.AutoOption;
import frc.robot.commands.autos.utils.AutoUtil;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
public final class WheelCharacterizationAuto {
    public static AutoOption create(AutoContext ctx) {
        return AutoUtil.commandOption(() -> DriveCommands.wheelRadiusCharacterization(ctx.drive()));
    }
}
