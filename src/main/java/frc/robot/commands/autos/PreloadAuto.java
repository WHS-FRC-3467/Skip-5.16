package frc.robot.commands.autos;

import frc.robot.commands.autos.utils.AutoCommands;
import frc.robot.commands.autos.utils.AutoContext;
import frc.robot.commands.autos.utils.AutoOption;
import frc.robot.commands.autos.utils.AutoUtil;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
public final class PreloadAuto {
    public static AutoOption create(AutoContext ctx) {
        return AutoUtil.commandOption(
                () ->
                        AutoCommands.hubShootCommand(
                                ctx.drive(),
                                ctx.intake(),
                                ctx.indexer(),
                                ctx.tower(),
                                ctx.shooter(),
                                10.0));
    }
}
