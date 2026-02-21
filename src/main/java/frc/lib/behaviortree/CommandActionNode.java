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

package frc.lib.behaviortree;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

/**
 * A leaf node that wraps a WPILib {@link Command} as a behavior tree action.
 *
 * <p>This is the primary way to integrate the behavior tree with the WPILib command system. The
 * node manages the command's full lifecycle: initialize → execute → end.
 *
 * <p>Behavior:
 *
 * <ul>
 *   <li>First tick: calls {@link Command#initialize()} and begins execution
 *   <li>While running: calls {@link Command#execute()} each tick and returns {@link
 *       NodeStatus#RUNNING}
 *   <li>When the command finishes ({@link Command#isFinished()} returns true): calls {@link
 *       Command#end(boolean)} and returns {@link NodeStatus#SUCCESS}
 *   <li>When {@link #reset()} is called: ends the command with {@code interrupted = true}
 * </ul>
 *
 * <p><b>Important:</b> Commands running inside the behavior tree are NOT scheduled through the
 * {@link edu.wpi.first.wpilibj2.command.CommandScheduler}. Subsystem requirement conflicts are not
 * automatically enforced. For safe usage, run the entire {@link BehaviorTree} as the root command
 * of a subsystem or use commands that don't have conflicting requirements.
 *
 * <p>Use the {@link Supplier} constructor to create a fresh command instance each time the node
 * starts (recommended for commands that hold state).
 *
 * <p>Example:
 *
 * <pre>{@code
 * // Using a command factory (recommended - creates a fresh command each activation)
 * new CommandActionNode("Shoot", () -> Commands.parallel(indexer.shoot(), tower.shoot()))
 *
 * // Using a pre-built command (reused each activation)
 * new CommandActionNode("Spin Up Shooter", shooter.spinUpShooter())
 * }</pre>
 */
public class CommandActionNode extends BehaviorTreeNode {

    private final Supplier<Command> commandFactory;
    private Command activeCommand = null;

    /**
     * Creates a command action node with a factory that creates a fresh command instance each time
     * the node is activated. This is the recommended approach for stateful commands.
     *
     * @param name display name for logging and visualization
     * @param commandFactory supplier that creates a new command each time this node starts
     */
    public CommandActionNode(String name, Supplier<Command> commandFactory) {
        super(name);
        this.commandFactory = commandFactory;
    }

    /**
     * Creates a command action node that reuses the same command instance each activation.
     *
     * <p>Use this for simple commands that don't hold state between runs. For stateful commands,
     * prefer the {@link Supplier} constructor.
     *
     * @param name display name for logging and visualization
     * @param command the command to run when this node is ticked
     */
    public CommandActionNode(String name, Command command) {
        super(name);
        this.commandFactory = () -> command;
    }

    @Override
    protected NodeStatus execute() {
        // Initialize the command on the first tick
        if (activeCommand == null) {
            activeCommand = commandFactory.get();
            activeCommand.initialize();
        }

        // Execute the command this tick
        activeCommand.execute();

        // Check if the command has finished
        if (activeCommand.isFinished()) {
            activeCommand.end(false);
            activeCommand = null;
            return NodeStatus.SUCCESS;
        }

        return NodeStatus.RUNNING;
    }

    /**
     * Resets this node by ending any active command with {@code interrupted = true}.
     *
     * <p>Called automatically when {@link BehaviorTree#reset()} is invoked.
     */
    @Override
    public void reset() {
        if (activeCommand != null) {
            activeCommand.end(true);
            activeCommand = null;
        }
        super.reset();
    }

    @Override
    public String getType() {
        return "Command";
    }
}
