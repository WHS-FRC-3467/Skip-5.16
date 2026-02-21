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

import java.util.function.Supplier;

/**
 * A leaf node that performs an action by calling a {@link Supplier}{@code <NodeStatus>}.
 *
 * <p>Use this for simple, non-command actions where you control the logic directly. The supplier
 * runs every tick and returns the current status.
 *
 * <p>For actions that wrap a WPILib Command, use {@link CommandActionNode} instead.
 *
 * <p>Example:
 *
 * <pre>{@code
 * // Simple instant action that always succeeds
 * new ActionNode("LogMessage", () -> {
 *     System.out.println("Behavior tree reached this node!");
 *     return NodeStatus.SUCCESS;
 * });
 *
 * // Action that runs for a while
 * new ActionNode("WaitForCondition", () -> {
 *     if (someCondition.getAsBoolean()) return NodeStatus.SUCCESS;
 *     doSomethingEachTick();
 *     return NodeStatus.RUNNING;
 * });
 * }</pre>
 */
public class ActionNode extends BehaviorTreeNode {

    private final Supplier<NodeStatus> action;

    /**
     * Creates an action node.
     *
     * @param name display name for logging and visualization
     * @param action a supplier that executes the action and returns the current status
     */
    public ActionNode(String name, Supplier<NodeStatus> action) {
        super(name);
        this.action = action;
    }

    @Override
    protected NodeStatus execute() {
        return action.get();
    }

    @Override
    public String getType() {
        return "Action";
    }
}
