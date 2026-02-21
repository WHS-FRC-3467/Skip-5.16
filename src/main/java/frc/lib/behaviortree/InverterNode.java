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

import java.util.List;

/**
 * A decorator node that flips the result of its child.
 *
 * <p>Useful for expressing negative conditions—"do this ONLY if something is NOT true."
 *
 * <p>Behavior:
 *
 * <ul>
 *   <li>Child returns {@link NodeStatus#SUCCESS} → Inverter returns {@link NodeStatus#FAILURE}
 *   <li>Child returns {@link NodeStatus#FAILURE} → Inverter returns {@link NodeStatus#SUCCESS}
 *   <li>Child returns {@link NodeStatus#RUNNING} → Inverter returns {@link NodeStatus#RUNNING}
 *       (RUNNING is not inverted)
 * </ul>
 *
 * <p>Example: "Check that the robot does NOT have a game piece"
 *
 * <pre>{@code
 * // "No game piece staged" condition
 * new InverterNode("NoBallStaged", new ConditionNode("BallStaged", tower.isStaged))
 * }</pre>
 */
public class InverterNode extends BehaviorTreeNode {

    private final BehaviorTreeNode child;

    /**
     * Creates an inverter decorator.
     *
     * @param name display name for logging and visualization
     * @param child the child node whose result will be inverted
     */
    public InverterNode(String name, BehaviorTreeNode child) {
        super(name);
        this.child = child;
    }

    @Override
    protected NodeStatus execute() {
        NodeStatus status = child.tick();
        switch (status) {
            case SUCCESS:
                return NodeStatus.FAILURE;
            case FAILURE:
                return NodeStatus.SUCCESS;
            default:
                return NodeStatus.RUNNING;
        }
    }

    @Override
    public List<BehaviorTreeNode> getChildren() {
        return List.of(child);
    }

    @Override
    public String getType() {
        return "Inverter";
    }
}
