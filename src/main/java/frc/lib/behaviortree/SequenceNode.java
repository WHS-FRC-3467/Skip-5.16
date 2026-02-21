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

import java.util.Arrays;
import java.util.List;

/**
 * A composite node that executes its children one by one in order.
 *
 * <p>The sequence acts like a logical AND: <b>all children must succeed</b> for the sequence to
 * succeed. Think of it as a checklist—if any item fails, the whole checklist fails.
 *
 * <p>Behavior:
 *
 * <ul>
 *   <li>Ticks children left to right in order
 *   <li>If a child returns {@link NodeStatus#FAILURE}: stops immediately and returns FAILURE
 *   <li>If a child returns {@link NodeStatus#RUNNING}: stops and returns RUNNING (resumes next
 *       tick)
 *   <li>If ALL children return {@link NodeStatus#SUCCESS}: returns SUCCESS
 * </ul>
 *
 * <p>Example: "Shoot if shooter is ready AND ball is staged AND robot is aimed"
 *
 * <pre>{@code
 * new SequenceNode("ShootWhenReady",
 *     new ConditionNode("BallStaged", tower.isStaged),
 *     new ConditionNode("ShooterReady", shooter.readyToShoot),
 *     new ConditionNode("RobotAimed", robotState.facingTarget),
 *     new CommandActionNode("Shoot", Commands.parallel(indexer.shoot(), tower.shoot()))
 * )
 * }</pre>
 */
public class SequenceNode extends BehaviorTreeNode {

    private final List<BehaviorTreeNode> children;

    // Index of the child currently being ticked (persists across ticks for RUNNING children)
    private int currentIndex = 0;

    /**
     * Creates a sequence node.
     *
     * @param name display name for logging and visualization
     * @param children child nodes to execute in order
     */
    public SequenceNode(String name, BehaviorTreeNode... children) {
        super(name);
        this.children = Arrays.asList(children);
    }

    @Override
    protected NodeStatus execute() {
        while (currentIndex < children.size()) {
            NodeStatus status = children.get(currentIndex).tick();

            if (status == NodeStatus.FAILURE) {
                // Any child failure = sequence failure; reset for next tick
                currentIndex = 0;
                return NodeStatus.FAILURE;
            } else if (status == NodeStatus.RUNNING) {
                // Child is still working; pause here and resume next tick
                return NodeStatus.RUNNING;
            } else {
                // SUCCESS: child succeeded; move to the next one
                currentIndex++;
            }
        }

        // All children succeeded
        currentIndex = 0;
        return NodeStatus.SUCCESS;
    }

    @Override
    public void reset() {
        currentIndex = 0;
        super.reset();
    }

    @Override
    public List<BehaviorTreeNode> getChildren() {
        return children;
    }

    @Override
    public String getType() {
        return "Sequence";
    }
}
