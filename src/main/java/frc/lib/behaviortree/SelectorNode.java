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
 * A composite node that tries its children in order until one succeeds.
 *
 * <p>The selector acts like a logical OR (also called a "Fallback" node): <b>any one child
 * succeeding</b> is enough for the selector to succeed. Think of it as a priority list of
 * alternatives—try the first option, and if it fails, fall back to the next.
 *
 * <p>Behavior:
 *
 * <ul>
 *   <li>Ticks children left to right in order
 *   <li>If a child returns {@link NodeStatus#SUCCESS}: stops immediately and returns SUCCESS
 *   <li>If a child returns {@link NodeStatus#RUNNING}: stops and returns RUNNING (resumes next
 *       tick)
 *   <li>If ALL children return {@link NodeStatus#FAILURE}: returns FAILURE
 * </ul>
 *
 * <p>Example: "Shoot if ready, otherwise intake if ball available, otherwise idle"
 *
 * <pre>{@code
 * new SelectorNode("AutoDecision",
 *     new SequenceNode("ShootIfReady",
 *         new ConditionNode("BallStaged", tower.isStaged),
 *         new ConditionNode("ShooterReady", shooter.readyToShoot),
 *         new CommandActionNode("Shoot", ...)
 *     ),
 *     new SequenceNode("IntakeIfAvailable",
 *         new ConditionNode("NoGamePiece", new InverterNode("Not", tower.isStaged)),
 *         new CommandActionNode("Intake", intake.extendIntake())
 *     ),
 *     new CommandActionNode("Idle", Commands.none())
 * )
 * }</pre>
 */
public class SelectorNode extends BehaviorTreeNode {

    private final List<BehaviorTreeNode> children;

    // Index of the child currently being ticked (persists across ticks for RUNNING children)
    private int currentIndex = 0;

    /**
     * Creates a selector (fallback) node.
     *
     * @param name display name for logging and visualization
     * @param children child nodes to try in priority order (highest priority first)
     */
    public SelectorNode(String name, BehaviorTreeNode... children) {
        super(name);
        this.children = Arrays.asList(children);
    }

    @Override
    protected NodeStatus execute() {
        while (currentIndex < children.size()) {
            NodeStatus status = children.get(currentIndex).tick();

            if (status == NodeStatus.SUCCESS) {
                // Any child success = selector success; reset for next tick
                currentIndex = 0;
                return NodeStatus.SUCCESS;
            } else if (status == NodeStatus.RUNNING) {
                // Child is still working; pause here and resume next tick
                return NodeStatus.RUNNING;
            } else {
                // FAILURE: this child failed; try the next one
                currentIndex++;
            }
        }

        // All children failed
        currentIndex = 0;
        return NodeStatus.FAILURE;
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
        return "Selector";
    }
}
