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
 * A decorator node that repeats its child a specified number of times.
 *
 * <p>Useful for retrying a behavior multiple times or running an action a fixed number of loops.
 *
 * <p>Behavior:
 *
 * <ul>
 *   <li>Ticks the child repeatedly until it succeeds {@code maxCount} times
 *   <li>If the child returns {@link NodeStatus#RUNNING}: returns RUNNING (waits for completion)
 *   <li>If the child returns {@link NodeStatus#SUCCESS}: increments count, re-initializes child
 *   <li>If the child returns {@link NodeStatus#FAILURE}: returns FAILURE immediately
 *   <li>After succeeding {@code maxCount} times: returns SUCCESS and resets
 * </ul>
 *
 * <p>Example: "Intake a game piece 3 times before moving on"
 *
 * <pre>{@code
 * new RepeatNode("IntakeThreeTimes", 3,
 *     new CommandActionNode("Intake", intake.extendIntake())
 * )
 * }</pre>
 */
public class RepeatNode extends BehaviorTreeNode {

    private final BehaviorTreeNode child;
    private final int maxCount;

    // How many times the child has succeeded so far in this repetition cycle
    private int completedCount = 0;

    /**
     * Creates a repeat decorator.
     *
     * @param name display name for logging and visualization
     * @param maxCount number of times to repeat the child before returning SUCCESS
     * @param child the child node to repeat
     */
    public RepeatNode(String name, int maxCount, BehaviorTreeNode child) {
        super(name);
        this.maxCount = maxCount;
        this.child = child;
    }

    @Override
    protected NodeStatus execute() {
        NodeStatus status = child.tick();

        switch (status) {
            case FAILURE:
                // Child failed—stop repeating and propagate failure
                completedCount = 0;
                return NodeStatus.FAILURE;

            case SUCCESS:
                completedCount++;
                if (completedCount >= maxCount) {
                    // Done all repetitions!
                    completedCount = 0;
                    return NodeStatus.SUCCESS;
                }
                // Reset child so it can run again
                child.reset();
                return NodeStatus.RUNNING;

            default:
                // RUNNING—child is still working
                return NodeStatus.RUNNING;
        }
    }

    @Override
    public void reset() {
        completedCount = 0;
        super.reset();
    }

    @Override
    public List<BehaviorTreeNode> getChildren() {
        return List.of(child);
    }

    @Override
    public String getType() {
        return "Repeat";
    }
}
