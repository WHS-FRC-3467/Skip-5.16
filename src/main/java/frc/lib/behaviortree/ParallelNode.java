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
 * A composite node that ticks ALL children on every tick, regardless of their results.
 *
 * <p>Unlike {@link SequenceNode} and {@link SelectorNode} which stop early, the parallel node
 * always runs every child. The overall result depends on the chosen {@link Policy}.
 *
 * <p>Useful for running multiple concurrent behaviors—like driving and shooting at the same time.
 *
 * <p>Example: "Aim drivetrain AND spin up flywheel at the same time"
 *
 * <pre>{@code
 * new ParallelNode("PrepareShot", ParallelNode.Policy.REQUIRE_ALL,
 *     new CommandActionNode("AimDrive", DriveCommands.staticAimTowardsTarget(drive)),
 *     new CommandActionNode("SpinFlywheel", shooter.spinUpShooter())
 * )
 * }</pre>
 */
public class ParallelNode extends BehaviorTreeNode {

    /**
     * Policy that determines when the parallel node succeeds or fails.
     *
     * <p>Choose based on what you need:
     *
     * <ul>
     *   <li>{@link #REQUIRE_ALL}: like an AND gate—all children must succeed
     *   <li>{@link #REQUIRE_ONE}: like an OR gate—any child succeeding is enough
     * </ul>
     */
    public enum Policy {
        /**
         * Succeeds only when ALL children succeed. Returns FAILURE immediately if any child fails.
         * Returns RUNNING while children are still running (and none have failed).
         */
        REQUIRE_ALL,

        /**
         * Succeeds as soon as ANY child succeeds. Returns FAILURE only when all children fail.
         * Returns RUNNING while no child has succeeded yet (and not all have failed).
         */
        REQUIRE_ONE
    }

    private final List<BehaviorTreeNode> children;
    private final Policy policy;

    /**
     * Creates a parallel node.
     *
     * @param name display name for logging and visualization
     * @param policy determines how child results combine into the overall result
     * @param children child nodes to tick simultaneously each loop
     */
    public ParallelNode(String name, Policy policy, BehaviorTreeNode... children) {
        super(name);
        this.policy = policy;
        this.children = Arrays.asList(children);
    }

    @Override
    protected NodeStatus execute() {
        int successCount = 0;
        int failureCount = 0;

        // Tick every child regardless of result
        for (BehaviorTreeNode child : children) {
            NodeStatus status = child.tick();
            if (status == NodeStatus.SUCCESS) {
                successCount++;
            } else if (status == NodeStatus.FAILURE) {
                failureCount++;
            }
        }

        if (policy == Policy.REQUIRE_ALL) {
            if (failureCount > 0) return NodeStatus.FAILURE;
            if (successCount == children.size()) return NodeStatus.SUCCESS;
        } else { // REQUIRE_ONE
            if (successCount > 0) return NodeStatus.SUCCESS;
            if (failureCount == children.size()) return NodeStatus.FAILURE;
        }

        return NodeStatus.RUNNING;
    }

    @Override
    public List<BehaviorTreeNode> getChildren() {
        return children;
    }

    @Override
    public String getType() {
        return "Parallel";
    }
}
