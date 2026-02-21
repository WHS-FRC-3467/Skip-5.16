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

import java.util.ArrayList;
import java.util.List;

/**
 * Abstract base class for all behavior tree nodes.
 *
 * <p>Behavior trees are composed of three kinds of nodes:
 *
 * <ul>
 *   <li><b>Composite</b> nodes have children and control execution flow ({@link SequenceNode},
 *       {@link SelectorNode}, {@link ParallelNode})
 *   <li><b>Decorator</b> nodes wrap a single child and modify its behavior ({@link InverterNode},
 *       {@link RepeatNode})
 *   <li><b>Leaf</b> nodes perform actual work or check conditions ({@link ConditionNode}, {@link
 *       ActionNode}, {@link CommandActionNode})
 * </ul>
 *
 * <p>Each node is {@link #tick() ticked} once per robot loop. The {@link #tick()} method records
 * the status and delegates to the subclass {@link #execute()} method.
 */
public abstract class BehaviorTreeNode {

    private final String name;

    // The result from the last tick() call. Starts as FAILURE until first tick.
    private NodeStatus lastStatus = NodeStatus.FAILURE;

    /**
     * Creates a new behavior tree node.
     *
     * @param name human-readable name used for logging and visualization
     */
    protected BehaviorTreeNode(String name) {
        this.name = name;
    }

    /**
     * Returns the name of this node.
     *
     * @return the human-readable node name
     */
    public String getName() {
        return name;
    }

    /**
     * Returns the status from the last time this node was ticked.
     *
     * <p>Before the first tick, returns {@link NodeStatus#FAILURE} as the default.
     *
     * @return the last known status of this node
     */
    public NodeStatus getLastStatus() {
        return lastStatus;
    }

    /**
     * Ticks this node once, updating and returning its current status.
     *
     * <p>Call this from the {@link BehaviorTree} each robot loop. The result is stored and
     * accessible via {@link #getLastStatus()}.
     *
     * @return the status after executing this node's logic
     */
    public final NodeStatus tick() {
        lastStatus = execute();
        return lastStatus;
    }

    /**
     * Implements this node's logic. Must be overridden in every subclass.
     *
     * <p>This method is called by {@link #tick()} and should NOT be called directly.
     *
     * @return the status of this node after executing its logic
     */
    protected abstract NodeStatus execute();

    /**
     * Returns the child nodes of this node, used for tree visualization and traversal.
     *
     * <p>Leaf nodes return an empty list (the default). Composite and decorator nodes override this
     * to return their children.
     *
     * @return list of child nodes (empty for leaf nodes)
     */
    public List<BehaviorTreeNode> getChildren() {
        return new ArrayList<>();
    }

    /**
     * Returns a short type label for this node, used in visualization.
     *
     * <p>Examples: {@code "Sequence"}, {@code "Selector"}, {@code "Condition"}, {@code "Command"}
     *
     * @return the type name of this node
     */
    public abstract String getType();

    /**
     * Resets this node and all its children to their initial state.
     *
     * <p>Call this when you want to restart the tree from scratch (e.g., at the start of a new
     * autonomous routine). Subclasses that hold state (like {@link SequenceNode} tracking its
     * current child index) override this to clear that state.
     */
    public void reset() {
        lastStatus = NodeStatus.FAILURE;
        for (BehaviorTreeNode child : getChildren()) {
            child.reset();
        }
    }
}
