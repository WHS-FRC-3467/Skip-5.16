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

import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * A behavior tree that makes logical decisions and executes WPILib Commands based on Trigger
 * conditions.
 *
 * <p>A behavior tree is a hierarchical structure of nodes that are evaluated ("ticked") every robot
 * loop. Each tick evaluates the tree from the root and returns a {@link NodeStatus}. The tree
 * automatically logs its state to AdvantageKit and publishes live data to NetworkTables for
 * real-time visualization in the web viewer ({@code /behaviortree/index.html} on the robot).
 *
 * <p>The tree is composed of:
 *
 * <ul>
 *   <li>{@link ConditionNode} — checks a {@link edu.wpi.first.wpilibj2.command.button.Trigger} or
 *       {@link java.util.function.BooleanSupplier}
 *   <li>{@link CommandActionNode} — runs a WPILib {@link edu.wpi.first.wpilibj2.command.Command}
 *   <li>{@link ActionNode} — runs custom logic
 *   <li>{@link SequenceNode} — AND logic (all children must succeed)
 *   <li>{@link SelectorNode} — OR logic (try children until one succeeds)
 *   <li>{@link ParallelNode} — run all children simultaneously
 *   <li>{@link InverterNode} — inverts child result
 *   <li>{@link RepeatNode} — repeats child N times
 * </ul>
 *
 * <p><b>Usage example:</b>
 *
 * <pre>{@code
 * // Build the tree once (e.g., in a subsystem constructor or RobotContainer)
 * BehaviorTree autoTree = new BehaviorTree("AutoDecision",
 *     new SelectorNode("Main",
 *         // Priority 1: Shoot if everything is ready
 *         new SequenceNode("ShootIfReady",
 *             new ConditionNode("BallStaged",    tower.isStaged),
 *             new ConditionNode("ShooterReady",  shooter.readyToShoot),
 *             new ConditionNode("RobotAimed",    robotState.facingTarget),
 *             new CommandActionNode("Shoot",     () -> Commands.parallel(indexer.shoot(), tower.shoot()))
 *         ),
 *         // Priority 2: Intake if no game piece
 *         new SequenceNode("IntakeIfEmpty",
 *             new InverterNode("NoBall",         new ConditionNode("BallStaged", tower.isStaged)),
 *             new CommandActionNode("Intake",    intake::extendIntake)
 *         ),
 *         // Fallback: Idle
 *         new ActionNode("Idle", () -> NodeStatus.SUCCESS)
 *     )
 * );
 *
 * // In a periodic() method (e.g., in a subsystem or RobotContainer):
 * autoTree.tick();
 * }</pre>
 *
 * <p><b>Visualization:</b> Open {@code http://roboRIO-XXXX-FRC.local:5800/behaviortree/index.html}
 * in a browser (or {@code http://localhost:5800/behaviortree/index.html} in simulation) to see
 * the tree executing in real time. Nodes are color-coded: green = SUCCESS, red = FAILURE, yellow =
 * RUNNING, gray = not yet evaluated.
 */
public class BehaviorTree {

    private final String name;
    private final BehaviorTreeNode root;

    // NetworkTables publishers for real-time web visualization
    private final StringArrayPublisher nodeNamesPublisher;
    private final StringArrayPublisher nodeTypesPublisher;
    private final StringArrayPublisher nodeStatusesPublisher;
    private final IntegerArrayPublisher nodeDepthsPublisher;
    private final StringPublisher structurePublisher;

    // Reusable lists for DFS traversal results (avoids allocations each tick)
    private final List<String> nodeNames = new ArrayList<>();
    private final List<String> nodeTypes = new ArrayList<>();
    private final List<NodeStatus> nodeStatuses = new ArrayList<>();
    private final List<Integer> nodeDepths = new ArrayList<>();

    /**
     * Creates a new behavior tree.
     *
     * <p>The tree structure is published to NetworkTables immediately so the web viewer can display
     * the layout before the first tick.
     *
     * @param name the tree name, used as the NetworkTables topic prefix and AdvantageKit log key
     *     (e.g., {@code "AutoDecision"} → topic {@code /BehaviorTree/AutoDecision/...})
     * @param root the root node of the tree
     */
    public BehaviorTree(String name, BehaviorTreeNode root) {
        this.name = name;
        this.root = root;

        // Create NetworkTables publishers under /BehaviorTree/<name>/
        String baseTopic = "/BehaviorTree/" + name;
        NetworkTableInstance nt = NetworkTableInstance.getDefault();

        nodeNamesPublisher = nt.getStringArrayTopic(baseTopic + "/NodeNames").publish();
        nodeTypesPublisher = nt.getStringArrayTopic(baseTopic + "/NodeTypes").publish();
        nodeStatusesPublisher = nt.getStringArrayTopic(baseTopic + "/NodeStatuses").publish();
        nodeDepthsPublisher = nt.getIntegerArrayTopic(baseTopic + "/NodeDepths").publish();
        structurePublisher = nt.getStringTopic(baseTopic + "/Structure").publish();

        // Publish the static tree structure so the web viewer can render the layout
        String structure = buildStructureJson(root);
        structurePublisher.set(structure);
        Logger.recordOutput("BehaviorTree/" + name + "/Structure", structure);
    }

    /**
     * Ticks the behavior tree once, evaluating the root node and all its children.
     *
     * <p>This should be called once per robot loop (typically in a subsystem's {@code periodic()}
     * method or in {@code RobotContainer}). After each tick, the updated node states are published
     * to NetworkTables and logged to AdvantageKit.
     *
     * @return the status of the root node after this tick
     */
    public NodeStatus tick() {
        NodeStatus status = root.tick();

        // Collect current node states via depth-first traversal
        nodeNames.clear();
        nodeTypes.clear();
        nodeStatuses.clear();
        nodeDepths.clear();
        collectNodeData(root, 0);

        // Publish to NetworkTables for the web visualization
        String[] namesArray = nodeNames.toArray(new String[0]);
        String[] typesArray = nodeTypes.toArray(new String[0]);
        String[] statusesArray = nodeStatuses.stream().map(Enum::name).toArray(String[]::new);
        long[] depthsArray = nodeDepths.stream().mapToLong(Integer::longValue).toArray();

        nodeNamesPublisher.set(namesArray);
        nodeTypesPublisher.set(typesArray);
        nodeStatusesPublisher.set(statusesArray);
        nodeDepthsPublisher.set(depthsArray);

        // Also log to AdvantageKit for replay and post-match analysis
        Logger.recordOutput("BehaviorTree/" + name + "/RootStatus", status.name());
        Logger.recordOutput("BehaviorTree/" + name + "/NodeNames", namesArray);
        Logger.recordOutput("BehaviorTree/" + name + "/NodeStatuses", statusesArray);

        return status;
    }

    /**
     * Resets the tree to its initial state.
     *
     * <p>Call this when starting a new autonomous routine or when you want the tree to re-evaluate
     * from scratch. All stateful nodes (like {@link SequenceNode} and {@link CommandActionNode})
     * will have their state cleared.
     */
    public void reset() {
        root.reset();
    }

    /**
     * Returns the name of this behavior tree.
     *
     * @return the tree name
     */
    public String getName() {
        return name;
    }

    /**
     * Returns the root node of this tree.
     *
     * @return the root node
     */
    public BehaviorTreeNode getRoot() {
        return root;
    }

    /**
     * Returns the status from the last call to {@link #tick()}.
     *
     * @return the last known root status
     */
    public NodeStatus getLastStatus() {
        return root.getLastStatus();
    }

    /** Walks the tree depth-first and collects node metadata into the flat lists. */
    private void collectNodeData(BehaviorTreeNode node, int depth) {
        nodeNames.add(node.getName());
        nodeTypes.add(node.getType());
        nodeStatuses.add(node.getLastStatus());
        nodeDepths.add(depth);

        for (BehaviorTreeNode child : node.getChildren()) {
            collectNodeData(child, depth + 1);
        }
    }

    /** Builds a JSON representation of the tree structure for the web visualization layout. */
    private String buildStructureJson(BehaviorTreeNode node) {
        StringBuilder sb = new StringBuilder();
        sb.append("{\"name\":\"")
                .append(escapeJson(node.getName()))
                .append("\",\"type\":\"")
                .append(node.getType())
                .append("\"");

        List<BehaviorTreeNode> children = node.getChildren();
        if (!children.isEmpty()) {
            sb.append(",\"children\":[");
            for (int i = 0; i < children.size(); i++) {
                if (i > 0) sb.append(",");
                sb.append(buildStructureJson(children.get(i)));
            }
            sb.append("]");
        }

        sb.append("}");
        return sb.toString();
    }

    /** Escapes special characters for safe JSON string embedding. */
    private static String escapeJson(String s) {
        return s.replace("\\", "\\\\").replace("\"", "\\\"");
    }
}
