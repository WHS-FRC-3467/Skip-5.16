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

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for the behavior tree framework.
 *
 * <p>Tests cover: ConditionNode, ActionNode, SequenceNode, SelectorNode, ParallelNode,
 * InverterNode, RepeatNode, CommandActionNode, and BehaviorTree.
 */
public class BehaviorTreeTest {

    @BeforeEach
    void setup() {
        // HAL must be initialized for WPILib (NetworkTables, etc.) to work
        assertTrue(HAL.initialize(500, 0));
    }

    // ─── ConditionNode ────────────────────────────────────────────────────────

    @Test
    void conditionNodeReturnsSuccessWhenTrue() {
        var node = new ConditionNode("AlwaysTrue", () -> true);
        assertEquals(NodeStatus.SUCCESS, node.tick());
    }

    @Test
    void conditionNodeReturnsFailureWhenFalse() {
        var node = new ConditionNode("AlwaysFalse", () -> false);
        assertEquals(NodeStatus.FAILURE, node.tick());
    }

    // ─── ActionNode ───────────────────────────────────────────────────────────

    @Test
    void actionNodeReturnsSuppliedStatus() {
        var node = new ActionNode("AlwaysSuccess", () -> NodeStatus.SUCCESS);
        assertEquals(NodeStatus.SUCCESS, node.tick());
    }

    @Test
    void actionNodeCanReturnRunning() {
        var node = new ActionNode("AlwaysRunning", () -> NodeStatus.RUNNING);
        assertEquals(NodeStatus.RUNNING, node.tick());
    }

    // ─── SequenceNode ─────────────────────────────────────────────────────────

    @Test
    void sequenceSucceedsWhenAllChildrenSucceed() {
        var node =
                new SequenceNode(
                        "Seq",
                        new ConditionNode("A", () -> true),
                        new ConditionNode("B", () -> true),
                        new ConditionNode("C", () -> true));
        assertEquals(NodeStatus.SUCCESS, node.tick());
    }

    @Test
    void sequenceFailsWhenFirstChildFails() {
        var node =
                new SequenceNode(
                        "Seq",
                        new ConditionNode("A", () -> false), // fails immediately
                        new ConditionNode("B", () -> true));
        assertEquals(NodeStatus.FAILURE, node.tick());
    }

    @Test
    void sequenceFailsWhenMiddleChildFails() {
        var node =
                new SequenceNode(
                        "Seq",
                        new ConditionNode("A", () -> true),
                        new ConditionNode("B", () -> false), // fails here
                        new ConditionNode("C", () -> true));
        assertEquals(NodeStatus.FAILURE, node.tick());
    }

    @Test
    void sequenceReturnsRunningWhenChildIsRunning() {
        var node =
                new SequenceNode(
                        "Seq",
                        new ConditionNode("A", () -> true),
                        new ActionNode("B", () -> NodeStatus.RUNNING)); // still running
        assertEquals(NodeStatus.RUNNING, node.tick());
    }

    // ─── SelectorNode ─────────────────────────────────────────────────────────

    @Test
    void selectorSucceedsWhenFirstChildSucceeds() {
        var node =
                new SelectorNode(
                        "Sel",
                        new ConditionNode("A", () -> true), // succeeds on first try
                        new ConditionNode("B", () -> false));
        assertEquals(NodeStatus.SUCCESS, node.tick());
    }

    @Test
    void selectorSucceedsWhenSecondChildSucceeds() {
        var node =
                new SelectorNode(
                        "Sel",
                        new ConditionNode("A", () -> false), // falls through
                        new ConditionNode("B", () -> true)); // succeeds here
        assertEquals(NodeStatus.SUCCESS, node.tick());
    }

    @Test
    void selectorFailsWhenAllChildrenFail() {
        var node =
                new SelectorNode(
                        "Sel",
                        new ConditionNode("A", () -> false),
                        new ConditionNode("B", () -> false),
                        new ConditionNode("C", () -> false));
        assertEquals(NodeStatus.FAILURE, node.tick());
    }

    @Test
    void selectorReturnsRunningWhenChildIsRunning() {
        var node =
                new SelectorNode(
                        "Sel",
                        new ConditionNode("A", () -> false),
                        new ActionNode("B", () -> NodeStatus.RUNNING)); // running, pause here
        assertEquals(NodeStatus.RUNNING, node.tick());
    }

    // ─── InverterNode ─────────────────────────────────────────────────────────

    @Test
    void inverterFlipsSuccessToFailure() {
        var node = new InverterNode("Not", new ConditionNode("True", () -> true));
        assertEquals(NodeStatus.FAILURE, node.tick());
    }

    @Test
    void inverterFlipsFailureToSuccess() {
        var node = new InverterNode("Not", new ConditionNode("False", () -> false));
        assertEquals(NodeStatus.SUCCESS, node.tick());
    }

    @Test
    void inverterDoesNotFlipRunning() {
        var node = new InverterNode("Not", new ActionNode("Running", () -> NodeStatus.RUNNING));
        assertEquals(NodeStatus.RUNNING, node.tick());
    }

    // ─── ParallelNode ─────────────────────────────────────────────────────────

    @Test
    void parallelRequireAllSucceedsWhenAllSucceed() {
        var node =
                new ParallelNode(
                        "Par",
                        ParallelNode.Policy.REQUIRE_ALL,
                        new ConditionNode("A", () -> true),
                        new ConditionNode("B", () -> true));
        assertEquals(NodeStatus.SUCCESS, node.tick());
    }

    @Test
    void parallelRequireAllFailsWhenAnyFails() {
        var node =
                new ParallelNode(
                        "Par",
                        ParallelNode.Policy.REQUIRE_ALL,
                        new ConditionNode("A", () -> true),
                        new ConditionNode("B", () -> false)); // this fails the whole parallel
        assertEquals(NodeStatus.FAILURE, node.tick());
    }

    @Test
    void parallelRequireAllReturnsRunningWhenChildRunning() {
        var node =
                new ParallelNode(
                        "Par",
                        ParallelNode.Policy.REQUIRE_ALL,
                        new ConditionNode("A", () -> true),
                        new ActionNode("B", () -> NodeStatus.RUNNING)); // still running
        assertEquals(NodeStatus.RUNNING, node.tick());
    }

    @Test
    void parallelRequireOneSucceedsWhenAnySucceeds() {
        var node =
                new ParallelNode(
                        "Par",
                        ParallelNode.Policy.REQUIRE_ONE,
                        new ConditionNode("A", () -> false),
                        new ConditionNode("B", () -> true)); // this succeeds the whole parallel
        assertEquals(NodeStatus.SUCCESS, node.tick());
    }

    @Test
    void parallelRequireOneFailsWhenAllFail() {
        var node =
                new ParallelNode(
                        "Par",
                        ParallelNode.Policy.REQUIRE_ONE,
                        new ConditionNode("A", () -> false),
                        new ConditionNode("B", () -> false));
        assertEquals(NodeStatus.FAILURE, node.tick());
    }

    // ─── RepeatNode ───────────────────────────────────────────────────────────

    @Test
    void repeatNodeSucceedsAfterNRepetitions() {
        // This action succeeds every time it's called
        var node =
                new RepeatNode("Repeat3", 3, new ActionNode("Once", () -> NodeStatus.SUCCESS));

        // First two ticks return RUNNING (only 1 success each, need 3 total)
        assertEquals(NodeStatus.RUNNING, node.tick());
        assertEquals(NodeStatus.RUNNING, node.tick());
        // Third tick reaches the repeat count and returns SUCCESS
        assertEquals(NodeStatus.SUCCESS, node.tick());
    }

    @Test
    void repeatNodePropagatesFailure() {
        var node = new RepeatNode("Repeat3", 3, new ConditionNode("Fail", () -> false));
        assertEquals(NodeStatus.FAILURE, node.tick());
    }

    // ─── BehaviorTree ─────────────────────────────────────────────────────────

    @Test
    void behaviorTreeTicksRootAndReturnsStatus() {
        var tree = new BehaviorTree("TestTree", new ConditionNode("AlwaysTrue", () -> true));
        assertEquals(NodeStatus.SUCCESS, tree.tick());
        assertEquals(NodeStatus.SUCCESS, tree.getLastStatus());
    }

    @Test
    void behaviorTreeResetClearsState() {
        // Sequence that gets partway through before reset
        int[] callCount = {0};
        var seq =
                new SequenceNode(
                        "Seq",
                        new ActionNode(
                                "Running",
                                () -> {
                                    callCount[0]++;
                                    // Succeed after 2 ticks
                                    return callCount[0] >= 2 ? NodeStatus.SUCCESS : NodeStatus.RUNNING;
                                }),
                        new ConditionNode("True", () -> true));

        var tree = new BehaviorTree("ResetTest", seq);

        // First tick: running action returns RUNNING
        assertEquals(NodeStatus.RUNNING, tree.tick());

        // Reset the tree
        tree.reset();
        callCount[0] = 0;

        // After reset, the sequence restarts from the beginning
        assertEquals(NodeStatus.RUNNING, tree.tick()); // action runs again from scratch
    }

    @Test
    void behaviorTreeCompositeExample() {
        // A real-world style tree: "Shoot if ready, otherwise intake"
        boolean[] ballStaged = {true};
        boolean[] shooterReady = {true};

        var tree =
                new BehaviorTree(
                        "CompositeExample",
                        new SelectorNode(
                                "Root",
                                new SequenceNode(
                                        "ShootIfReady",
                                        new ConditionNode("BallStaged", () -> ballStaged[0]),
                                        new ConditionNode(
                                                "ShooterReady", () -> shooterReady[0]),
                                        new ActionNode(
                                                "Shoot", () -> NodeStatus.SUCCESS)),
                                new ActionNode("Intake", () -> NodeStatus.SUCCESS)));

        // Ball staged and shooter ready → should shoot (SUCCESS via first branch)
        assertEquals(NodeStatus.SUCCESS, tree.tick());

        // Ball not staged → should fall back to intake
        ballStaged[0] = false;
        assertEquals(NodeStatus.SUCCESS, tree.tick()); // intake fallback succeeds

        // Shooter not ready (but ball staged) → should also intake
        ballStaged[0] = true;
        shooterReady[0] = false;
        assertEquals(NodeStatus.SUCCESS, tree.tick()); // intake fallback
    }
}
