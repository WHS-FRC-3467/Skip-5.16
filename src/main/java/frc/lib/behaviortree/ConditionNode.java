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

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

/**
 * A leaf node that evaluates a boolean condition each tick.
 *
 * <p>The condition can be any {@link BooleanSupplier} or WPILib {@link Trigger}. It checks the
 * condition instantly and returns immediately—condition nodes never return RUNNING.
 *
 * <p>Behavior:
 *
 * <ul>
 *   <li>Condition is {@code true} → returns {@link NodeStatus#SUCCESS}
 *   <li>Condition is {@code false} → returns {@link NodeStatus#FAILURE}
 * </ul>
 *
 * <p>Common uses: checking sensor states, Trigger states, robot mode, game piece presence.
 *
 * <p>Example:
 *
 * <pre>{@code
 * // Using a LoggedTrigger from a subsystem
 * new ConditionNode("BallStaged", tower.isStaged)
 *
 * // Using an inline lambda
 * new ConditionNode("ShooterReady", shooter::isAtSetpoint)
 *
 * // Using a boolean supplier from robotState
 * new ConditionNode("FacingTarget", robotState.facingTarget)
 * }</pre>
 */
public class ConditionNode extends BehaviorTreeNode {

    private final BooleanSupplier condition;

    /**
     * Creates a condition node from a {@link BooleanSupplier} or lambda.
     *
     * @param name display name for logging and visualization
     * @param condition the condition to evaluate each tick
     */
    public ConditionNode(String name, BooleanSupplier condition) {
        super(name);
        this.condition = condition;
    }

    /**
     * Creates a condition node from a WPILib {@link Trigger}.
     *
     * <p>Since {@link Trigger} implements {@link BooleanSupplier}, this is equivalent to passing
     * the trigger directly, but this overload makes the intent clear.
     *
     * @param name display name for logging and visualization
     * @param trigger the trigger to check each tick
     */
    public ConditionNode(String name, Trigger trigger) {
        super(name);
        this.condition = trigger;
    }

    @Override
    protected NodeStatus execute() {
        return condition.getAsBoolean() ? NodeStatus.SUCCESS : NodeStatus.FAILURE;
    }

    @Override
    public String getType() {
        return "Condition";
    }
}
