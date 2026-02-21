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

/**
 * Represents the execution status of a behavior tree node after a tick.
 *
 * <p>Every behavior tree node returns one of these three states when ticked:
 *
 * <ul>
 *   <li>{@link #SUCCESS} - The node completed its task successfully
 *   <li>{@link #FAILURE} - The node failed to complete its task
 *   <li>{@link #RUNNING} - The node is still working (e.g. a long-running Command)
 * </ul>
 *
 * <p>Think of it like a traffic light: GREEN = Success, RED = Failure, YELLOW = Running.
 */
public enum NodeStatus {
    /** The node completed its task successfully. */
    SUCCESS,

    /** The node failed to complete its task. */
    FAILURE,

    /**
     * The node is still executing. Used for long-running tasks like WPILib Commands that take
     * multiple loop iterations to complete.
     */
    RUNNING
}
