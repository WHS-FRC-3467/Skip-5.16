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
package frc.lib.util;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Annotation to automatically register a command method on the SmartDashboard during build time.
 *
 * <p>
 * When applied to a command factory method in a subsystem, this annotation generates code that will
 * call {@code SmartDashboard.putData()} with the specified key and the command returned by the
 * annotated method.
 *
 * <p>
 * <b>Example Usage:</b>
 *
 * <pre>{@code
 * public class IntakeSuperstructure extends SubsystemBase {
 *     &#64;SmartDashboardCommand(key = "Intake Linear/Extend")
 *     public Command extendIntake() {
 *         return Commands.sequence(
 *             runRoller(() -> RotationsPerSecond.of(10.0)),
 *             extendLinear())
 *             .withName("Extend Intake");
 *     }
 * }
 * }</pre>
 *
 * <p>
 * This will generate code equivalent to:
 *
 * <pre>{@code
 * SmartDashboard.putData("Intake Linear/Extend", intake.extendIntake());
 * }</pre>
 *
 * <p>
 * <b>Key Naming Conventions:</b>
 * <ul>
 * <li>Use forward slashes to create dashboard groups (e.g., "Subsystem/Command Name")</li>
 * <li>Consider using the subsystem's NAME constant for consistency</li>
 * </ul>
 *
 * <p>
 * <b>Requirements:</b>
 * <ul>
 * <li>Must be applied to a public, non-static method that returns {@code Command}</li>
 * <li>The containing class must be a subsystem accessible from {@code RobotContainer}</li>
 * <li>The key must be a non-empty string</li>
 * </ul>
 */
@Retention(RetentionPolicy.SOURCE)
@Target(ElementType.METHOD)
public @interface SmartDashboardCommand {
    /**
     * The key to use when registering the command on SmartDashboard. This key determines where the
     * command appears in the SmartDashboard hierarchy.
     *
     * <p>
     * Example: "Intake Linear/Extend" will create a command under the "Intake Linear" group with
     * the name "Extend".
     *
     * @return the SmartDashboard key for this command
     */
    String key();
}
