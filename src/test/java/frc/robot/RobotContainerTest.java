/* Copyright (C) 2025 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in
 the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

 package frc.robot;

 import static org.junit.jupiter.api.Assertions.fail;
 
 import org.junit.jupiter.api.Test;
 
 public class RobotContainerTest {
 
     @Test
     public void createRobotContainer() {
         // Instantiate RobotContainer
         try {
             new RobotContainer();
         } catch (Exception e) {
             e.printStackTrace();
             fail("Failed to instantiate RobotContainer, see stack trace above.");
         }
     }
 }