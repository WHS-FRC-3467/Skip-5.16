/*
 * Copyright (C) 2025 Windham Windup
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

import edu.wpi.first.math.geometry.Translation2d;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.List;

public class PointInPolygon {
    // Checking if a point is inside a polygon
    public static boolean pointInPolygon(Translation2d point, List<Translation2d> polygon)
    {
        Path2D path = new Path2D.Double();

        // Move to the first point in the polygon
        path.moveTo(polygon.get(0).getX(), polygon.get(0).getY());

        // Connect the points in the polygon
        for (int i = 1; i < polygon.size(); i++) {
            path.lineTo(polygon.get(i).getX(), polygon.get(i).getY());
        }

        // Close the path
        path.closePath();

        // Create a Point2D object for the test point
        Point2D testPoint = new Point2D.Double(point.getX(), point.getY());

        // Check if the test point is inside the polygon
        return path.contains(testPoint);
    }
}
