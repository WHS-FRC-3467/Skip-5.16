package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.RobotState;
import lombok.Getter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Simple 3D physics simulator for "fuel" (game pieces) used in simulation and testing.
 *
 * <p>
 * The simulator models gravity, air drag, collisions with field geometry, hubs, robots, trenches,
 * and fuel-to-fuel interactions. It can publish fuel positions to NetworkTables for visualization
 * and integrates with a registered robot pose and field-relative speeds.
 */
public class FuelSim {
    protected static final double PERIOD = 0.02; // sec
    protected static final Translation3d GRAVITY = new Translation3d(0, 0, -9.81); // m/s^2
    // Room temperature dry air density: https://en.wikipedia.org/wiki/Density_of_air#Dry_air
    protected static final double AIR_DENSITY = 1.2041; // kg/m^3
    protected static final double FIELD_COR = Math.sqrt(22 / 51.5); // coefficient of restitution
                                                                    // with the field
    protected static final double FUEL_COR = 0.5; // coefficient of restitution with another fuel
    protected static final double NET_COR = 0.2; // coefficient of restitution with the net
    protected static final double ROBOT_COR = 0.1; // coefficient of restitution with a robot
    protected static final double FUEL_RADIUS = 0.075;
    protected static final double FIELD_LENGTH = 16.51;
    protected static final double FIELD_WIDTH = 8.04;
    protected static final double TRENCH_WIDTH = 1.265;
    protected static final double TRENCH_BLOCK_WIDTH = 0.305;
    protected static final double TRENCH_HEIGHT = 0.565;
    protected static final double TRENCH_BAR_HEIGHT = 0.102;
    protected static final double TRENCH_BAR_WIDTH = 0.152;
    protected static final double FRICTION = 0.1; // proportion of horizontal vel to lose per sec
                                                  // while on ground
    protected static final double FUEL_MASS = 0.448 * 0.45392; // kgs
    protected static final double FUEL_CROSS_AREA = Math.PI * FUEL_RADIUS * FUEL_RADIUS;
    // Drag coefficient of smooth sphere:
    // https://en.wikipedia.org/wiki/Drag_coefficient#/media/File:14ilf1l.svg
    protected static final double DRAG_COF = 0.47; // dimensionless
    protected static final double DRAG_FORCE_FACTOR =
        0.5 * AIR_DENSITY * DRAG_COF * FUEL_CROSS_AREA;

    protected static final int HOPPER_CAPACITY = 50;
    protected static final double HOPPER_FLOOR_HEIGHT = 0.25; // TODO: Replace with actual value, in
                                                              // meters

    Pose2d previousRobot = new Pose2d();

    @Getter
    private int heldFuel = 0;

    protected static final Translation3d[] FIELD_XZ_LINE_STARTS = {
            new Translation3d(0, 0, 0),
            new Translation3d(3.96, 1.57, 0),
            new Translation3d(3.96, FIELD_WIDTH / 2 + 0.60, 0),
            new Translation3d(4.61, 1.57, 0.165),
            new Translation3d(4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
            new Translation3d(FIELD_LENGTH - 5.18, 1.57, 0),
            new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH / 2 + 0.60, 0),
            new Translation3d(FIELD_LENGTH - 4.61, 1.57, 0.165),
            new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
            new Translation3d(3.96, TRENCH_WIDTH, TRENCH_HEIGHT),
            new Translation3d(3.96, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
            new Translation3d(FIELD_LENGTH - 5.18, TRENCH_WIDTH, TRENCH_HEIGHT),
            new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
            new Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
            new Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57,
                TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
            new Translation3d(FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, 0,
                TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
            new Translation3d(
                FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57,
                TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    };

    protected static final Translation3d[] FIELD_XZ_LINE_ENDS = {
            new Translation3d(FIELD_LENGTH, FIELD_WIDTH, 0),
            new Translation3d(4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
            new Translation3d(4.61, FIELD_WIDTH - 1.57, 0.165),
            new Translation3d(5.18, FIELD_WIDTH / 2 - 0.60, 0),
            new Translation3d(5.18, FIELD_WIDTH - 1.57, 0),
            new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
            new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH - 1.57, 0.165),
            new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH / 2 - 0.60, 0),
            new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57, 0),
            new Translation3d(5.18, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
            new Translation3d(5.18, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
            new Translation3d(FIELD_LENGTH - 3.96, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
                TRENCH_HEIGHT),
            new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH,
                TRENCH_HEIGHT),
            new Translation3d(
                4.61 + TRENCH_BAR_WIDTH / 2, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
                TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
            new Translation3d(4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH,
                TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
            new Translation3d(
                FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2,
                TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
                TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
            new Translation3d(FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH,
                TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    };

    /** Internal representation of a single fuel (ball) in the simulation. */
    protected static class Fuel {
        protected Translation3d pos;
        protected Translation3d vel;
        protected boolean inHopper;
        protected int indexInHopper;

        protected Fuel(Translation3d pos, Translation3d vel)
        {
            this.pos = pos;
            this.vel = vel;
            this.inHopper = false;
            this.indexInHopper = -1;
        }

        protected Fuel(Translation3d pos)
        {
            this(pos, new Translation3d());
        }

        /**
         * Advance the fuel's state for a single physics substep.
         *
         * @param simulateAirResistance whether to apply drag
         * @param subticks number of substeps per main step (used to scale integration)
         */
        protected void update(boolean simulateAirResistance, int subticks)
        {
            if (!inHopper) {
                pos = pos.plus(vel.times(PERIOD / subticks));
                if (pos.getZ() > FUEL_RADIUS) {
                    Translation3d Fg = GRAVITY.times(FUEL_MASS);
                    Translation3d Fd = new Translation3d();

                    if (simulateAirResistance) {
                        double speed = vel.getNorm();
                        if (speed > 1e-6) {
                            Fd = vel.times(-DRAG_FORCE_FACTOR * speed);
                        }
                    }

                    Translation3d accel = Fg.plus(Fd).div(FUEL_MASS);
                    vel = vel.plus(accel.times(PERIOD / subticks));
                }
                if (Math.abs(vel.getZ()) < 0.05 && pos.getZ() <= FUEL_RADIUS + 0.03) {
                    vel = new Translation3d(vel.getX(), vel.getY(), 0);
                    vel = vel.times(1 - FRICTION * PERIOD / subticks);
                    // pos = new Translation3d(pos.getX(), pos.getY(), FUEL_RADIUS);
                }
                handleFieldCollisions(subticks);
            }
        }

        /**
         * Check and resolve collisions between this fuel and an XZ-aligned line segment.
         *
         * <p>
         * The method projects the fuel center into the XZ plane, finds the closest point on the
         * segment, and if the distance is less than the ball radius applies a position correction
         * and velocity reflection using the field restitution.
         *
         * @param lineStart start point of the line in XZ coordinates (y ignored)
         * @param lineEnd end point of the line in XZ coordinates (y ignored)
         */
        protected void handleXZLineCollision(Translation3d lineStart, Translation3d lineEnd)
        {
            if (pos.getY() < lineStart.getY() || pos.getY() > lineEnd.getY())
                return; // not within y range
            // Convert into 2D
            Translation2d start2d = new Translation2d(lineStart.getX(), lineStart.getZ());
            Translation2d end2d = new Translation2d(lineEnd.getX(), lineEnd.getZ());
            Translation2d pos2d = new Translation2d(pos.getX(), pos.getZ());
            Translation2d lineVec = end2d.minus(start2d);

            // Get closest point on line
            Translation2d projected =
                start2d.plus(
                    lineVec.times(pos2d.minus(start2d).dot(lineVec) / lineVec.getSquaredNorm()));

            if (projected.getDistance(start2d) + projected.getDistance(end2d) > lineVec.getNorm())
                return; // projected point not on line
            double dist = pos2d.getDistance(projected);
            if (dist > FUEL_RADIUS)
                return; // not intersecting line
            // Back into 3D
            Translation3d normal =
                new Translation3d(-lineVec.getY(), 0, lineVec.getX()).div(lineVec.getNorm());

            // Apply collision response
            pos = pos.plus(normal.times(FUEL_RADIUS - dist));
            if (vel.dot(normal) > 0)
                return; // already moving away from line
            vel = vel.minus(normal.times((1 + FIELD_COR) * vel.dot(normal)));
        }

        /**
         * Handle collisions between this fuel and the field: floor, edges, hubs and trenches.
         *
         * @param subticks number of physics substeps used for time integration
         */
        protected void handleFieldCollisions(int subticks)
        {
            // floor and bumps
            for (int i = 0; i < FIELD_XZ_LINE_STARTS.length; i++) {
                handleXZLineCollision(FIELD_XZ_LINE_STARTS[i], FIELD_XZ_LINE_ENDS[i]);
            }

            // edges
            if (pos.getX() < FUEL_RADIUS && vel.getX() < 0) {
                pos = pos.plus(new Translation3d(FUEL_RADIUS - pos.getX(), 0, 0));
                vel = vel.plus(new Translation3d(-(1 + FIELD_COR) * vel.getX(), 0, 0));
            } else if (pos.getX() > FIELD_LENGTH - FUEL_RADIUS && vel.getX() > 0) {
                pos = pos.plus(new Translation3d(FIELD_LENGTH - FUEL_RADIUS - pos.getX(), 0, 0));
                vel = vel.plus(new Translation3d(-(1 + FIELD_COR) * vel.getX(), 0, 0));
            }

            if (pos.getY() < FUEL_RADIUS && vel.getY() < 0) {
                pos = pos.plus(new Translation3d(0, FUEL_RADIUS - pos.getY(), 0));
                vel = vel.plus(new Translation3d(0, -(1 + FIELD_COR) * vel.getY(), 0));
            } else if (pos.getY() > FIELD_WIDTH - FUEL_RADIUS && vel.getY() > 0) {
                pos = pos.plus(new Translation3d(0, FIELD_WIDTH - FUEL_RADIUS - pos.getY(), 0));
                vel = vel.plus(new Translation3d(0, -(1 + FIELD_COR) * vel.getY(), 0));
            }

            // hubs
            handleHubCollisions(Hub.BLUE_HUB, subticks);
            handleHubCollisions(Hub.RED_HUB, subticks);

            handleTrenchCollisions();
        }

        /** Delegate hub-specific collision handling to the provided hub. */
        protected void handleHubCollisions(Hub hub, int subticks)
        {
            hub.handleHubInteraction(this, subticks);
            hub.fuelCollideSide(this);

            double netCollision = hub.fuelHitNet(this);
            if (netCollision != 0) {
                pos = pos.plus(new Translation3d(netCollision, 0, 0));
                vel = new Translation3d(-vel.getX() * NET_COR, vel.getY() * NET_COR, vel.getZ());
            }
        }

        /** Check collisions against trench geometry and apply corrections. */
        protected void handleTrenchCollisions()
        {
            fuelCollideRectangle(
                this,
                new Translation3d(3.96, TRENCH_WIDTH, 0),
                new Translation3d(5.18, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT));
            fuelCollideRectangle(
                this,
                new Translation3d(3.96, FIELD_WIDTH - 1.57, 0),
                new Translation3d(5.18, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT));
            fuelCollideRectangle(
                this,
                new Translation3d(FIELD_LENGTH - 5.18, TRENCH_WIDTH, 0),
                new Translation3d(FIELD_LENGTH - 3.96, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
                    TRENCH_HEIGHT));
            fuelCollideRectangle(
                this,
                new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH - 1.57, 0),
                new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH,
                    TRENCH_HEIGHT));
            fuelCollideRectangle(
                this,
                new Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT),
                new Translation3d(
                    4.61 + TRENCH_BAR_WIDTH / 2,
                    TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
                    TRENCH_HEIGHT + TRENCH_BAR_HEIGHT));
            fuelCollideRectangle(
                this,
                new Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
                new Translation3d(4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH,
                    TRENCH_HEIGHT + TRENCH_BAR_HEIGHT));
            fuelCollideRectangle(
                this,
                new Translation3d(FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT),
                new Translation3d(
                    FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2,
                    TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
                    TRENCH_HEIGHT + TRENCH_BAR_HEIGHT));
            fuelCollideRectangle(
                this,
                new Translation3d(FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57,
                    TRENCH_HEIGHT),
                new Translation3d(
                    FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2,
                    FIELD_WIDTH,
                    TRENCH_HEIGHT + TRENCH_BAR_HEIGHT));
        }

        /** Add an instantaneous velocity impulse (m/s). */
        protected void addImpulse(Translation3d impulse)
        {
            vel = vel.plus(impulse);
        }

        /**
         * If the fuel is in the hopper, update it's pose by applying the robot's translation
         * since the last update. This keeps hopper-held fuel fixed relative to the robot.
         *
         * @param robotPose current robot pose
         * @param previousRobotPose previous robot pose used to compute delta
         */
        protected void updateHopper(Pose2d robotPose, Pose2d previousRobotPose)
        {
            // If this fuel is stored in the hopper (has an assigned index), compute its
            // absolute field position from the robot's current pose and the stored
            // robot-relative hopper location. This accounts for robot translation and
            // rotation every update so the fuel stays fixed relative to the robot.
            if (inHopper && indexInHopper >= 0) {
                Translation3d rel = Hopper.getRelativePosInHopper(indexInHopper);
                // Rotate the robot-relative X/Y by the robot heading and translate into field
                Translation2d rel2 = new Translation2d(rel.getX(), rel.getY())
                    .rotateBy(robotPose.getRotation());
                double fieldX = robotPose.getX() + rel2.getX();
                double fieldY = robotPose.getY() + rel2.getY();
                double fieldZ = rel.getZ();
                pos = new Translation3d(fieldX, fieldY, fieldZ);
                // while in hopper, zero velocity so physics won't move it
                vel = Translation3d.kZero;
            }
        }
    }

    /**
     * Resolve a collision between two fuels using a simple elastic impulse and positional
     * correction to avoid overlap.
     *
     * @param a first fuel
     * @param b second fuel
     */
    protected static void handleFuelCollision(Fuel a, Fuel b)
    {
        Translation3d normal = a.pos.minus(b.pos);
        double distance = normal.getNorm();
        if (distance == 0) {
            normal = new Translation3d(1, 0, 0);
            distance = 1;
        }
        normal = normal.div(distance);
        double impulse = 0.5 * (1 + FUEL_COR) * (b.vel.minus(a.vel).dot(normal));
        double intersection = FUEL_RADIUS * 2 - distance;
        a.pos = a.pos.plus(normal.times(intersection / 2));
        b.pos = b.pos.minus(normal.times(intersection / 2));
        a.addImpulse(normal.times(impulse));
        b.addImpulse(normal.times(-impulse));
    }

    protected static final double CELL_SIZE = 0.25;
    protected static final int GRID_COLS = (int) Math.ceil(FIELD_LENGTH / CELL_SIZE);
    protected static final int GRID_ROWS = (int) Math.ceil(FIELD_WIDTH / CELL_SIZE);

    @SuppressWarnings("unchecked")
    protected final ArrayList<Fuel>[][] grid = new ArrayList[GRID_COLS][GRID_ROWS];

    /**
     * Broad-phase + narrow-phase collision detection for fuels using a uniform grid. Packs fuels
     * into grid cells (CELL_SIZE) and tests neighboring cells for pairwise collisions to limit
     * complexity.
     *
     * @param fuels list of fuels to process
     */
    protected void handleFuelCollisions(ArrayList<Fuel> fuels)
    {
        // Clear grid
        for (int i = 0; i < GRID_COLS; i++) {
            for (int j = 0; j < GRID_ROWS; j++) {
                grid[i][j].clear();
            }
        }

        // Populate grid
        for (Fuel fuel : fuels) {
            int col = (int) (fuel.pos.getX() / CELL_SIZE);
            int row = (int) (fuel.pos.getY() / CELL_SIZE);

            if (col >= 0 && col < GRID_COLS && row >= 0 && row < GRID_ROWS) {
                grid[col][row].add(fuel);
            }
        }

        // Check collisions
        for (Fuel fuel : fuels) {
            int col = (int) (fuel.pos.getX() / CELL_SIZE);
            int row = (int) (fuel.pos.getY() / CELL_SIZE);

            // Check 3x3 neighbor cells
            for (int i = col - 1; i <= col + 1; i++) {
                for (int j = row - 1; j <= row + 1; j++) {
                    if (i >= 0 && i < GRID_COLS && j >= 0 && j < GRID_ROWS) {
                        for (Fuel other : grid[i][j]) {
                            if (fuel != other
                                && fuel.pos.getDistance(other.pos) < FUEL_RADIUS * 2) {
                                if (fuel.hashCode() < other.hashCode()) {
                                    handleFuelCollision(fuel, other);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    protected ArrayList<Fuel> fuels = new ArrayList<Fuel>();
    protected boolean running = false;
    protected boolean simulateAirResistance = false;
    protected Supplier<Pose2d> robotPoseSupplier = null;
    protected Supplier<ChassisSpeeds> robotFieldSpeedsSupplier = null;
    protected double robotWidth; // size along the robot's y axis
    protected double robotLength; // size along the robot's x axis
    protected double bumperHeight;
    protected ArrayList<SimIntake> intakes = new ArrayList<>();
    protected int subticks = 5;

    /**
     * Creates a new instance of FuelSim
     * 
     * @param tableKey NetworkTable to log fuel positions to as an array of {@link Translation3d}
     *        structs.
     */
    public FuelSim(String tableKey)
    {
        // Initialize grid
        for (int i = 0; i < GRID_COLS; i++) {
            for (int j = 0; j < GRID_ROWS; j++) {
                grid[i][j] = new ArrayList<Fuel>();
            }
        }

        fuelPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic(tableKey + "/Fuels", Translation3d.struct)
            .publish();
    }

    /**
     * Creates a new instance of FuelSim with log path "/Fuel Simulation"
     */
    public FuelSim()
    {
        this("/Fuel Simulation");
    }

    /**
     * Clears the field of fuel
     */
    public void clearFuel()
    {
        // Free any occupied hopper slots before removing fuels so slots can be reused.
        for (Fuel f : fuels) {
            if (f.inHopper && f.indexInHopper >= 0) {
                Hopper.freeIndex(f.indexInHopper);
            }
        }
        fuels.clear();
        // Reset held count to zero since all fuels were cleared
        heldFuel = 0;
    }

    /**
     * Spawns fuel in the neutral zone and depots
     */
    public void spawnStartingFuel()
    {
        // Center fuel
        Translation3d center = new Translation3d(FIELD_LENGTH / 2, FIELD_WIDTH / 2, FUEL_RADIUS);
        for (int i = 0; i < 15; i++) {
            for (int j = 0; j < 6; j++) {
                fuels.add(new Fuel(center
                    .plus(new Translation3d(0.076 + 0.152 * j, 0.0254 + 0.076 + 0.152 * i, 0))));
                fuels.add(new Fuel(center
                    .plus(new Translation3d(-0.076 - 0.152 * j, 0.0254 + 0.076 + 0.152 * i, 0))));
                fuels.add(new Fuel(center
                    .plus(new Translation3d(0.076 + 0.152 * j, -0.0254 - 0.076 - 0.152 * i, 0))));
                fuels.add(new Fuel(center
                    .plus(new Translation3d(-0.076 - 0.152 * j, -0.0254 - 0.076 - 0.152 * i, 0))));
            }
        }

        // Depots
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                fuels.add(new Fuel(
                    new Translation3d(0.076 + 0.152 * j, 5.95 + 0.076 + 0.152 * i, FUEL_RADIUS)));
                fuels.add(new Fuel(
                    new Translation3d(0.076 + 0.152 * j, 5.95 - 0.076 - 0.152 * i, FUEL_RADIUS)));
                fuels.add(new Fuel(
                    new Translation3d(FIELD_LENGTH - 0.076 - 0.152 * j, 2.09 + 0.076 + 0.152 * i,
                        FUEL_RADIUS)));
                fuels.add(new Fuel(
                    new Translation3d(FIELD_LENGTH - 0.076 - 0.152 * j, 2.09 - 0.076 - 0.152 * i,
                        FUEL_RADIUS)));
            }
        }

        // DEBUG: Log XZ lines
        // Translation3d[][] lines = new Translation3d[FIELD_XZ_LINE_STARTS.length][2];
        // for (int i = 0; i < FIELD_XZ_LINE_STARTS.length; i++) {
        // lines[i][0] = FIELD_XZ_LINE_STARTS[i];
        // lines[i][1] = FIELD_XZ_LINE_ENDS[i];
        // }

        // Logger.recordOutput("Fuel Simulation/Lines (debug)", lines);
    }

    protected StructArrayPublisher<Translation3d> fuelPublisher;

    /**
     * Adds array of `Translation3d`'s to NetworkTables at tableKey + "/Fuels"
     */
    public void logFuels()
    {
        fuelPublisher.set(fuels.stream().map((fuel) -> fuel.pos).toArray(Translation3d[]::new));
    }

    /**
     * Start the simulation. `updateSim` must still be called every loop
     */
    public void start()
    {
        running = true;
    }

    /**
     * Pause the simulation.
     */
    public void stop()
    {
        running = false;
    }

    /** Enables accounting for drag force in physics step **/
    public void enableAirResistance()
    {
        simulateAirResistance = true;
    }

    /**
     * Sets the number of physics iterations per loop (0.02s)
     * 
     * @param subticks number of physics substeps per main simulation step
     */
    public void setSubticks(int subticks)
    {
        this.subticks = subticks;
    }

    /**
     * Registers a robot with the fuel simulator
     * 
     * @param width from left to right (y-axis)
     * @param length from front to back (x-axis)
     * @param bumperHeight height of the robot bumpers
     * @param poseSupplier supplier that returns the current robot pose (field frame)
     * @param fieldSpeedsSupplier field-relative `ChassisSpeeds` supplier
     */
    public void registerRobot(
        double width,
        double length,
        double bumperHeight,
        Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> fieldSpeedsSupplier)
    {
        this.robotPoseSupplier = poseSupplier;
        this.robotFieldSpeedsSupplier = fieldSpeedsSupplier;
        this.robotWidth = width;
        this.robotLength = length;
        this.bumperHeight = bumperHeight;
    }

    /**
     * To be called periodically Will do nothing if sim is not running
     */
    public void updateSim()
    {
        if (!running)
            return;

        stepSim();
        Logger.recordOutput("FuelSim/NumberInHopper", heldFuel);
    }

    /**
     * Run the simulation forward 1 time step (0.02s)
     */
    public void stepSim()
    {
        for (int i = 0; i < subticks; i++) {
            for (Fuel fuel : fuels) {
                if (!fuel.inHopper) {
                    fuel.update(this.simulateAirResistance, this.subticks);
                }
            }

            handleFuelCollisions(fuels);

            if (robotPoseSupplier != null) {
                handleRobotCollisions(fuels);
                handleIntakes(fuels);
                handleHopper(fuels);
            }
        }

        logFuels();
    }

    /**
     * Adds a fuel onto the field
     * 
     * @param pos Position to spawn at
     * @param vel Initial velocity vector
     */
    public void spawnFuel(Translation3d pos, Translation3d vel)
    {
        fuels.add(new Fuel(pos, vel));
    }

    /**
     * Spawn a fuel using shooter parameters and robot state.
     *
     * <p>
     * Computes the launch pose from the registered robot pose plus the provided launch height,
     * converts shooter angles (hood + turret) into a field-relative velocity vector, adds the
     * robot's current field-relative velocity, and spawns the fuel at the computed position with
     * the computed velocity.
     *
     * @param launchVelocity initial speed of the shooter (units-aware)
     * @param hoodAngle pitch/hood angle (0 = horizontal)
     * @param turretYaw yaw of the turret relative to the robot
     * @param launchHeight vertical offset for spawn to avoid immediate robot collision
     * @throws IllegalStateException if robot is not registered with the simulator
     */
    public void launchFuel(LinearVelocity launchVelocity, Angle hoodAngle, Angle turretYaw,
        Distance launchHeight)
    {
        if (robotPoseSupplier == null || robotFieldSpeedsSupplier == null) {
            throw new IllegalStateException("Robot must be registered before launching fuel.");
        }

        // Build the launch pose: robot position with a vertical offset for the shooter
        Pose3d launchPose = new Pose3d(this.robotPoseSupplier.get())
            .plus(new Transform3d(new Translation3d(Meters.zero(), Meters.zero(), launchHeight),
                Rotation3d.kZero));

        // Get robot's current field-relative linear velocity so projectile inherits it
        ChassisSpeeds fieldSpeeds = this.robotFieldSpeedsSupplier.get();

        // Decompose shooter speed into horizontal and vertical components using hood angle
        double horizontalVel = Math.cos(hoodAngle.in(Radians)) * launchVelocity.in(MetersPerSecond);
        double verticalVel = Math.sin(hoodAngle.in(Radians)) * launchVelocity.in(MetersPerSecond);

        // Convert shooter-relative horizontal speed into field X/Y using turret yaw + robot heading
        double turretPlusHeading =
            turretYaw.plus(launchPose.getRotation().getMeasureZ()).in(Radians);
        double xVel = horizontalVel * Math.cos(turretPlusHeading);
        double yVel = horizontalVel * Math.sin(turretPlusHeading);

        // Add robot's current linear velocity so the launched fuel has correct field frame motion
        xVel += fieldSpeeds.vxMetersPerSecond;
        yVel += fieldSpeeds.vyMetersPerSecond;

        // Spawn the fuel at the computed launch position with the computed velocity
        // If there are fuels in the hopper, prefer to launch one of them so we free its
        // hopper slot and reuse the actual Fuel object (keeps counts and visuals consistent).
        int slotToUse = -1;
        for (int i = HOPPER_CAPACITY - 1; i >= 0; i--) {
            if (Hopper.occupied[i]) {
                slotToUse = i;
                break;
            }
        }

        if (slotToUse >= 0) {
            // Find the fuel object that occupies this slot
            Fuel chosen = null;
            for (Fuel f : fuels) {
                if (f.inHopper && f.indexInHopper == slotToUse) {
                    chosen = f;
                    break;
                }
            }

            if (chosen != null) {
                // Remove from hopper, free slot, and set launched pos/velocity
                chosen.inHopper = false;
                chosen.indexInHopper = -1;
                Hopper.freeIndex(slotToUse);
                chosen.pos = launchPose.getTranslation();
                chosen.vel = new Translation3d(xVel, yVel, verticalVel);
                if (heldFuel > 0)
                    heldFuel--;
                return;
            } else {
                // Occupied but no matching Fuel found (defensive) — just free the slot
                Hopper.freeIndex(slotToUse);
                if (heldFuel > 0)
                    heldFuel--;
            }
        }

        // Fallback: spawn a new fuel if hopper had none available
        // spawnFuel(launchPose.getTranslation(), new Translation3d(xVel, yVel, verticalVel));
    }

    /**
     * Launch a fuel from the hopper if one is available; otherwise spawn a new fuel.
     *
     * <p>
     * This method prefers to reuse an existing Fuel object that is currently marked as in the
     * hopper so the simulator's visual state and indexing remain consistent. If a stored fuel is
     * found it will be removed from the hopper, its slot freed, its pose/velocity set to the
     * provided values, and {@code heldFuel} decremented. If no stored fuel is available this
     * behaves like {@link #spawnFuel}.
     *
     * @param pos launch position (field coordinates)
     * @param vel launch velocity (field coordinates)
     */
    public void launchFromHopper(Translation3d pos, Translation3d vel)
    {
        int slotToUse = -1;
        for (int i = HOPPER_CAPACITY - 1; i >= 0; i--) {
            if (Hopper.occupied[i]) {
                slotToUse = i;
                break;
            }
        }

        if (slotToUse >= 0) {
            Fuel chosen = null;
            for (Fuel f : fuels) {
                if (f.inHopper && f.indexInHopper == slotToUse) {
                    chosen = f;
                    break;
                }
            }

            if (chosen != null) {
                chosen.inHopper = false;
                chosen.indexInHopper = -1;
                Hopper.freeIndex(slotToUse);
                chosen.pos = pos;
                chosen.vel = vel;
                if (heldFuel > 0) {
                    heldFuel--;
                }
                return;
            } else {
                // Defensive: free slot if occupied but no matching fuel found
                Hopper.freeIndex(slotToUse);
                if (heldFuel > 0) {
                    heldFuel--;
                }
            }
        }

        // Fallback: spawn a new fuel if none available in hopper
        spawnFuel(pos, vel);
    }

    /**
     * Set the number of fuels considered "held" in the hopper and reconcile actual simulator state
     * (Fuel objects and occupied slots) to match. This avoids mismatches where external code
     * updates the count but doesn't update stored Fuel objects.
     *
     * @param numFuel desired number of fuels in the hopper (clamped 0..HOPPER_CAPACITY)
     */
    private void setHeldFuel(int numFuel)
    {
        if (numFuel < 0)
            numFuel = 0;
        if (numFuel > HOPPER_CAPACITY)
            numFuel = HOPPER_CAPACITY;

        // Count current in-hopper fuels
        int current = 0;
        for (Fuel f : fuels) {
            if (f.inHopper)
                current++;
        }

        if (numFuel == current) {
            heldFuel = numFuel;
            return;
        }

        if (numFuel < current) {
            // Need to remove (current - numFuel) fuels from hopper: free their slots and
            // mark them as on-field (leave them near robot so they don't immediately
            // disappear). We'll remove the highest-indexed ones first.
            int toRemove = current - numFuel;
            // iterate over a copy to allow modification
            for (Fuel f : fuels) {
                if (toRemove == 0)
                    break;
                if (f.inHopper && f.indexInHopper >= 0) {
                    Hopper.freeIndex(f.indexInHopper);
                    f.inHopper = false;
                    f.indexInHopper = -1;
                    // drop the fuel slightly in front of the robot to make it visible
                    Pose2d robotPose =
                        (robotPoseSupplier != null) ? robotPoseSupplier.get() : new Pose2d();
                    Translation2d forward =
                        new Translation2d(0.2, 0).rotateBy(robotPose.getRotation());
                    f.pos = new Translation3d(robotPose.getX() + forward.getX(),
                        robotPose.getY() + forward.getY(), FUEL_RADIUS);
                    f.vel = Translation3d.kZero;
                    toRemove--;
                }
            }
            heldFuel = numFuel;
            return;
        }

        // n > current: create placeholder fuels in hopper to reach desired count
        int toAdd = numFuel - current;
        Pose2d robotPose = (robotPoseSupplier != null) ? robotPoseSupplier.get() : new Pose2d();
        for (int i = 0; i < toAdd; i++) {
            int slot = Hopper.allocateNextIndex();
            Translation3d rel = Hopper.getRelativePosInHopper(slot);
            Translation2d rel2 =
                new Translation2d(rel.getX(), rel.getY()).rotateBy(robotPose.getRotation());
            double fieldX = robotPose.getX() + rel2.getX();
            double fieldY = robotPose.getY() + rel2.getY();
            double fieldZ = rel.getZ();
            Fuel placeholder = new Fuel(new Translation3d(fieldX, fieldY, fieldZ));
            placeholder.inHopper = true;
            placeholder.indexInHopper = slot;
            placeholder.vel = Translation3d.kZero;
            fuels.add(placeholder);
        }
        heldFuel = numFuel;
    }

    /**
     * Fill the hopper to exactly n fuels (creates placeholder in-hopper Fuel objects if necessary).
     *
     * @param n desired number of fuels in the hopper
     */
    public void fillHopper(int numFuel)
    {
        setHeldFuel(numFuel);
    }

    /**
     * Empty the hopper (free all occupied slots and remove in-hopper Fuel objects).
     */
    public void emptyHopper()
    {
        setHeldFuel(0);
    }

    /**
     * Resolve collision between a fuel and the axis-aligned robot rectangle (in robot frame).
     *
     * <p>
     * The method computes the fuel position relative to the robot, checks whether it intersects the
     * robot bounding box (accounting for the ball radius), applies a minimal translation to push
     * the ball out, and applies velocity impulses based on restitution and robot velocity.
     */
    protected void handleRobotCollision(Fuel fuel, Pose2d robot, Translation2d robotVel)
    {
        Translation2d relativePos = new Pose2d(fuel.pos.toTranslation2d(), Rotation2d.kZero)
            .relativeTo(robot)
            .getTranslation();

        if (fuel.pos.getZ() > bumperHeight)
            return; // above bumpers
        double distanceToBottom = -FUEL_RADIUS - robotLength / 2 - relativePos.getX();
        double distanceToTop = -FUEL_RADIUS - robotLength / 2 + relativePos.getX();
        double distanceToRight = -FUEL_RADIUS - robotWidth / 2 - relativePos.getY();
        double distanceToLeft = -FUEL_RADIUS - robotWidth / 2 + relativePos.getY();

        // not inside robot
        if (distanceToBottom > 0 || distanceToTop > 0 || distanceToRight > 0 || distanceToLeft > 0)
            return;

        Translation2d posOffset;
        // find minimum distance to side and send corresponding collision response
        if ((distanceToBottom >= distanceToTop
            && distanceToBottom >= distanceToRight
            && distanceToBottom >= distanceToLeft)) {
            posOffset = new Translation2d(distanceToBottom, 0);
        } else if ((distanceToTop >= distanceToBottom
            && distanceToTop >= distanceToRight
            && distanceToTop >= distanceToLeft)) {
            posOffset = new Translation2d(-distanceToTop, 0);
        } else if ((distanceToRight >= distanceToBottom
            && distanceToRight >= distanceToTop
            && distanceToRight >= distanceToLeft)) {
            posOffset = new Translation2d(0, distanceToRight);
        } else {
            posOffset = new Translation2d(0, -distanceToLeft);
        }

        // Rotate the collision offset back into field coordinates and apply
        posOffset = posOffset.rotateBy(robot.getRotation());
        fuel.pos = fuel.pos.plus(new Translation3d(posOffset));
        Translation2d normal = posOffset.div(posOffset.getNorm());
        if (fuel.vel.toTranslation2d().dot(normal) < 0)
            fuel.addImpulse(
                new Translation3d(
                    normal.times(-fuel.vel.toTranslation2d().dot(normal) * (1 + ROBOT_COR))));
        if (robotVel.dot(normal) > 0)
            fuel.addImpulse(new Translation3d(normal.times(robotVel.dot(normal))));
    }

    protected void handleRobotCollisions(ArrayList<Fuel> fuels)
    {
        Pose2d robot = robotPoseSupplier.get();
        ChassisSpeeds speeds = robotFieldSpeedsSupplier.get();
        Translation2d robotVel =
            new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        for (Fuel fuel : fuels) {
            handleRobotCollision(fuel, robot, robotVel);
        }
    }

    protected void handleIntakes(ArrayList<Fuel> fuels)
    {
        Pose2d robot = robotPoseSupplier.get();
        for (SimIntake intake : intakes) {
            for (int i = 0; i < fuels.size(); i++) {
                if (intake.shouldIntake(fuels.get(i), robot)) {
                    // Allocate a hopper slot index and mark the fuel as stored.
                    int slot = Hopper.allocateNextIndex();
                    Fuel f = fuels.get(i);
                    f.inHopper = true;
                    f.indexInHopper = slot;
                    // set initial placeholder position at robot XY and hopper floor; the
                    // precise field position will be computed each update in updateHopper()
                    f.pos = new Translation3d(robot.getX(), robot.getY(), HOPPER_FLOOR_HEIGHT);
                    f.vel = Translation3d.kZero;
                    heldFuel++;
                    i--;
                }
            }
        }
    }

    /** Update the state of each fuel that is in the hopper */
    protected void handleHopper(ArrayList<Fuel> fuels)
    {
        Pose2d robot = robotPoseSupplier.get();
        for (int i = 0; i < fuels.size(); i++) {
            fuels.get(i).updateHopper(robot, previousRobot);
        }
        previousRobot = robotPoseSupplier.get();
    }

    /**
     * Collide a fuel with an axis-aligned rectangular prism (defined by start and end). Applies
     * position correction and reflects the appropriate velocity component using the field
     * coefficient of restitution.
     */
    protected static void fuelCollideRectangle(Fuel fuel, Translation3d start, Translation3d end)
    {
        if (fuel.pos.getZ() > end.getZ() + FUEL_RADIUS
            || fuel.pos.getZ() < start.getZ() - FUEL_RADIUS)
            return; // above rectangle
        double distanceToLeft = start.getX() - FUEL_RADIUS - fuel.pos.getX();
        double distanceToRight = fuel.pos.getX() - end.getX() - FUEL_RADIUS;
        double distanceToTop = fuel.pos.getY() - end.getY() - FUEL_RADIUS;
        double distanceToBottom = start.getY() - FUEL_RADIUS - fuel.pos.getY();

        // not inside hub
        if (distanceToLeft > 0 || distanceToRight > 0 || distanceToTop > 0 || distanceToBottom > 0)
            return;

        Translation2d collision;
        // find minimum distance to side and send corresponding collision response
        if (fuel.pos.getX() < start.getX()
            || (distanceToLeft >= distanceToRight
                && distanceToLeft >= distanceToTop
                && distanceToLeft >= distanceToBottom)) {
            collision = new Translation2d(distanceToLeft, 0);
        } else if (fuel.pos.getX() >= end.getX()
            || (distanceToRight >= distanceToLeft
                && distanceToRight >= distanceToTop
                && distanceToRight >= distanceToBottom)) {
            collision = new Translation2d(-distanceToRight, 0);
        } else if (fuel.pos.getY() > end.getY()
            || (distanceToTop >= distanceToLeft
                && distanceToTop >= distanceToRight
                && distanceToTop >= distanceToBottom)) {
            collision = new Translation2d(0, -distanceToTop);
        } else {
            collision = new Translation2d(0, distanceToBottom);
        }

        if (collision.getX() != 0) {
            fuel.pos = fuel.pos.plus(new Translation3d(collision));
            fuel.vel = fuel.vel.plus(new Translation3d(-(1 + FIELD_COR) * fuel.vel.getX(), 0, 0));
        } else if (collision.getY() != 0) {
            fuel.pos = fuel.pos.plus(new Translation3d(collision));
            fuel.vel = fuel.vel.plus(new Translation3d(0, -(1 + FIELD_COR) * fuel.vel.getY(), 0));
        }
    }

    /**
     * Registers an intake with the fuel simulator. This intake will remove fuel from the field
     * based on the `ableToIntake` parameter.
     * 
     * @param xMin Minimum x position for the bounding box
     * @param xMax Maximum x position for the bounding box
     * @param yMin Minimum y position for the bounding box
     * @param yMax Maximum y position for the bounding box
     * @param ableToIntake Should a return a boolean whether the intake is active
     * @param intakeCallback Function to call when a fuel is intaked
     */
    public void registerIntake(
        double xMin, double xMax, double yMin, double yMax, BooleanSupplier ableToIntake,
        Runnable intakeCallback)
    {
        intakes.add(new SimIntake(xMin, xMax, yMin, yMax, ableToIntake, intakeCallback));
    }

    /**
     * Registers an intake with the fuel simulator. This intake will remove fuel from the field
     * based on the `ableToIntake` parameter.
     * 
     * @param xMin Minimum x position for the bounding box
     * @param xMax Maximum x position for the bounding box
     * @param yMin Minimum y position for the bounding box
     * @param yMax Maximum y position for the bounding box
     * @param ableToIntake Should a return a boolean whether the intake is active
     */
    public void registerIntake(double xMin, double xMax, double yMin, double yMax,
        BooleanSupplier ableToIntake)
    {
        registerIntake(xMin, xMax, yMin, yMax, ableToIntake, () -> {
        });
    }

    /**
     * Registers an intake with the fuel simulator. This intake will always remove fuel from the
     * field.
     * 
     * @param xMin Minimum x position for the bounding box
     * @param xMax Maximum x position for the bounding box
     * @param yMin Minimum y position for the bounding box
     * @param yMax Maximum y position for the bounding box
     * @param intakeCallback Function to call when a fuel is intaked
     */
    public void registerIntake(double xMin, double xMax, double yMin, double yMax,
        Runnable intakeCallback)
    {
        registerIntake(xMin, xMax, yMin, yMax, () -> true, intakeCallback);
    }

    /**
     * Registers an intake with the fuel simulator. This intake will always remove fuel from the
     * field.
     * 
     * @param xMin Minimum x position for the bounding box
     * @param xMax Maximum x position for the bounding box
     * @param yMin Minimum y position for the bounding box
     * @param yMax Maximum y position for the bounding box
     */
    public void registerIntake(double xMin, double xMax, double yMin, double yMax)
    {
        registerIntake(xMin, xMax, yMin, yMax, () -> true, () -> {
        });
    }

    public static class Hub {
        public static final Hub BLUE_HUB =
            new Hub(new Translation2d(4.61, FIELD_WIDTH / 2),
                new Translation3d(5.3, FIELD_WIDTH / 2, 0.89), 1);
        public static final Hub RED_HUB = new Hub(
            new Translation2d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2),
            new Translation3d(FIELD_LENGTH - 5.3, FIELD_WIDTH / 2, 0.89),
            -1);

        protected static final double ENTRY_HEIGHT = 1.83;
        protected static final double ENTRY_RADIUS = 0.56;

        protected static final double SIDE = 1.2;

        protected static final double NET_HEIGHT_MAX = 3.057;
        protected static final double NET_HEIGHT_MIN = 1.5;
        protected static final double NET_OFFSET = SIDE / 2 + 0.261;
        protected static final double NET_WIDTH = 1.484;

        protected final Translation2d center;
        protected final Translation3d exit;
        protected final int exitVelXMult;

        protected int score = 0;

        protected Hub(Translation2d center, Translation3d exit, int exitVelXMult)
        {
            this.center = center;
            this.exit = exit;
            this.exitVelXMult = exitVelXMult;
        }

        protected void handleHubInteraction(Fuel fuel, int subticks)
        {
            if (didFuelScore(fuel, subticks)) {
                fuel.pos = exit;
                fuel.vel = getDispersalVelocity();
                score++;
            }
        }

        protected boolean didFuelScore(Fuel fuel, int subticks)
        {
            return fuel.pos.toTranslation2d().getDistance(center) <= ENTRY_RADIUS
                && fuel.pos.getZ() <= ENTRY_HEIGHT
                && fuel.pos.minus(fuel.vel.times(PERIOD / subticks)).getZ() > ENTRY_HEIGHT;
        }

        protected Translation3d getDispersalVelocity()
        {
            return new Translation3d(exitVelXMult * (Math.random() + 0.1) * 1.5,
                Math.random() * 2 - 1, 0);
        }

        /**
         * Reset this hub's score to 0
         */
        public void resetScore()
        {
            score = 0;
        }

        /**
         * Get the current count of fuel scored in this hub
         *
         * @return the number of fuel pieces scored in this hub
         */
        public int getScore()
        {
            return score;
        }

        protected void fuelCollideSide(Fuel fuel)
        {
            fuelCollideRectangle(
                fuel,
                new Translation3d(center.getX() - SIDE / 2, center.getY() - SIDE / 2, 0),
                new Translation3d(center.getX() + SIDE / 2, center.getY() + SIDE / 2,
                    ENTRY_HEIGHT - 0.1));
        }

        protected double fuelHitNet(Fuel fuel)
        {
            if (fuel.pos.getZ() > NET_HEIGHT_MAX || fuel.pos.getZ() < NET_HEIGHT_MIN)
                return 0;
            if (fuel.pos.getY() > center.getY() + NET_WIDTH / 2
                || fuel.pos.getY() < center.getY() - NET_WIDTH / 2)
                return 0;
            if (fuel.pos.getX() > center.getX() + NET_OFFSET * exitVelXMult) {
                return Math.max(0,
                    center.getX() + NET_OFFSET * exitVelXMult - (fuel.pos.getX() - FUEL_RADIUS));
            } else {
                return Math.min(0,
                    center.getX() + NET_OFFSET * exitVelXMult - (fuel.pos.getX() + FUEL_RADIUS));
            }
        }
    }

    protected class SimIntake {
        double xMin, xMax, yMin, yMax;
        BooleanSupplier ableToIntake;
        Runnable callback;

        protected SimIntake(
            double xMin,
            double xMax,
            double yMin,
            double yMax,
            BooleanSupplier ableToIntake,
            Runnable intakeCallback)
        {
            this.xMin = xMin;
            this.xMax = xMax;
            this.yMin = yMin;
            this.yMax = yMax;
            this.ableToIntake = ableToIntake;
            this.callback = intakeCallback;
        }

        protected boolean shouldIntake(Fuel fuel, Pose2d robotPose)
        {
            if (!ableToIntake.getAsBoolean() || fuel.pos.getZ() > bumperHeight
                || heldFuel >= HOPPER_CAPACITY)
                return false;

            Translation2d fuelRelativePos = new Pose2d(fuel.pos.toTranslation2d(), Rotation2d.kZero)
                .relativeTo(robotPose)
                .getTranslation();

            boolean result = fuelRelativePos.getX() >= xMin
                && fuelRelativePos.getX() <= xMax
                && fuelRelativePos.getY() >= yMin
                && fuelRelativePos.getY() <= yMax;
            if (result) {
                callback.run();
            }
            return result;
        }
    }

    /** Utility class to managing a fuel's position in the hopper based on an index */
    protected static class Hopper {

        protected static final int PER_LAYER = 14; // 3+4+3+4 layout
        protected static final double SPACING = 0.152; // spacing between ball centers (m)
        // Small gap between layers - may remove, TBD
        protected static final double LAYER_HEIGHT = (FUEL_RADIUS * 2) + 0.01;
        protected static final double X_OFFSET = 0.35;

        // Stores a robot-relative position of a fuel that is in the hopper based on an index
        protected static final HashMap<Integer, Translation3d> hopperMap =
            new HashMap<Integer, Translation3d>();

        // Tracks whether a given hopper index is currently occupied by a stored fuel.
        // This separates occupancy from the position cache (hopperMap) so cached
        // positions don't imply the slot is taken.
        protected static final boolean[] occupied = new boolean[HOPPER_CAPACITY];

        /**
         * Get a robot-relative position for the given hopper index. The hopper is arranged in
         * layers of 14 balls (row pattern 3,4,3,4). This method returns a consistent translation
         * for a given index and caches the result in {@code hopperMap} so repeated calls return the
         * same position.
         *
         * @param index zero-based hopper slot index
         * @return robot-relative Translation3d for the slot
         */
        protected static Translation3d getRelativePosInHopper(int index)
        {
            if (hopperMap.containsKey(index)) {
                return hopperMap.get(index);
            }

            int layer = index / PER_LAYER;
            int idxInLayer = index % PER_LAYER;

            // row counts and their x-offset multipliers (front -> back)
            int[] rowCounts = {3, 4, 3, 4};
            double[] rowXMultipliers = {0, -1, -2, -3};

            int running = 0;
            int row = 0;
            // Find which row within the 4-row pattern contains idxInLayer.
            // `running` accumulates the sizes of previous rows; when
            // idxInLayer is less than running + rowCounts[row], we've found the row.
            for (; row < rowCounts.length; row++) {
                if (idxInLayer < running + rowCounts[row])
                    break;
                running += rowCounts[row];
            }
            int indexInRow = idxInLayer - running;

            // compute x (forward/back) based on row
            double x = rowXMultipliers[row] * SPACING + X_OFFSET;

            // compute y (left/right) centered about robot center
            int cols = rowCounts[row];
            double y = (-(cols - 1) / 2.0 + indexInRow) * SPACING;

            // compute z based on layer (floor + stacked layers)
            double z = HOPPER_FLOOR_HEIGHT + layer * LAYER_HEIGHT;

            Translation3d pos = new Translation3d(x, y, z);
            hopperMap.put(index, pos);
            return pos;
        }

        /**
         * Convenience: return the next available slot position (first unused index). Caches the
         * computed position in {@code hopperMap}.
         *
         * @return robot-relative Translation3d for the next free hopper slot
         */
        protected static Translation3d getRelativePosInHopper()
        {
            int i = 0;
            // find first unoccupied slot
            while (i < HOPPER_CAPACITY && occupied[i]) {
                i++;
            }
            if (i >= HOPPER_CAPACITY) {
                // fallback to last slot if full
                i = HOPPER_CAPACITY - 1;
            }
            return getRelativePosInHopper(i);
        }

        /**
         * Allocate and return the next hopper index. The index is cached and can be used to place
         * fuel into the hopper with a consistent robot-relative position.
         *
         * @return allocated zero-based hopper index
         */
        protected static int allocateNextIndex()
        {
            int i = 0;
            // find first free index
            while (i < HOPPER_CAPACITY && occupied[i]) {
                i++;
            }
            if (i >= HOPPER_CAPACITY) {
                // if full, fallback to last slot
                i = HOPPER_CAPACITY - 1;
            }
            // mark occupied and ensure a cached robot-relative position exists
            occupied[i] = true;
            getRelativePosInHopper(i);
            return i;
        }

        /**
         * Free a previously allocated hopper index so it can be reused by future allocations. Safe
         * to call multiple times for the same index.
         *
         * @param index zero-based hopper slot index to free
         */
        protected static void freeIndex(int index)
        {
            if (index >= 0 && index < HOPPER_CAPACITY) {
                occupied[index] = false;
            }
        }
    }

    /**
     * Calculates the launch velocity vector for fuel based on shooter parameters and robot state
     * 
     * @param vel Linear velocity of the shooter
     * @param angle Launch angle of the shooter
     * @return 3D velocity vector in field coordinates
     */
    public Translation3d launchVel(LinearVelocity vel, Angle angle)
    {
        Pose3d robot = new Pose3d(RobotState.getInstance().getEstimatedPose());
        ChassisSpeeds fieldSpeeds = RobotState.getInstance().getFieldRelativeVelocity();

        double horizontalVel = Math.cos(angle.in(Radians)) * vel.in(MetersPerSecond);
        double verticalVel = Math.sin(angle.in(Radians)) * vel.in(MetersPerSecond);
        double xVel =
            horizontalVel * Math.cos(robot.getRotation().toRotation2d().getRadians());
        double yVel =
            horizontalVel * Math.sin(robot.getRotation().toRotation2d().getRadians());

        xVel += fieldSpeeds.vxMetersPerSecond;
        yVel += fieldSpeeds.vyMetersPerSecond;

        return new Translation3d(xVel, yVel, verticalVel);
    }
}
