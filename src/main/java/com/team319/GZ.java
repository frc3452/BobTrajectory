package com.team319;

import com.team254.lib.trajectory.WaypointSequence.Waypoint;

public class GZ {
        // Field and robot measurements
        public static class Measurements {
                public static Measurement robotWidth = new Measurement(26.5);
                public static Measurement robotLength = new Measurement(34);

                public static final Measurement FIELD_EDGE_FLAIR = new Measurement(29.69);
                public static final Measurement FIELD_DRIVER_WALL = new Measurement(264);
                public static final Measurement FIELD_WIDTH = new Measurement(
                                FIELD_DRIVER_WALL.in() + FIELD_EDGE_FLAIR.in() * 2);

                public static final Measurement FIELD_DRIVER_WALL_TO_SWITCH = new Measurement(140);
                public static final Measurement FIELD_WALL_TO_SIDE_OF_SWITCH = new Measurement(85.25);
                public static final Measurement FIELD_DRIVER_WALL_TO_CLOSEST_PYRAMID_CUBE = new Measurement(98);

                public static final Measurement FIELD_CENTER = new Measurement(FIELD_WIDTH.in_2());
                public static final Measurement FIELD_DRIVER_WALL_TO_HALF_BETWEEN_SWITCH_AND_PLATFORM = new Measurement(
                                226.73500);

                public static final Measurement FIELD_SWITCH_WIDTH = new Measurement(56);

                public static final Measurement FIELD_WALL_TO_EDGE_OF_SCALE_BUCKET = new Measurement(71.57);

                public static final Measurement FIELD_DRIVER_WALL_TO_SCALE_BUCKET = new Measurement(299.65);

                public static final Measurement _switch_displacement = new Measurement(3);
        }

        // Starting positions
        public static class StartingPositions {
                public static Waypoint kLeftStartingPoint = new Waypoint(
                                Measurements.robotLength.f_2(), Measurements.FIELD_DRIVER_WALL.f()
                                                + Measurements.FIELD_EDGE_FLAIR.f() - Measurements.robotWidth.f_2(),
                                0, 0, 0);

                public static Waypoint kRightStartingPoint = new Waypoint(Measurements.robotLength.f_2(),
                                Measurements.FIELD_EDGE_FLAIR.f() + (Measurements.robotWidth.f_2()), 0, 0, 0);

                public static Waypoint kMiddleStartingPoint = new Waypoint(Measurements.robotLength.f_2(),
                                Measurements.FIELD_CENTER.f() + 1 - Measurements.robotWidth.f_2(), 0, 0, 0);

        }

        // Waypoints with non-valid theta and velocities
        public static class Points {
                public static final Waypoint mCubeNearestDriverWall = new Waypoint(
                                Measurements.FIELD_DRIVER_WALL_TO_CLOSEST_PYRAMID_CUBE.f(),
                                Measurements.FIELD_CENTER.f(), 0, 0, 0);

                public static final Waypoint mCubePlatformZone1LeftScale = new Waypoint(
                                Measurements.FIELD_DRIVER_WALL_TO_SWITCH.f() + Measurements.FIELD_SWITCH_WIDTH.f() + 1,
                                Measurements.FIELD_WIDTH.f() - Measurements.FIELD_WALL_TO_SIDE_OF_SWITCH.f() - .5, 0, 0,
                                0);

        }

        // Waypoints with valid thetas and nonvalid velocities
        public static class Waypoints {

                public static final Waypoint mSwitchTurnPointLeft = new Waypoint(
                                StartingPositions.kMiddleStartingPoint.x + 2.6, Measurements.FIELD_CENTER.f(), 0, 0, 0);

                public static final Waypoint mSwitchTurnPointRight = new Waypoint(
                                StartingPositions.kMiddleStartingPoint.x + 2.6, Measurements.FIELD_CENTER.f(), 0, 0, 0);

                public static final Waypoint mLeftSwitchFrontPlace = new Waypoint(
                                Measurements.FIELD_DRIVER_WALL_TO_SWITCH.f() - Measurements.robotLength.f_2()
                                                - Measurements._switch_displacement.f(),
                                Measurements.FIELD_WIDTH.f() - Measurements.FIELD_WALL_TO_SIDE_OF_SWITCH.f()
                                                - Measurements.robotWidth.f(),
                                0, 0, 0);

                public static final Waypoint mRightSwitchFrontPlace = new Waypoint(
                                Measurements.FIELD_DRIVER_WALL_TO_SWITCH.f() - Measurements.robotLength.f_2()
                                                - Measurements._switch_displacement.f(),
                                Measurements.FIELD_WALL_TO_SIDE_OF_SWITCH.f() + Measurements.robotWidth.f(), 0, 0, 0);

                public static double scaleFactor = Math.sin(Math.toRadians(45) * Measurements.robotLength.f_2());
                public static final Waypoint mLeftScaleCorner = new Waypoint(
                                Measurements.FIELD_DRIVER_WALL_TO_SCALE_BUCKET.f() - scaleFactor,
                                Measurements.FIELD_WIDTH.f() - Measurements.FIELD_WALL_TO_EDGE_OF_SCALE_BUCKET.f()
                                                + Measurements.robotWidth.f_2() - .5,
                                Math.toRadians(-35), 0, 0);

                public static final Waypoint mLeftScaleTurnAround = new Waypoint(mLeftScaleCorner.x - 3,
                                mLeftScaleCorner.y + 2, Math.toRadians(-90), 0, 0);

                public static final Waypoint mCubePlatformZone1LeftScale = new Waypoint(
                                Points.mCubePlatformZone1LeftScale.x + Measurements.robotLength.f_2(),
                                Points.mCubePlatformZone1LeftScale.y + Measurements.robotWidth.f_2(), Math.toRadians(-90-55), 0, 0);

        }
}