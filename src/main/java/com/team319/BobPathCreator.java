package com.team319;

import static java.util.Arrays.asList;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.trajectory.WaypointSequence.Waypoint;
import com.team319.trajectory.AbstractBobPathCreator;
import com.team319.trajectory.BobPath;
import com.team319.trajectory.SrxTranslatorConfig;

public class BobPathCreator extends AbstractBobPathCreator {

	private static Measurement robotWidth = new Measurement(26.5);
	private static Measurement robotLength = new Measurement(34);

	private SrxTranslatorConfig config = new SrxTranslatorConfig();

	private BobPathCreator() {
		config.max_acc = 8.0; // Maximum acceleration in FPS
		config.max_vel = 10.0; // Maximum velocity in FPS
		config.wheel_dia_inches = 6;
		config.scale_factor = 1.0; // Used to adjust for a gear ratio and or distance tuning
		config.encoder_ticks_per_rev = 4096; // Count of ticks on your encoder
		config.robotLength = robotLength.in(); // Robot length in inches, used for drawing the robot
		config.robotWidth = robotWidth.in(); // Robot width in inches, used for drawing the robot
		config.highGear = true;
	}

	protected List<BobPath> getArcs() {
		List<BobPath> paths = new ArrayList<>();
		
		paths.addAll(getBaselines());

		paths.addAll(getMiddleSwitch());
		
		paths.addAll(getLeftSwitch());
		paths.addAll(getRightSwitch());
		
		return paths;
	}

	/** MEASUREMENTS */
	private static final Measurement FIELD_EDGE_FLAIR = new Measurement(29.69);
	private static final Measurement FIELD_DRIVER_WALL = new Measurement(264);
	private static final Measurement FIELD_WIDTH = new Measurement(FIELD_DRIVER_WALL.in() + FIELD_EDGE_FLAIR.in() * 2);

	private static final Measurement FIELD_DRIVER_WALL_TO_SWITCH = new Measurement(140);
	private static final Measurement FIELD_WALL_TO_SIDE_OF_SWITCH = new Measurement(85.25);
	private static final Measurement FIELD_DRIVER_WALL_TO_CLOSEST_PYRAMID_CUBE = new Measurement(98);

	private static final Measurement FIELD_CENTER = new Measurement(FIELD_WIDTH.in_2());
	private static final Measurement FIELD_DRIVER_WALL_TO_HALF_BETWEEN_SWITCH_AND_PLATFORM = new Measurement(226.73500);
	
	private static final Measurement FIELD_SWITCH_WIDTH = new Measurement(56);
	
	
	private static final Measurement _switch_displacement = new Measurement(3);


	/**STARTING POSITIONS */
	private static Waypoint kLeftStartingPoint = new Waypoint(robotLength.f_2(),
			FIELD_DRIVER_WALL.f() + FIELD_EDGE_FLAIR.f() - robotWidth.f_2(), 0, 0, 0);

	private static Waypoint kRightStartingPoint = new Waypoint(robotLength.f_2(),
			FIELD_EDGE_FLAIR.f() + (robotWidth.f_2()), 0, 0, 0);

	private static Waypoint kMiddleStartingPoint = new Waypoint(robotLength.f_2(), FIELD_CENTER.f()+1-robotWidth.f_2(), 0, 0, 0);


	/** PATH WAYPOINTS */
	private static final Waypoint mSwitchTurnPointLeft = new Waypoint(kMiddleStartingPoint.x + 2.6, FIELD_CENTER.f(), 0, 0,
			0);

	private static final Waypoint mSwitchTurnPointRight = new Waypoint(kMiddleStartingPoint.x + 2.6, FIELD_CENTER.f(), 0, 0, 0);

	private static final Waypoint mCubeNearestDriverWall = new Waypoint(FIELD_DRIVER_WALL_TO_CLOSEST_PYRAMID_CUBE.f(),
			FIELD_CENTER.f(), 0, 0, 0);


	private static final Waypoint mLeftSwitchFrontPlace = new Waypoint(
			FIELD_DRIVER_WALL_TO_SWITCH.f() - robotLength.f_2() - _switch_displacement.f(),
			FIELD_WIDTH.f() - FIELD_WALL_TO_SIDE_OF_SWITCH.f() - robotWidth.f(), 0, 0, 0);

	private static final Waypoint mRightSwitchFrontPlace = new Waypoint(
			FIELD_DRIVER_WALL_TO_SWITCH.f() - robotLength.f_2() - _switch_displacement.f(),
			FIELD_WALL_TO_SIDE_OF_SWITCH.f() + robotWidth.f(), 0, 0, 0);


	

	private List<BobPath> getMiddleSwitch() {
		/** SWITCH LEFT */
		BobPath switch_left = new BobPath(config, "Middle_Left_Switch");
		switch_left.addWaypoint(kMiddleStartingPoint);
		switch_left.addWaypoint(mLeftSwitchFrontPlace, 0, 3);

		switch_left.addWaypoint(mSwitchTurnPointLeft, 0, 3);

		switch_left.addWaypoint(mCubeNearestDriverWall.x - robotLength.f_2(), mCubeNearestDriverWall.y, 0,
				0, 3);

		switch_left.addWaypoint(mSwitchTurnPointLeft, 0, 3);

		switch_left.addWaypoint(mLeftSwitchFrontPlace, 0, 3);

		/** SWITCH RIGHT */
		BobPath switch_right = new BobPath(config, "Middle_Right_Switch");
		switch_right.addWaypoint(kMiddleStartingPoint);

		switch_right.addWaypoint(mRightSwitchFrontPlace, 0, 3);
		switch_right.addWaypoint(mSwitchTurnPointRight, 0, 3);

		switch_right.addWaypoint(mCubeNearestDriverWall.x - robotLength.f_2(), mCubeNearestDriverWall.y, 0,
		0, 3);

		switch_right.addWaypoint(mSwitchTurnPointRight, 0, 3);

		switch_right.addWaypoint(mRightSwitchFrontPlace, 0, 3);

		return asList(switch_left, switch_right);
	}

	private List<BobPath> getBaselines()
	{
		BobPath middle = new BobPath(config, "Baseline_Middle");
		middle.addWaypoint(kMiddleStartingPoint);
		middle.addWaypointRelative(5, 2, 89.99, 1, 3);
		middle.addWaypointRelative(-3, 3, 89.99, 0, 3);
		middle.addWaypoint(mLeftSwitchFrontPlace.x,mLeftSwitchFrontPlace.y, 179.99, 0,3);

		BobPath left = new BobPath(config, "Baseline_Left");
		left.addWaypoint(kLeftStartingPoint);
		left.addWaypointRelative(FIELD_DRIVER_WALL_TO_SWITCH.f(), 0, 0);


		BobPath right = new BobPath(config, "Baseline_Right");
		right.addWaypoint(kRightStartingPoint);
		right.addWaypointRelative(FIELD_DRIVER_WALL_TO_SWITCH.f(), 0, 0);

		return asList(middle, left, right);
	}

	private List<BobPath> getLeftSwitch()
	{
		BobPath left = new BobPath(config, "Left_Left_Switch");
		left.addWaypoint(kLeftStartingPoint);
		left.addWaypointRelative(FIELD_DRIVER_WALL_TO_SWITCH.f() - 2.5, 1, 0, 2.5, 3);
		left.addWaypoint(FIELD_DRIVER_WALL_TO_SWITCH.f() + 2.5, FIELD_WIDTH.f() - FIELD_WALL_TO_SIDE_OF_SWITCH.f() + robotLength.f_2() + _switch_displacement.f(), -89.99, 0, 3);

		BobPath right = new BobPath(config, "Left_Right_Switch");
		right.addWaypoint(kLeftStartingPoint);
		right.addWaypoint(FIELD_DRIVER_WALL_TO_HALF_BETWEEN_SWITCH_AND_PLATFORM.f() * 2/3, kLeftStartingPoint.y + 1, 0, 3, 3);
		right.addWaypoint(robotWidth.f_2() + FIELD_DRIVER_WALL_TO_HALF_BETWEEN_SWITCH_AND_PLATFORM.f(), kLeftStartingPoint.y - 5, -89.99, 3, 3);
		right.addWaypoint(robotWidth.f_2() + FIELD_DRIVER_WALL_TO_HALF_BETWEEN_SWITCH_AND_PLATFORM.f(), ((kLeftStartingPoint.y - 5) * 2/3), -89.99, 3, 3);
		right.addWaypoint(FIELD_DRIVER_WALL_TO_SWITCH.f() + FIELD_SWITCH_WIDTH.f() + robotWidth.f() + .5 , FIELD_WALL_TO_SIDE_OF_SWITCH.f() + 2, -170.99, 0 ,3);

		return asList(left, right);
	}

	private List<BobPath> getRightSwitch()
	{
		BobPath right = new BobPath(config, "Right_Right_Switch");
		right.addWaypoint(kRightStartingPoint);
		right.addWaypointRelative(FIELD_DRIVER_WALL_TO_SWITCH.f() - 2.5, -1, 0, 2.5, 3);
		right.addWaypoint(FIELD_DRIVER_WALL_TO_SWITCH.f() + 2.5, FIELD_WALL_TO_SIDE_OF_SWITCH.f() - robotLength.f_2() - _switch_displacement.f(), 89.99, 0, 3);

		BobPath left = new BobPath(config, "Right_Left_Switch");
		left.addWaypoint(kRightStartingPoint);
		left.addWaypoint(FIELD_DRIVER_WALL_TO_HALF_BETWEEN_SWITCH_AND_PLATFORM.f() * 2/3, kRightStartingPoint.y - 1, 0, 3, 3);
		left.addWaypoint(robotWidth.f_2() + FIELD_DRIVER_WALL_TO_HALF_BETWEEN_SWITCH_AND_PLATFORM.f(), kRightStartingPoint.y + 5, 89.99, 3, 3);
		left.addWaypoint(robotWidth.f_2() + FIELD_DRIVER_WALL_TO_HALF_BETWEEN_SWITCH_AND_PLATFORM.f(), ((kRightStartingPoint.y + 5) * (1+(2.0/3))), 89.99, 3, 3);
		left.addWaypoint(FIELD_DRIVER_WALL_TO_SWITCH.f() + FIELD_SWITCH_WIDTH.f() + robotWidth.f() + .5 , FIELD_WIDTH.f() - FIELD_WALL_TO_SIDE_OF_SWITCH.f() - 2, 170.99, 0 ,3);

		return asList(right,left);
	}

	public static void main(String[] args) {
		new BobPathCreator().generatePaths();
	}
}