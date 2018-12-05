package com.team319;

import static java.util.Arrays.asList;

import java.util.ArrayList;
import java.util.List;

import com.team319.GZ.Measurements;
import com.team319.GZ.Points;
import com.team319.GZ.StartingPositions;
import com.team319.GZ.Waypoints;
import com.team319.trajectory.AbstractBobPathCreator;
import com.team319.trajectory.BobPath;
import com.team319.trajectory.SrxTranslatorConfig;

public class BobPathCreator extends AbstractBobPathCreator {

	private SrxTranslatorConfig config = new SrxTranslatorConfig();

	private BobPathCreator() {
		config.max_acc = 8.0; // Maximum acceleration in FPS
		config.max_vel = 10.0; // Maximum velocity in FPS
		config.wheel_dia_inches = 6;
		config.scale_factor = 1.0; // Used to adjust for a gear ratio and or distance tuning
		config.encoder_ticks_per_rev = 4096; // Count of ticks on your encoder
		config.robotLength = Measurements.robotLength.in(); // Robot length in inches, used for drawing the robot
		config.robotWidth = Measurements.robotWidth.in(); // Robot width in inches, used for drawing the robot
		config.highGear = true;
	}

	protected List<BobPath> getArcs() {
		List<BobPath> paths = new ArrayList<>();

		// paths.addAll(getBaselines());

		// paths.addAll(getMiddleSwitch());

		// paths.addAll(getLeftSwitch());
		// paths.addAll(getRightSwitch());

		paths.addAll(getLeftScale());

		return paths;
	}

	private List<BobPath> getMiddleSwitch() {
		/** SWITCH LEFT */
		BobPath switch_left = new BobPath(config, "Middle_Left_Switch");
		switch_left.addWaypoint(StartingPositions.kMiddleStartingPoint);
		switch_left.addWaypoint(Waypoints.mLeftSwitchFrontPlace, 0, 3);

		switch_left.addWaypoint(Waypoints.mSwitchTurnPointLeft, 0, 3);

		switch_left.addWaypoint(Points.mCubeNearestDriverWall.x - Measurements.robotLength.f_2(),
				Points.mCubeNearestDriverWall.y, 0, 0, 3);

		switch_left.addWaypoint(Waypoints.mSwitchTurnPointLeft, 0, 3);

		switch_left.addWaypoint(Waypoints.mLeftSwitchFrontPlace, 0, 3);

		/** SWITCH RIGHT */
		BobPath switch_right = new BobPath(config, "Middle_Right_Switch");
		switch_right.addWaypoint(StartingPositions.kMiddleStartingPoint);

		switch_right.addWaypoint(Waypoints.mRightSwitchFrontPlace, 0, 3);
		switch_right.addWaypoint(Waypoints.mSwitchTurnPointRight, 0, 3);

		switch_right.addWaypoint(Points.mCubeNearestDriverWall.x - Measurements.robotLength.f_2(),
				Points.mCubeNearestDriverWall.y, 0, 0, 3);

		switch_right.addWaypoint(Waypoints.mSwitchTurnPointRight, 0, 3);

		switch_right.addWaypoint(Waypoints.mRightSwitchFrontPlace, 0, 3);

		switch_left.print();
		switch_right.print();

		return asList(switch_left, switch_right);
	}

	private List<BobPath> getBaselines() {
		BobPath middle = new BobPath(config, "Baseline_Middle");
		middle.addWaypoint(StartingPositions.kMiddleStartingPoint);
		middle.addWaypointRelative(5, 2, 89.99, 1, 3);
		middle.addWaypointRelative(-3, 3, 89.99, 0, 3);
		middle.addWaypoint(Waypoints.mLeftSwitchFrontPlace.x, Waypoints.mLeftSwitchFrontPlace.y, 179.99, 0, 3);

		BobPath left = new BobPath(config, "Baseline_Left");
		left.addWaypoint(StartingPositions.kLeftStartingPoint);
		left.addWaypointRelative(Measurements.FIELD_DRIVER_WALL_TO_SWITCH.f(), 0, 0, 0, 3);

		BobPath right = new BobPath(config, "Baseline_Right");
		right.addWaypoint(StartingPositions.kRightStartingPoint);
		right.addWaypointRelative(Measurements.FIELD_DRIVER_WALL_TO_SWITCH.f(), 0, 0);

		middle.print();
		left.print();
		right.print();

		return asList(middle, left, right);
	}

	private List<BobPath> getLeftSwitch() {
		BobPath left = new BobPath(config, "Left_Left_Switch");
		left.addWaypoint(StartingPositions.kLeftStartingPoint);
		left.addWaypointRelative(Measurements.FIELD_DRIVER_WALL_TO_SWITCH.f() - 2.5, 1, 0, 2.5, 3);
		left.addWaypoint(Measurements.FIELD_DRIVER_WALL_TO_SWITCH.f() + 2.5,
				Measurements.FIELD_WIDTH.f() - Measurements.FIELD_WALL_TO_SIDE_OF_SWITCH.f()
						+ Measurements.robotLength.f_2() + Measurements._switch_displacement.f(),
				-89.99, 0, 3);

		BobPath right = new BobPath(config, "Left_Right_Switch");
		right.addWaypoint(StartingPositions.kLeftStartingPoint);
		right.addWaypoint(Measurements.FIELD_DRIVER_WALL_TO_HALF_BETWEEN_SWITCH_AND_PLATFORM.f() * (2.0 / 3),
				StartingPositions.kLeftStartingPoint.y + 1, 0, 3, 3);

		right.addWaypoint(
				Measurements.robotWidth.f_2() + Measurements.FIELD_DRIVER_WALL_TO_HALF_BETWEEN_SWITCH_AND_PLATFORM.f(),
				StartingPositions.kLeftStartingPoint.y - 5, -89.99, 3, 3);
		right.addWaypoint(
				Measurements.robotWidth.f_2() + Measurements.FIELD_DRIVER_WALL_TO_HALF_BETWEEN_SWITCH_AND_PLATFORM.f(),
				((StartingPositions.kLeftStartingPoint.y - 5) * (2.0 / 3)), -89.99, 3, 3);
		right.addWaypoint(
				Measurements.FIELD_DRIVER_WALL_TO_SWITCH.f() + Measurements.FIELD_SWITCH_WIDTH.f()
						+ Measurements.robotWidth.f() + .5,
				Measurements.FIELD_WALL_TO_SIDE_OF_SWITCH.f() + 2, -170.99, 0, 3);

		left.print();
		right.print();
		return asList(left, right);
	}

	private List<BobPath> getRightSwitch() {
		BobPath right = new BobPath(config, "Right_Right_Switch");
		right.addWaypoint(StartingPositions.kRightStartingPoint);
		right.addWaypointRelative(Measurements.FIELD_DRIVER_WALL_TO_SWITCH.f() - 2.5, -1, 0, 2.5, 3);
		right.addWaypoint(Measurements.FIELD_DRIVER_WALL_TO_SWITCH.f() + 2.5,
				Measurements.FIELD_WALL_TO_SIDE_OF_SWITCH.f() - Measurements.robotLength.f_2()
						- Measurements._switch_displacement.f(),
				89.99, 0, 3);

		BobPath left = new BobPath(config, "Right_Left_Switch");
		left.addWaypoint(StartingPositions.kRightStartingPoint);
		left.addWaypoint(Measurements.FIELD_DRIVER_WALL_TO_HALF_BETWEEN_SWITCH_AND_PLATFORM.f() * (2.0 / 3),
				StartingPositions.kRightStartingPoint.y - 1, 0, 3, 3);
		left.addWaypoint(
				Measurements.robotWidth.f_2() + Measurements.FIELD_DRIVER_WALL_TO_HALF_BETWEEN_SWITCH_AND_PLATFORM.f(),
				StartingPositions.kRightStartingPoint.y + 5, 89.99, 3, 3);
		left.addWaypoint(
				Measurements.robotWidth.f_2() + Measurements.FIELD_DRIVER_WALL_TO_HALF_BETWEEN_SWITCH_AND_PLATFORM.f(),
				((StartingPositions.kRightStartingPoint.y + 5) * (1 + (2.0 / 3))), 89.99, 3, 3);
		left.addWaypoint(
				Measurements.FIELD_DRIVER_WALL_TO_SWITCH.f() + Measurements.FIELD_SWITCH_WIDTH.f()
						+ Measurements.robotWidth.f() + .5,
				Measurements.FIELD_WIDTH.f() - Measurements.FIELD_WALL_TO_SIDE_OF_SWITCH.f() - 2, 170.99, 0, 3);

		right.print();
		left.print();
		return asList(right, left);
	}

	private List<BobPath> getLeftScale() {
		BobPath left = new BobPath(config, "Left_Left_Scale");
		left.addWaypoint(StartingPositions.kLeftStartingPoint);
		left.addWaypoint((Waypoints.mLeftScaleCorner.x) * (2.0 / 3), Waypoints.mLeftScaleCorner.y + 3, 0, 0, 3);

		left.addWaypoint(Waypoints.mLeftScaleCorner, 0, 3);
		left.addWaypoint(Waypoints.mLeftScaleTurnAround, 0, 3);
		left.addWaypoint(Waypoints.mCubePlatformZone1LeftScale, 0, 3);
		left.addWaypoint(Waypoints.mLeftScaleTurnAround, 0, 3);
		left.addWaypoint(Waypoints.mLeftScaleCorner, 0, 3);

		left.print();
		return asList(left);
	}

	public static void main(String[] args) {
		new BobPathCreator().generatePaths();
	}
}