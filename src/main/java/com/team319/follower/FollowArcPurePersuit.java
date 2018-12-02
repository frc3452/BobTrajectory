package com.team319.follower;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.Trajectory.Segment;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;

public class FollowArcPurePersuit extends Command {

	private int distancePidSlot = 0;
	private int rotationPidSlot = 1;
	private int kMinPointsInTalon = 5;
	private int maxLookAhead = 10;
	private boolean isFinished = false;
	private Trajectory trajectoryToFollow = null;
	private MotionProfileStatus status = new MotionProfileStatus();
	private boolean hasPathStarted;

	/**
	 * this is only either Disable, Enable, or Hold. Since we'd never want one side
	 * to be enabled while the other is disabled, we'll use the same status for both
	 * sides.
	 */
	private SetValueMotionProfile setValue = SetValueMotionProfile.Disable;

	private class BufferLoader implements java.lang.Runnable {
		private int lastPointSent = 0;
		private TalonSRX talon;
		private Trajectory prof;
		private double startPosition = 0;

		public BufferLoader(TalonSRX talon, Trajectory prof, double startPosition) {
			this.talon = talon;
			this.prof = prof;
			this.startPosition = startPosition;
		}

		public void run() {
			talon.processMotionProfileBuffer();

			if (lastPointSent >= prof.getNumSegments()) {
				return;
			}

			Pose2d currentPose = drivetrain.getCurrentPose();
			Segment lookAheadSegment = prof.getSegment(lastPointSent + maxLookAhead); 
			Pose2d lookAheadPose = new Pose2d(lookAheadSegment.x, lookAheadSegment.y, Rotation2d.fromDegrees(lookAheadSegment.heading));

			if (!talon.isMotionProfileTopLevelBufferFull() && lastPointSent < prof.getNumSegments()) {
				TrajectoryPoint point = new TrajectoryPoint();
				/* for each point, fill our structure and pass it to API */
				 //TODO: incorrect distance
				point.position = lookAheadPose.distance(currentPose) + startPosition;

				 //TODO: incorrect velocity units
				point.velocity = prof.getSegment(lastPointSent).vel;
				point.timeDur = TrajectoryDuration.Trajectory_Duration_10ms;
				
				//TODO: incorrect heading calc
				point.auxiliaryPos = 10* (lookAheadPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees());
				point.profileSlotSelect0 = distancePidSlot;
				point.profileSlotSelect1 = rotationPidSlot;
				point.zeroPos = false;
				point.isLastPoint = false;
				if ((lastPointSent + 1) == prof.getNumSegments()) {
					point.isLastPoint = true; /** set this to true on the last point */
				}

				talon.pushMotionProfileTrajectory(point);
				lastPointSent++;
				hasPathStarted = true;
			}
		}
	}

	// Runs the runnable
	private Notifier buffer;
	private FollowsArc drivetrain;
	private TalonSRX rightTalon;
	private TalonSRX leftTalon;

	public FollowArcPurePersuit(FollowsArc drivetrain, Trajectory trajectoryToFollow, int maxLookAhead) {
		this.drivetrain = drivetrain;
		requires(drivetrain.getRequiredSubsystem());
		this.trajectoryToFollow = trajectoryToFollow;
		this.maxLookAhead = maxLookAhead;

		rightTalon = drivetrain.getRight();
		leftTalon = drivetrain.getLeft();
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		setUpTalon(leftTalon);
		setUpTalon(rightTalon);

		setValue = SetValueMotionProfile.Disable;

		rightTalon.set(ControlMode.MotionProfileArc, setValue.value);
		leftTalon.follow(rightTalon, FollowerType.AuxOutput1);
		buffer = new Notifier(
				new BufferLoader(rightTalon, trajectoryToFollow,drivetrain.getDistance()));

		buffer.startPeriodic(.005);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		rightTalon.getMotionProfileStatus(status);

		if (status.isUnderrun) {
			// if either MP has underrun, stop both
			setValue = SetValueMotionProfile.Disable;
		} else if (status.btmBufferCnt > kMinPointsInTalon) {
			// if we have enough points in the talon, go.
			setValue = SetValueMotionProfile.Enable;
		} else if (status.activePointValid && status.isLast) {
			// if both profiles are at their last points, hold the last point
			setValue = SetValueMotionProfile.Hold;
		}

		rightTalon.set(ControlMode.MotionProfileArc, setValue.value);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if (!hasPathStarted) {
			return false;
		}
		boolean leftComplete = status.activePointValid && status.isLast;
		boolean trajectoryComplete = leftComplete;
		return trajectoryComplete || isFinished;
	}

	// Called once after isFinished returns true
	protected void end() {
		buffer.stop();
		resetTalon(rightTalon, ControlMode.PercentOutput, 0);
		resetTalon(leftTalon, ControlMode.PercentOutput, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		buffer.stop();
		resetTalon(rightTalon, ControlMode.PercentOutput, 0);
		resetTalon(leftTalon, ControlMode.PercentOutput, 0);
	}

	// set up the talon for motion profile control
	private void setUpTalon(TalonSRX talon) {
		talon.clearMotionProfileTrajectories();
		talon.changeMotionControlFramePeriod(5);
		talon.clearMotionProfileHasUnderrun(10);
	}

	// set the to the desired controlMode
	// used at the end of the motion profile
	private void resetTalon(TalonSRX talon, ControlMode controlMode, double setValue) {
		talon.clearMotionProfileTrajectories();
		talon.clearMotionProfileHasUnderrun(10);
		talon.changeMotionControlFramePeriod(10);
		talon.set(controlMode, setValue);
	}
}