package com.team319.follower;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.lib.geometry.Pose2d;

import edu.wpi.first.wpilibj.command.Subsystem;



public interface FollowsArc {
    public TalonSRX getLeft();
    public TalonSRX getRight();
    public double getDistance();
    public Pose2d getCurrentPose();
    public Subsystem getRequiredSubsystem();
}