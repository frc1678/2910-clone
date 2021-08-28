package com.team1678.frc2021.auto.actions;

import com.team1678.frc2021.subsystems.Swerve;
import com.team2910.lib.control.Trajectory;

public class SetTrajectoryAction extends RunOnceAction{
	Trajectory trajectory;
    double goalHeading;
    double rotationScalar;
	Swerve swerve;
	
	public SetTrajectoryAction(Trajectory trajectory, double goalHeading, double rotationScalar){
		this.trajectory = trajectory;
        this.goalHeading = goalHeading;
        this.rotationScalar = rotationScalar;
		swerve = Swerve.getInstance();
	}
	
	@Override
	public synchronized void runOnce(){
		// swerve.setTrajectory(trajectory, goalHeading, rotationScalar); // TODO: Fix
	}
}
