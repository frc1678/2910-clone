package com.team1678.frc2021.auto.actions;

import com.team1678.frc2021.subsystems.Swerve;

public class SetTargetHeadingAction extends RunOnceAction{
	double targetHeading;
	Swerve swerve;
	
	public SetTargetHeadingAction(double targetHeading){
		this.targetHeading = targetHeading;
		swerve = Swerve.getInstance();
	}
	
	@Override
	public void runOnce() {
		// swerve.setAbsolutePathHeading(targetHeading); // TODO: Fix
	}

}
