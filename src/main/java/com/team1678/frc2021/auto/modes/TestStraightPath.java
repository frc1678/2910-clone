package com.team1678.frc2021.auto.modes;

import java.util.Arrays;
import java.util.List;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.auto.AutoModeBase;
import com.team1678.frc2021.auto.AutoModeEndedException;
//import com.team1678.frc2021.auto.actions.RemainingProgressAction;
import com.team1678.frc2021.auto.actions.ResetPoseAction;
import com.team1678.frc2021.auto.actions.SetTrajectoryAction;
import com.team1678.frc2021.auto.actions.WaitAction;
import com.team1678.frc2021.auto.actions.WaitForDistanceAction;
import com.team1678.frc2021.auto.actions.WaitForHeadingAction;
import com.team1678.frc2021.auto.actions.WaitToFinishPathAction;
import com.team1678.frc2021.auto.actions.WaitToPassXCoordinateAction;
import com.team1678.frc2021.auto.actions.WaitToPassYCoordinateAction;
import com.team1678.frc2021.loops.LimelightProcessor;
import com.team1678.frc2021.loops.LimelightProcessor.Pipeline;

import com.team1678.frc2021.subsystems.Swerve;
import com.team1678.frc2021.*;

import com.team2910.lib.autos.*;
import com.team2910.lib.control.Trajectory;

import edu.wpi.first.wpilibj.Timer;

public class TestStraightPath extends AutoModeBase {

    final boolean left;
    final double directionFactor;

    @Override
    public List<Trajectory> getPaths() {
        //return Arrays.asList(trajectories.testStraightTrajectory);
        return null;
    }

	public TestStraightPath(boolean left) {
        
        this.left = left;
        directionFactor = left ? -1.0 : 1.0;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        super.startTime = Timer.getFPGATimestamp();

        System.out.println("RUNNING STRAIGHT PATH AUTO!!!");

        runAction(new ResetPoseAction(true));
        // runAction(new SetTrajectoryAction(trajectories.testStraightTrajectory, 0.0, 1.0));
        // runAction(new SetTrajectoryAction(trajectories.straightPath2, 0.0, 1.0));

        System.out.println("Auto mode finished in " + currentTime() + " seconds");
    }
}
