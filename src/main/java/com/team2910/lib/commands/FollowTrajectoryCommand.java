package com.team2910.lib.commands;

import java.util.Set;
import java.util.function.Supplier;

import com.team1678.frc2021.Robot;
import com.team1678.frc2021.RobotContainer;
import com.team1678.frc2021.subsystems.Swerve;
import com.team2910.lib.control.Trajectory;
import com.team2910.lib.math.RigidTransform2;
import com.team2910.lib.math.Vector2;
import com.team2910.lib.robot.SwerveModule;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class FollowTrajectoryCommand extends CommandBase {

    private RobotContainer robotContainer = RobotContainer.getInstance();

    private final Supplier<Trajectory> trajectorySupplier;

    private Trajectory trajectory;

    private Boolean runsWhenDisabled;

    public FollowTrajectoryCommand(Trajectory trajectory) {
        this(() -> trajectory);
        addRequirements(Swerve.getInstance());
    }

    public FollowTrajectoryCommand(Supplier<Trajectory> trajectorySupplier) {
        this.trajectorySupplier = trajectorySupplier;

        addRequirements(Swerve.getInstance());
        runsWhenDisabled = true;
    }

    @Override
    public void initialize() {
        System.out.println("Running follow trajectory :)");
        trajectory = trajectorySupplier.get();
        Swerve.getInstance().resetPose(RigidTransform2.ZERO);
        Swerve.getInstance().getFollower().follow(trajectory);
    }

    protected void end() {
        //Swerve.getInstance().setSnapRotation(trajectory.calculate(trajectory.getDuration()).getPathState().getRotation().toRadians());

        new Thread(() -> {
            robotContainer.getDriver().getRawJoystick().setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
            robotContainer.getDriver().getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
            Timer.delay(0.5);
            robotContainer.getDriver().getRawJoystick().setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
            robotContainer.getDriver().getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
        }).start();
    }

    protected void interrupted() {
        end();
        Swerve.getInstance().getFollower().cancel();
    }

    @Override
    public boolean isFinished() {
        // Only finish when the trajectory is completed
        return Swerve.getInstance().getFollower().getCurrentTrajectory().isEmpty();
    }
    
    /*
    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
    */

    @Override
    public boolean runsWhenDisabled(){
        return runsWhenDisabled;
    }
    
}
