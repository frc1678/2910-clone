package com.team2910.lib.autos;

import java.util.LinkedList;
import java.util.Queue;

import com.team1678.frc2021.RobotContainer;
import com.team1678.frc2021.auto.modes.StandStillMode;
import com.team1678.frc2021.subsystems.Swerve;
import com.team2910.lib.commands.FollowTrajectoryCommand;
import com.team2910.lib.control.Trajectory;
import com.team2910.lib.math.RigidTransform2;
import com.team2910.lib.math.Rotation2;
import com.team2910.lib.util.Side;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousSelector {
    private final AutonomousTrajectories trajectories;
    
    private static SendableChooser<Rotation2> orientationChooser;
    private static SendableChooser<AutonomousMode> autonomousModeChooser;
    //private static NetworkTableEntry testPath;

    private Queue<Command> hybridCommandQueue = new LinkedList<>();

    static {

        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto settings");

        orientationChooser = new SendableChooser<>();
        orientationChooser.setDefaultOption("Forward", Rotation2.ZERO);
        orientationChooser.addOption("Backwards", Rotation2.fromDegrees(180.0));
        orientationChooser.addOption("Left", Rotation2.fromDegrees(90.0));
        orientationChooser.addOption("Right", Rotation2.fromDegrees(270.0));
        autoTab.add("Starting Orientation", orientationChooser);

        autonomousModeChooser = new SendableChooser<>();
        autonomousModeChooser.addOption("Test Straight", AutonomousMode.TEST_STRAIGHT);
        autoTab.add("Mode", autonomousModeChooser);

    }

    public AutonomousSelector(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;
    }

    private Command getTestStraight(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        resetRobotPose(command, container, trajectories.getTestPath());
        follow(command, container, trajectories.getTestPath());

        return command;
    }

    private void follow(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        // Not worying about autos for now
        //command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory));
    }


    /**
     * @deprecated Doesn't do anything needs fix
     */
    @Deprecated
    private void resetRobotPose(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        /*
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetGyroAngle(Rotation2.ZERO)));
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetPose(
                new RigidTransform2(trajectory.calculate(0.0).getPathState().getPosition(), Rotation2.ZERO))));*/
    }

    public Command getCommand(RobotContainer container){
        AutonomousMode mode = autonomousModeChooser.getSelected();

        switch (mode) {
            case TEST_STRAIGHT:
                return getTestStraight(container);
            default:
                System.out.println("ERROR: unexpected auto mode: " + mode);
                break; 
        }

        return getTestStraight(container);

    }

    public Queue<Command> getHybridQueue() {
        return hybridCommandQueue;
    }

    private enum AutonomousMode {
        TEST_STRAIGHT
    }
}