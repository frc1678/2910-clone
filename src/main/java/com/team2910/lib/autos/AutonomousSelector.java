package com.team2910.lib.autos;

import java.util.LinkedList;
import java.util.Queue;

import com.team1678.frc2021.auto.modes.StandStillMode;
import com.team1678.frc2021.subsystems.Swerve;
import com.team2910.lib.commands.FollowTrajectoryCommand;
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

public class AutonomousSelector {
    private final AutonomousTrajectories trajectories;

    private static SendableChooser<Side> sideChooser;
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
        autonomousModeChooser.setDefaultOption("Driven", AutonomousMode.DRIVEN);
        autonomousModeChooser.addOption("Hybrid", AutonomousMode.HYBRID);
        autonomousModeChooser.addOption("Autonomous", AutonomousMode.AUTONOMOUS);
        autonomousModeChooser.addOption("Test Straight", AutonomousMode.TEST_STRAIGHT);
        autoTab.add("Mode", autonomousModeChooser);

        //testPath = autoTab.add("Test Path", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    }

    public AutonomousSelector(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;
    }

    public CommandBase getCommand(){
        AutonomousMode mode = autonomousModeChooser.getSelected();
        Rotation2 startingOrientation = orientationChooser.getSelected();
        FollowTrajectoryCommand testPathCommand = new FollowTrajectoryCommand(trajectories.getTestPath());

        CommandGroupBase group = CommandGroupBase.sequence();
        
        //group.setRunWhenDisabled(true);

        CommandGroupBase.sequence(new InstantCommand(() -> {
            Swerve.getInstance().getGyroscope().setAdjustmentAngle(
                    Swerve.getInstance().getGyroscope().getUnadjustedAngle().rotateBy(startingOrientation)
            );
        }));

        switch (mode) {
            case DRIVEN: 
                break;
            case HYBRID: 
                break;
            case AUTONOMOUS:
                break;
            case TEST_STRAIGHT:
                group.addCommands(testPathCommand);
                break;
            default:
                System.out.println("ERROR: unexpected auto mode: " + mode);
                break; 
        }
        return group;

    }

    public Queue<Command> getHybridQueue() {
        return hybridCommandQueue;
    }

    private enum AutonomousMode {
        DRIVEN,
        HYBRID,
        AUTONOMOUS,
        TEST_STRAIGHT
    }
}
