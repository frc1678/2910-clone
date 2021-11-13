package com.team1678.frc2021.auto;

import java.util.List;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.commands.AutoVisionRotateToTargetCommand;
import com.team1678.frc2021.commands.ShootCommand;
import com.team1678.frc2021.subsystems.Limelight;
import com.team1678.frc2021.subsystems.Superstructure;
import com.team1678.frc2021.subsystems.Swerve;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class CenterThreeFront extends SequentialCommandGroup{

    public CenterThreeFront(Swerve s_Swerve) {
        
        final Superstructure mSuperstructure = Superstructure.getInstance();
        final Limelight mLimelight = Limelight.getInstance();
        
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Trajectory moveFront =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(2.90, 5.84, Rotation2d.fromDegrees(0.0)),
                List.of(),
                new Pose2d(3.90, 5.84 , Rotation2d.fromDegrees(0.0)),
                Constants.AutoConstants.defaultConfig);

        SwerveControllerCommand moveFrontCommand =
            new SwerveControllerCommand(
                moveFront,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                () -> Rotation2d.fromDegrees(0),
                s_Swerve::setModuleStates,
                s_Swerve);
        
        ShootCommand shoot =
            new ShootCommand(mSuperstructure);

        AutoVisionRotateToTargetCommand vision =
            new AutoVisionRotateToTargetCommand(s_Swerve, mLimelight);

        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(2.9, 5.84, Rotation2d.fromDegrees(0.0)))),
            vision,
            shoot,
            moveFrontCommand
        );

    }

    
}