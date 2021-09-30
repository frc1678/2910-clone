package com.team1678.frc2021;

import com.team1678.frc2021.subsystems.Swerve;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import com.team2910.lib.autos.AutonomousSelector;
import com.team2910.lib.autos.AutonomousTrajectories;
import com.team2910.lib.commands.DriveCommand;
import com.team2910.lib.math.Rotation2;
import com.team2910.lib.robot.*;

public class RobotContainer {
    
    private final XboxController driver = new XboxController(0);

    private final Swerve swerve = Swerve.getInstance();
    
    private final AutonomousSelector autonomousSelector;
    private AutonomousTrajectories autonomousTrajectories;

    private static RobotContainer instance = null;
	
	public static RobotContainer getInstance(){
		if(instance == null)
			instance = new RobotContainer();
		return instance;
	}

    public RobotContainer() {
        driver.getLeftXAxis().setInverted(true);
        driver.getRightXAxis().setInverted(true);

        autonomousTrajectories  = new AutonomousTrajectories();
        autonomousSelector= new AutonomousSelector(autonomousTrajectories);

        CommandScheduler.getInstance().setDefaultCommand(swerve, new DriveCommand(swerve, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));


        configureButtonBindings();
    }

    private void configureButtonBindings() {
        driver.getBackButton().whenPressed(
                () -> swerve.resetGyroAngle(Rotation2.ZERO)
        );
        driver.getStartButton().whenPressed(
                swerve::resetWheelAngles
        );
    }

    private Axis getDriveForwardAxis() {
        return driver.getLeftYAxis();
    }

    private Axis getDriveStrafeAxis() {
        return driver.getLeftXAxis();
    }

    private Axis getDriveRotationAxis() {
        return driver.getRightXAxis();
    }

    public Swerve getDrivetrainSubsystem() {
        return swerve;
    }

    public XboxController getDriver() {
        return driver;
    }

    public Command getAutonomousCommand() {
        return autonomousSelector.getCommand(this);
    }

    public void outputTelemetry(){
        SmartDashboard.putNumber("Drive Forward Axis", getDriveForwardAxis().get());
        SmartDashboard.putNumber("Drive Strafe Axis", getDriveStrafeAxis().get());
        SmartDashboard.putNumber("Drive Rotation Axis", getDriveRotationAxis().get());
    }

}
