package com.team1678.frc2021.commands;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.SwerveTools;
import com.team1678.frc2021.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve s_Swerve;
    private Joystick controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;

    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, Joystick controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {

        double yAxis;
        double xAxis;
        double rAxis;

        Translation2d tAxes; // translational axis

        /* Inversions */
        yAxis = Constants.Swerve.invertYAxis ? controller.getRawAxis(translationAxis) : controller.getRawAxis(translationAxis);
        xAxis = Constants.Swerve.invertXAxis ? controller.getRawAxis(strafeAxis) : controller.getRawAxis(strafeAxis);
        rAxis = Constants.Swerve.invertRAxis ? controller.getRawAxis(rotationAxis) : controller.getRawAxis(rotationAxis);

        /* Deadbands */
        tAxes = SwerveTools.applyTranslationalDeadband(new Translation2d(yAxis, xAxis));
        rAxis = SwerveTools.applyRotationalDeadband(rAxis);

        translation = new Translation2d(tAxes.getX(), tAxes.getY()).times(Constants.Swerve.maxSpeed);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }

    public Translation2d getChassisTranslation() {
        double[] axes = SwerveTools.getAxes(controller, translationAxis, strafeAxis, rotationAxis);

        double yAxis = axes[0];
        double xAxis = axes[1];

        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);

        return translation;
    }

    public double getChassisRotation() {
        double[] axes = SwerveTools.getAxes(controller, translationAxis, strafeAxis, rotationAxis);

        double rAxis = axes[2];
        
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;

        return rotation;
    }

}
