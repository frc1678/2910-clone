package com.team1678.frc2021;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class SwerveTools {
    int translationAxis;
    int strafeAxis;
    int rotationAxis;

    public SwerveTools() {
        // empty
    }

    /**
     * Calculates translational deadband and ramping for translational input for swerve drive
     *
     * @param input
     *            A Translation2d composed of translation and strafe axes as (yAxis, xAxis)
     */
    public static Translation2d applyTranslationalDeadband(Translation2d input) {
        double deadband = Constants.stickDeadband;
        if (input.getNorm() < deadband) {
            return new Translation2d();
        } else {
            Rotation2d deadband_direction = new Rotation2d(input.getX(), input.getY());
            Translation2d deadband_vector = new Translation2d(Constants.stickDeadband, deadband_direction);

            double scaled_x = input.getX() - (Math.signum(input.getX()) * deadband_vector.getX()) / (1 - deadband_vector.getX());
            double scaled_y = input.getY() - (Math.signum(input.getY()) * deadband_vector.getY()) / (1 - deadband_vector.getY());
            return new Translation2d(scaled_x, scaled_y);
        }
    }

    /**
     * Calculates rotational deadband and ramping for rotational input for swerve drive
     *
     * @param input
     *            Input for rotational axis (rAxis) as double
     */
    public static double applyRotationalDeadband(double input){
        double deadband = Constants.stickDeadband;
        if (Math.abs(input) < deadband) {
            return 0.0;
        } else {
            return (input - (Math.signum(input) * deadband)) / (1 - deadband);
        }
    }

    /**
     * Retrieves axes with swerve deadband and ramping for translation and rotation control applied
     *
     * @param controller
     *            Joystick controller from which raw axes are read
     * @param translationAxis
     *            yAxis input for translation control
     * @param strafeAxis
     *            xAxis input for strafe control
     * @param rotationAxis
     *            rAxis input for rotation control
     */
    public static double[] getAxes(Joystick controller, int translationAxis, int strafeAxis, int rotationAxis) {
        double yAxis = controller.getRawAxis(translationAxis);
        double xAxis = controller.getRawAxis(strafeAxis);
        double rAxis = controller.getRawAxis(rotationAxis);

        Translation2d tAxes;
        
        /* Deadbands */
        tAxes = applyTranslationalDeadband(new Translation2d(yAxis, xAxis));
        rAxis = applyRotationalDeadband(rAxis);

        double[] axes = {tAxes.getX(), tAxes.getY(), rAxis};

        return axes;
    }

}
