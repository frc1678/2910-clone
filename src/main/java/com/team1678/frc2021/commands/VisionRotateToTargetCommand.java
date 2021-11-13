package com.team1678.frc2021.commands;

import com.team1678.frc2021.subsystems.Swerve;
import com.team1678.frc2021.SwerveTools;

import com.team1678.frc2021.subsystems.Limelight;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team2910.lib.control.PidConstants;
import com.team2910.lib.control.PidController;
import com.team2910.lib.math.Vector2;

import java.util.function.DoubleSupplier;

public class VisionRotateToTargetCommand extends CommandBase {
    private static final PidConstants PID_CONSTANTS = new PidConstants(20.0, 0.0, 1.0);

    private final Swerve drivetrain;
    private final Limelight limelight;

    private final int yJoystick;
    private final int xJoystick;
    private final Joystick controller;

    private PidController pid_controller = new PidController(PID_CONSTANTS);
    private double lastTime = 0.0;

    public VisionRotateToTargetCommand(Swerve drivetrain, Limelight limelight,
                                       int yAxis, int xAxis, Joystick controller) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.yJoystick = yAxis;
        this.xJoystick = xAxis;
        this.controller = controller;

        addRequirements(drivetrain);

        pid_controller.setInputRange(0.0, 2.0 * Math.PI);
        pid_controller.setContinuous(true);
    }

    @Override
    public void initialize() {
        lastTime = Timer.getFPGATimestamp();
        limelight.setCamMode(0);
        pid_controller.reset();
    }
    

    @Override
    public void execute() {
        System.out.println("running command");
        double time = Timer.getFPGATimestamp();
        double dt = time - lastTime;
        lastTime = time;

        double yAxis = controller.getRawAxis(yJoystick);
        double xAxis = controller.getRawAxis(xJoystick);

        Translation2d translationalVelocity = SwerveTools.applyTranslationalDeadband(new Translation2d(yAxis, xAxis));

        double rotationalVelocity = 0.0;
        if (limelight.seesTarget()) {
            System.out.println("running inside");

            double currentAngle = drivetrain.getPose().getRotation().getRadians();
            double targetOffset = Math.toRadians(limelight.getTargetOffset().getAsDouble());

            pid_controller.setSetpoint(currentAngle + targetOffset);
            rotationalVelocity = pid_controller.calculate(currentAngle, dt);
            System.out.println(rotationalVelocity);

            /*
            if (translationalVelocity.length <
                    Swerve.FEEDFORWARD_CONSTANTS.getStaticConstant() / RobotController.getBatteryVoltage()) {
                rotationalVelocity += Math.copySign(
                        0.8 / RobotController.getBatteryVoltage(),
                        rotationalVelocity
                );
            }
            */
        }
        drivetrain.drive(translationalVelocity, rotationalVelocity, true, true);
    }

    @Override
    public void end(boolean interrupted) {
        // empty
    }
}
