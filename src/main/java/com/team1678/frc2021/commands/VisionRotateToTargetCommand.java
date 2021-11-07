package com.team1678.frc2021.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

import com.team1678.frc2021.subsystems.Limelight;
import com.team1678.frc2021.subsystems.Swerve;
import com.team2910.lib.control.PidConstants;
import com.team2910.lib.control.PidController;
import com.team2910.lib.math.Vector2;


public class VisionRotateToTargetCommand extends CommandBase {
    private static final PidConstants PID_CONSTANTS = new PidConstants(1.0, 0.0, 0.05);

    private final Swerve swerve;
    private final Limelight limelight;

    private final int yJoystick;
    private final int xJoystick;
    private final XboxController xboxController;

    private PidController controller = new PidController(PID_CONSTANTS);
    private double lastTime = 0.0;

    public VisionRotateToTargetCommand(Swerve drivetrain, Limelight visionSubsystem,
                                       int yAxis, int xAxis, XboxController xboxController) {
        this.swerve = drivetrain;
        this.limelight = visionSubsystem;
        this.yJoystick = yAxis;
        this.xJoystick = xAxis;
        this.xboxController = xboxController;

        addRequirements(drivetrain);

        controller.setInputRange(0.0, 2.0 * Math.PI);
        controller.setContinuous(true);
    }

    @Override
    public void initialize() {
        lastTime = Timer.getFPGATimestamp();
        limelight.setCamMode(0);
        //limelight.setSnapshotEnabled(true);
        controller.reset();
    }

    @Override
    public void execute() {

        System.out.println("wow");

        double time = Timer.getFPGATimestamp();
        double dt = time - lastTime;
        lastTime = time;
        
        double yAxis = xboxController.getRawAxis(yJoystick);
        double xAxis = xboxController.getRawAxis(xJoystick);

        Translation2d translationalVelocity = new Translation2d(yAxis, xAxis);

        double rotationalVelocity = 0.0;

        SmartDashboard.putBoolean("Sees target", limelight.seesTarget());
        if(true) {
            double currentAngle = swerve.getPose().getRotation().getRadians();
            double targetAngle = limelight.getTargetAngle().getAsDouble();

            SmartDashboard.putNumber("Current Angle", currentAngle);
            SmartDashboard.putNumber("Target Angle", targetAngle);

            controller.setSetpoint(targetAngle);
            rotationalVelocity = controller.calculate(currentAngle, dt);
        }
        swerve.drive(translationalVelocity, rotationalVelocity, true, true);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
