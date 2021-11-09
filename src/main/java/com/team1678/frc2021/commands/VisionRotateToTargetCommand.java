package com.team1678.frc2021.commands;

import com.team1678.frc2021.subsystems.Swerve;
import com.team1678.frc2021.subsystems.Vision;
import com.team1678.frc2021.SwerveTools;
import com.team1678.frc2021.subsystems.Limelight;

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
    private static final PidConstants PID_CONSTANTS = new PidConstants(1.0, 0.0, 0.05);

    private final Swerve drivetrain;
    private final Vision visionSubsystem;

    private final int xAxis;
    private final int yAxis;

    private PidController controller = new PidController(PID_CONSTANTS);
    private double lastTime = 0.0;

    public VisionRotateToTargetCommand(Swerve drivetrain, Vision visionSubsystem,
                                       int xAxis, int yAxis) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.xAxis = xAxis;
        this.yAxis = yAxis;
        addRequirements(drivetrain);
        addRequirements(visionSubsystem);

        controller.setInputRange(0.0, 2.0 * Math.PI);
        controller.setContinuous(true);
    }

    @Override
    public void initialize() {
        lastTime = Timer.getFPGATimestamp();
        visionSubsystem.setCamMode(Limelight.CamMode.VISION);
        visionSubsystem.setSnapshotEnabled(true);
        controller.reset();
    }
    

    @Override
    public void execute() {
        System.out.println("running command");
        double time = Timer.getFPGATimestamp();
        double dt = time - lastTime;
        lastTime = time;

        Translation2d translationalVelocity = SwerveTools.applyTranslationalDeadband(new Translation2d(xAxis, yAxis));

        double rotationalVelocity = 0.0;
        if (visionSubsystem.hasTarget()) {
            System.out.println("running inside");
            double currentAngle = drivetrain.getPose().getRotation().getRadians();
            double targetAngle = visionSubsystem.getAngleToTarget().getAsDouble();
            controller.setSetpoint(targetAngle);
            rotationalVelocity = controller.calculate(currentAngle, dt);

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
        visionSubsystem.setSnapshotEnabled(false);
    }
}
