// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.team1678.frc2021;

import com.team1323.lib.util.CrashTracker;
import com.team1678.frc2021.subsystems.Swerve;
import com.team2910.lib.robot.UpdateManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team1678.frc2021.SubsystemManager;
import com.team1678.frc2021.subsystems.*;
import com.team1678.frc2021.loops.*;
import com.team1678.frc2021.controlboard.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;
  public RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  // loopers
  private final Looper mEnabledLooper = new Looper();
  private final Looper mDisabledLooper = new Looper();
  
  // subsystem instances
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
  private final Canifier mCanifier = Canifier.getInstance();
  private final Infrastructure mInfrastructure = Infrastructure.getInstance();
  private final Pigeon mPigeon = Pigeon.getInstance();
  private final Intake mIntake = Intake.getInstance();
  private final Indexer mIndexer = Indexer.getInstance();
  private final Shooter mShooter = Shooter.getInstance();
  private final Hood mHood = Hood.getInstance();
  private final Limelight mLimelight = Limelight.getInstance();

  // superstructure
  private final Superstructure mSuperstructure = Superstructure.getInstance();

  // controlboard
  private final ControlBoard mControlBoard = ControlBoard.getInstance();

  /**
   * This function is run when the 3 is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    // TODO: Initialise hood and do hood setpoint login in Superstructure.
    m_robotContainer = new RobotContainer();

    try {
      CrashTracker.logRobotInit();
  
      mSubsystemManager.setSubsystems(
        mSuperstructure,
        mCanifier,
        mInfrastructure,
        mIntake,
        mIndexer,
        mShooter,
        mHood,
        mLimelight
      );

      mSubsystemManager.registerEnabledLoops(mEnabledLooper);
      mSubsystemManager.registerDisabledLoops(mDisabledLooper);

    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  /**
   * This function is called every 3 packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the 3's periodic
    // block in order for anything in the Command-based framework to work.
    // CommandScheduler.getInstance().run();

    mSubsystemManager.outputToSmartDashboard();
    mEnabledLooper.outputToSmartDashboard();

  }

  /** This function is called once each time the 3 enters Disabled mode. */
  @Override
  public void disabledInit() {
    try {
      CrashTracker.logDisabledInit();

      mLimelight.setLed(Limelight.LedMode.ON);
      mLimelight.triggerOutputs();

      mEnabledLooper.stop();

      mInfrastructure.setIsDuringAuto(true);

      mDisabledLooper.start();

      mHood.setNeutralMode(NeutralMode.Coast);

    } catch (Throwable t) {
        CrashTracker.logThrowableCrash(t);
        throw t;
    }
  }

  @Override
  public void disabledPeriodic() {
    // m_robotContainer.resetAngleToAbsolute();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    mLimelight.setLed(Limelight.LedMode.ON);

    SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

    try {
      CrashTracker.logAutoInit();
      mDisabledLooper.stop();

      mInfrastructure.setIsDuringAuto(true);
      mHood.setNeutralMode(NeutralMode.Brake);

      // RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

      mEnabledLooper.start();

      mLimelight.setPipeline(Constants.kPortPipeline);
    } catch (Throwable t) {
        CrashTracker.logThrowableCrash(t);
        throw t;
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    mLimelight.setLed(Limelight.LedMode.ON);
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    try {
      CrashTracker.logTeleopInit();
      mDisabledLooper.stop();
 
      mInfrastructure.setIsDuringAuto(false);

      mEnabledLooper.start();
      mHood.setNeutralMode(NeutralMode.Brake);

      mLimelight.setLed(Limelight.LedMode.ON);
      mLimelight.setPipeline(Constants.kPortPipeline);
      
    } catch (Throwable t) {
        CrashTracker.logThrowableCrash(t);
        throw t;
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    try {
      double timestamp = Timer.getFPGATimestamp();

      /* Check for hood scan */
      mSuperstructure.setWantHoodAdjustmentReset(mControlBoard.getWantHoodAdjustmentReset());
      
      /* Use control board to set superstructure and subsystem functionality */
      if (mControlBoard.getTuck()) {
          mSuperstructure.setWantTuck(true);
      } else if (mControlBoard.getUntuck()) {
          mSuperstructure.setWantTuck(false);
      } else if (mControlBoard.getPreShot()) {
          mSuperstructure.setWantPrep();
      } else if (mControlBoard.getShoot()) {
          mSuperstructure.setWantShoot();;
      } else if (mControlBoard.getTestSpit()) {
          mSuperstructure.setWantTestSpit();
      } else if (mControlBoard.getRunIntake()) {
          mIntake.setState(Intake.WantedAction.INTAKE);
      } else if (mControlBoard.getRetractIntake()) {
          mIntake.setState(Intake.WantedAction.RETRACT);
      } else if (mControlBoard.getReverseIntake()) {
          mIntake.setState(Intake.WantedAction.REVERSE);
      } else {
          mIntake.setState(Intake.WantedAction.NONE);
      }

    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();

    SmartDashboard.putString("Match Cycle", "TEST");

    try {
      System.out.println("Starting check systems.");

      mDisabledLooper.stop();
      mEnabledLooper.stop();

    } catch (Throwable t) {
        CrashTracker.logThrowableCrash(t);
        throw t;
    }

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
