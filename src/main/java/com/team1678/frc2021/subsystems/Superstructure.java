package com.team1678.frc2021.subsystems;

import java.util.OptionalDouble;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.controlboard.ControlBoard;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;
import com.team1678.frc2021.states.ShooterRegression;
import com.team1678.frc2021.subsystems.Indexer.WantedAction;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.Util;
import com.team2910.lib.math.MathUtils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure extends Subsystem {
    // Superstructure instance
    private static Superstructure mInstance;

    private Superstructure () {
        // empty
    }

    public static synchronized Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }
        return mInstance;
    }

    /* Required subsystem instances */
    private final Intake mIntake = Intake.getInstance();
    private final Indexer mIndexer = Indexer.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    private final Hood mHood = Hood.getInstance();
    private final ControlBoard mControlBoard = ControlBoard.getInstance();
    
    // private final Swerve mSwerve = new Swerve();
    // private final Vision mVisionTracker = new Vision(mSwerve);
    
    /* Setpoint variables */
    private double mHoodSetpoint = 50.0;
    private double mShooterSetpoint = 0.0;

    // Superstructure constants
    private final double kSpitVelocity = 200;

    /* SUPERSTRUCTURE FUNCTIONS */
    private boolean mWantsTuck = false;
    private boolean mWantsScan = false;
    private boolean mWantsPrep = false;
    private boolean mWantsShoot = false;
    private boolean mWantsSpit = false;

    // Status variables for functions
    private boolean mIsSpunUp = false;
    private double formal_shooter = 0.0;
    private double formal_hood = 0.0;
    private Indexer.WantedAction formal_indexer = Indexer.WantedAction.NONE;

    // Function setters
    public synchronized void setWantTuck(boolean tuck) {
        mWantsTuck = tuck;
        mWantsPrep = false;
        mWantsShoot = false;
        mWantsScan = false;
    }

    public synchronized void setWantHoodScan(boolean scan) {
        if (scan != mWantsScan) {
            if (scan) {
                mHoodSetpoint = Constants.kHoodMinLimit + 10;
            } else {
                mHoodSetpoint = mHood.getHoodAngle();
            }
        }
        mWantsScan = scan;
    }

    public synchronized void setWantPrep() {
        mWantsPrep = !mWantsPrep;
        mWantsShoot = false;
    }

    public synchronized void setWantShoot() {
        mWantsPrep = false;
        mWantsShoot = !mWantsShoot;
    }

    public synchronized void setWantTestSpit() {
        mWantsSpit = !mWantsSpit;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                // TODO: Add anything necessary
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Superstructure.this) {
                    maybeUpdateGoalFromHoodScan(timestamp);
                    setSetpoints();
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop(); // TODO: Add anything necessary
            }
        });
    }

    /* UPDATE SHOOTER AND HOOD GOAL WHEN VISION AIMING */
    public synchronized void maybeUpdateGoalFromVision(double timestamp) {
        /*
        if (mVisionTracker.hasTarget()) {
            OptionalDouble distance_to_target = mVisionTracker.getDistanceToTarget();
            if (distance_to_target.isPresent()) {
                mHoodSetpoint = getHoodSetpointFromRegression(distance_to_target.getAsDouble());
                mShooterSetpoint = getShooterSetpointFromRegression(distance_to_target.getAsDouble());
            }
        }
        */

    }

    /* UPDATE HOOD GOAL FOR SCANNING TARGET */
    public synchronized void maybeUpdateGoalFromHoodScan(double timestamp) {
        if (!mWantsScan) {
            return;
        }

        if (Util.epsilonEquals(mHood.getHoodAngle(), Constants.kHoodMinLimit + 10, 10.0)) {
            mHoodSetpoint = Constants.kHoodMaxLimit - 10;
        } else if (Util.epsilonEquals(mHood.getHoodAngle(), Constants.kHoodMaxLimit - 10, 10.0)) {
            mHoodSetpoint = Constants.kHoodMinLimit + 10;
        }
    }

    /* UPDATE AND SET ALL SETPOINTS */
    public void setSetpoints() {
        /* Default indexer wanted action to be set */
        Indexer.WantedAction real_indexer;
        /* Real hood angle setpoint to be set */
        double real_hood;
        /* Real shooter velocity setpoint to be set */
        double real_shooter;
        // status variable tracker for whether shooter is spun up
        mIsSpunUp = mShooter.spunUp(); 

        if (mWantsTuck) {
            real_indexer = Indexer.WantedAction.NONE;
            real_hood = Constants.kHoodMinLimit;
            real_shooter = 0.0;
        } else if (mWantsPrep) {
            real_indexer = Indexer.WantedAction.NONE;
            real_hood = mHoodSetpoint;
            real_shooter = 500/*mShooterSetpoint*/;
        } else if (mWantsShoot) {
            real_hood = mHoodSetpoint;
            real_shooter = 500/*mShooterSetpoint*/;
            real_indexer = Indexer.WantedAction.FEED;
        } else if (mWantsSpit) {
            real_hood = Constants.kHoodMinLimit;
            real_shooter = kSpitVelocity;
            real_indexer = Indexer.WantedAction.FEED;
        } else {
            real_indexer = Indexer.WantedAction.NONE;
            
            // read inputs for manual hood setting
            switch(mControlBoard.getManualHoodSet()){
                case 1:
                    mHoodSetpoint += 1;
                    break;
                case -1:
                    mHoodSetpoint += -1;
                    break;
                case 0:
                    mHoodSetpoint += 0;;
                    break;
            }
            real_hood = mHoodSetpoint;

            real_shooter = 0.0;
        }

        // clamp the hood goal between min and max hard stops for hood angle
        real_hood = MathUtils.clamp(real_hood, Constants.kHoodMinLimit, Constants.kHoodMaxLimit);

        /* FOLLOW HOOD SETPOINT GOAL */
        mHood.setSetpointMotionMagic(real_hood);

        /* FOLLOW HOOD SETPOINT GOAL */
        if (Math.abs(real_shooter) < Util.kEpsilon) {
            mShooter.setOpenLoop(0); // open loop if rpm goal is 0, to smooth spin down and stop belt skipping
        } else {
            mShooter.setVelocity(real_shooter);
        }

        /* SET INDEXER STATE */
        if (mIntake.getState() == Intake.State.INTAKING) {
            mIndexer.setState(Indexer.WantedAction.INDEX);
        } else if (mIntake.getState() == Intake.State.REVERSING) {
            mIndexer.setState(Indexer.WantedAction.REVERSE);
        } else if (mShooter.spunUp()) {
            mIndexer.setState(real_indexer);
        } else {
            mIndexer.setState(Indexer.WantedAction.NONE);
        }

        // update tracker variables for goals for smart dashboard readings
        formal_hood = real_hood;
        formal_shooter = real_shooter;
        formal_indexer = real_indexer;
    }

    /* GET SHOOTER AND HOOD SETPOINTS FROM SUPERSTRUCTURE CONSTANTS REGRESSION */
    private double getShooterSetpointFromRegression(double range) {
        if (ShooterRegression.kUseSmartdashboard) {
            return SmartDashboard.getNumber("Shooting RPM", 0);
        } else if (ShooterRegression.kUseFlywheelAutoAimPolynomial) {
            return ShooterRegression.kFlywheelAutoAimPolynomial.predict(range);
        } else {
            return ShooterRegression.kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
        }
    }
    private double getHoodSetpointFromRegression(double range) {
        if (ShooterRegression.kUseSmartdashboard) {
            return SmartDashboard.getNumber("Hood Angle", 0);
        } else if (ShooterRegression.kUseHoodAutoAimPolynomial) {
            return ShooterRegression.kHoodAutoAimPolynomial.predict(range);
        } else {
            return ShooterRegression.kHoodAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
        }
    }

    @Override
    public void outputTelemetry() {
        // Formal goals for hood, shooter, and indexer that are followed
        SmartDashboard.putNumber("Hood Goal", formal_hood);
        SmartDashboard.putNumber("Hood Setpoint", mHoodSetpoint);
        SmartDashboard.putNumber("Shooter Goal", formal_shooter);
        SmartDashboard.putNumber("Shooter Setpoint", mShooterSetpoint);
        SmartDashboard.putString("Indexer Goal", formal_indexer.toString());

        // Other status tracker variables
        SmartDashboard.putBoolean("Is Spun Up", mIsSpunUp);
        
        // Formal superstructure function values
        SmartDashboard.putBoolean("Wants Tuck", mWantsTuck);
        SmartDashboard.putBoolean("Wants Scan", mWantsScan);
        SmartDashboard.putBoolean("Wants Prep", mWantsPrep);
        SmartDashboard.putBoolean("Wants Shoot", mWantsShoot);
        SmartDashboard.putBoolean("Wants Spit", mWantsSpit);
    }

    @Override
    public void stop() {
        // empty
    }

    @Override
    public boolean checkSystem() {
        // empty
        return true;
    }

}
