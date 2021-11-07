package com.team1678.frc2021.subsystems;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure extends Subsystem {
    // Superstructure instance
    private static Superstructure mInstance;

    // Required subsystem instances
    private final Indexer mIndexer = Indexer.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    private final Hood mHood = Hood.getInstance();

    // Setpoint variables
    private double mHoodSetpoint = 30.0;;
    private double mShooterSetpoint = 100.0;

    public enum WantedAction {
        NONE, IDLE, TUCK, SCAN, PREP, SHOOT, SPIT
    }

    public enum State {
        IDLE, TUCKING, SCANNING, PREPPING, SHOOTING, SPITTING
    }
    private State mState = State.IDLE;

    private Superstructure () {
        // empty
    }

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
        case NONE:
            break;
        case IDLE:
            mState = State.IDLE;
            break;
        case TUCK:
            mState = State.TUCKING;
            break;
        case SCAN:
            mState = State.SCANNING;
            break;
        case PREP:
            mState = State.PREPPING;
            break;
        case SHOOT:
            mState = State.SHOOTING;
            break;
        case SPIT:
            mState = State.SPITTING;
            break;
        default:
            break;
        }
    }

    public synchronized State getState() {
        return mState;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mState = State.IDLE;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Superstructure.this) {
                    updateSetpoints();
                }
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.IDLE;
                stop();
            }
        });
    }

    public void updateSetpoints() {
        switch (mState) {
            case IDLE:
                mIndexer.setState(Indexer.WantedAction.NONE); // indexer should be inactive in idle
                mShooterSetpoint = 0.0; // shooter doesn't spin in idle
                // hood should keep its current setpoint

                mShooter.setVelocity(mShooterSetpoint);
                break;
            case TUCKING:
                mHood.setHoodTargetAngle(Constants.kHoodMinLimit);
                mShooterSetpoint = 0.0;

                mShooter.setVelocity(mShooterSetpoint);
                break;
            case SCANNING:
                mShooterSetpoint = 0.0;
                mHoodSetpoint = Constants.kHoodMinLimit + 10;
                if (Util.epsilonEquals(mHood.getHoodEncoderPosition(), Constants.kHoodMinLimit + 10, 10.0)) {
                    mHoodSetpoint = Constants.kHoodMaxLimit - 10;
                } else if (Util.epsilonEquals(mHood.getHoodEncoderPosition(), Constants.kHoodMaxLimit - 10, 10.0)) {
                    mHoodSetpoint = Constants.kHoodMinLimit + 10;
                }

                mShooter.setVelocity(mShooterSetpoint);
                break;
            case PREPPING:
                // double distanceToTarget = Vision.distanceToTarget(); // placeholder
                // mHoodSetpoint = getHoodSetpoint(distanceToTarget);
                // mShooterSetpoint = getShooterSetpoint(distanceToTarget);
                
                break;
            case SHOOTING:
                mShooter.setVelocity(mShooterSetpoint);
                // if (mShooter.spunUp()) {
                    mIndexer.setState(Indexer.WantedAction.FEED);
                // }
                break;
            case SPITTING:
                mHoodSetpoint = Constants.kHoodMinLimit + 10;
                mShooterSetpoint = 2000;

                mShooter.setVelocity(mShooterSetpoint);
                if (mShooter.spunUp()) {
                    mIndexer.setState(Indexer.WantedAction.FEED);
                }
                break;
            }

        mHood.setHoodTargetAngle(mHoodSetpoint);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Superstructure Wanted Action", getState().toString());
        SmartDashboard.putNumber("Hood Setpoint", mHoodSetpoint);
        SmartDashboard.putNumber("Shooter Setpoint", mShooterSetpoint);
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub

    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        return true;
    }

    public static synchronized Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }
        return mInstance;
    }

}
