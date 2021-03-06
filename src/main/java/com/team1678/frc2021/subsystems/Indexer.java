package com.team1678.frc2021.subsystems;

import java.nio.channels.ReadPendingException;
import java.time.zone.ZoneOffsetTransitionRule.TimeDefinition;
import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2021.Constants;

import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;
import com.team1678.frc2021.planners.IndexerMotionPlanner;
import com.team1678.lib.util.TimeDelayedBoolean;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer extends Subsystem {

    // Required Instances
    private final Intake mIntake = Intake.getInstance();
    private final Canifier mCanifier = Canifier.getInstance();    

    // Variable Declarations
    private static Indexer mInstance = null;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    
    private final TalonFX mMaster;

    // Slot Proxies
    // private final DigitalInput mLowerBeamBreak = new DigitalInput(Constants.kLowerBeamBreak);
    // private final DigitalInput mUpperBeamBreak = new DigitalInput(Constants.kUpperBeamBreak);

    // Intake Proxy Timer
    private TimeDelayedBoolean mIntakeProxyTimer = new TimeDelayedBoolean();  

    private static double kIdleVoltage = 0.0;
    private static double kIndexingVoltage = 7.0;
    private static double kFeedingVoltage = 8.0;
    private static final double kIndexTime = 1.0;

    private static double mLastTimestamp = 0.0;
    private static double mLastLowerBeamBreak = 0.0;
    private static boolean mSeenBall = false;

    /**
     * Gets the instant of the Indexer
     * @return Indexer Instance
     */
    public static synchronized Indexer getInstance() {
        if (mInstance == null) {
            mInstance = new Indexer();
        }
        return mInstance;
    }

    // Declare States and Wanted Actions
    public enum WantedAction {
        NONE, INDEX, FEED, REVERSE,
    }

    public enum State {
        IDLE, INDEXING, FEEDING, REVERSING,
    }

    private State mState = State.IDLE;


    /**
     * The Indexer Utility Class
     */
    private Indexer() {

        // Set up TalonFX for the Falcon500
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kIndexerId);
        mMaster.setInverted(true);
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);
    }

    /**
     * Sets the states based on Wanted Actions
     * @param wanted_state the Wanted Action Enum
     */
    public void setState(WantedAction wanted_state) {
        final State prev_state = mState;

        switch (wanted_state) {
            case NONE:
                // if (mIntake.getState() == Intake.State.INTAKING) {
                //     mState = State.INDEXING;
                // } else if (mIntake.getState() == Intake.State.REVERSING) {
                //     mState = State.REVERSING;
                // } else {
                mState = State.IDLE;
                // }
                break;
            case INDEX:
                mState = State.INDEXING;
                break;
            case FEED:
                mState = State.FEEDING;
                break;
            case REVERSE:
                mState = State.REVERSING;
        }
    }

    /**
     * Gets the current state of the Indexer
     * @return Indexer State
     */
    public synchronized State getState() {
        return mState;
    } 

    /**
     * Sets the percentage for the open loop
     * @param percentage The Percentage to set the open loop at
     */
    public synchronized void setOpenLoop(double percentage) {
        mPeriodicIO.demand = percentage;
    }

    /**
     * Stops the open loop
     */
    @Override
    public void stop() {
        setOpenLoop(0);
    }

    /**
     * Checks the system
     * @return true
     */
    @Override
    public boolean checkSystem() {
        return true;
    }

    /**
     * Zeros the encoder on the Falcon500 for the Indexer
     */
    @Override
    public void zeroSensors() {
        mMaster.setSelectedSensorPosition(0, 0, 10);
    }

    /**
     * Sets the periodic inputs for the Indexer
     */
    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.current = mMaster.getStatorCurrent();
        mPeriodicIO.lower_break = mIntakeProxyTimer.update(mCanifier.getIntakeBeamBreak(), 0.0);
        mPeriodicIO.upper_break = mCanifier.getShooterBeamBreak();
    }

    private boolean indexerIsFull() {
        return mPeriodicIO.upper_break;
    }

    private boolean ballAtIndexer() {
        return mPeriodicIO.lower_break;
    }

    private boolean indexNextBall() {
        // if (indexerIsFull()) {
        //     return false;
        // } else {
             return ballAtIndexer();
        // }
    }

    /**
     * Runs the state machine for the Indexer
     */
    public void runStateMachine() {
        switch (mState) {
            // Idling
            case IDLE:
                mPeriodicIO.demand = kIdleVoltage;
                break;
            // Indexing, pushing balls to the shooter
            case INDEXING:
                // mPeriodicIO.demand = kIndexingVoltage;
                mPeriodicIO.demand = indexNextBall() ? kIndexingVoltage : kIdleVoltage;
                break;
            // Feeding, pushing balls for shooting into the shooter
            case FEEDING:
                mPeriodicIO.demand = kFeedingVoltage;
                break;
            case REVERSING:
                mPeriodicIO.demand = -kIndexingVoltage;
                break;
            default:
                System.out.println("Fell through on Indexer states!");
        }

    }

    /**
     * Registers the enabled loops
     * @param enabledLooper the enabled ILooper
     */
    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mState = State.IDLE;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Indexer.this) {
                    runStateMachine();
                }
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.IDLE;
            }
        });
    }
    
    @Override
    public void writePeriodicOutputs() {
        mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);
    }

    /**
     * Sets the PeriodicIO needed for the Indexer
     */
    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double voltage;
        public double current;
        public boolean lower_break;
        public boolean upper_break;

        // OUTPUTS
        public double demand;
    }

    /**
     * Sets the Indexer output to the Smart Dashboard on the Driverstation
     */
    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Indexer State", mState.toString());
        SmartDashboard.putBoolean("Indexer is Full", indexerIsFull());
        SmartDashboard.putBoolean("Ball in Intake", ballAtIndexer());
        SmartDashboard.putNumber("Indexer Demand", mPeriodicIO.demand);
        SmartDashboard.putNumber("Indexer Voltage", mMaster.getMotorOutputVoltage());
        SmartDashboard.putNumber("Indexer Current", mPeriodicIO.current);
    }
}
