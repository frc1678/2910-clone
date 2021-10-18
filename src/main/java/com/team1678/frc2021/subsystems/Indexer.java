package com.team1678.frc2021.subsystems;

import java.nio.channels.ReadPendingException;
import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2021.Constants;

import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;
import com.team1678.frc2021.planners.IndexerMotionPlanner;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer extends Subsystem {

    // Intake Instance
    private final Intake mIntake = Intake.getInstance();

    // Variable Declarations
    private static Indexer mInstance = null;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    
    private final TalonFX mMaster;

    // Slot Proxies
    private final DigitalInput mLowerBeamBreak = new DigitalInput(Constants.kLowerBeamBreak);
    private final DigitalInput mUpperBeamBreak = new DigitalInput(Constants.kUpperBeamBreak);

    private static double kIdleVoltage = 0.0;
    private static double kIndexingVoltage = 9.0;
    private static double kFeedingVoltage = 12.0;
    private static final double kIndexTime = 1.0;

    private static double mLastTimestamp = 0.0;
    private static double mLastLowerBeamBreak = 0.0;
    private static boolean mSeenBall = false;

    // Declare States and Wanted Actions
    public enum WantedAction {
        NONE, INDEX, FEED,
    }

    public enum State {
        IDLE, INDEXING, FEEDING,
    }

    private State mState = State.IDLE;


    /**
     * The Indexer Utility Class
     */
    private Indexer() {

        // Set up TalonFX for the Falcon500
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kIndexerId);

        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(false);
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);
    }

    /**
     * Runs the state machine for the Indexer
     */
    public void runStateMachine() {
        switch (mState) {
            // Idling
            case IDLE:
                mPeriodicIO.demand = kIdleVoltage;

                if (mIntake.getState() == Intake.State.INTAKING) {
                    mState = State.INDEXING;
                }
                break;
            // Indexing, pushing balls to the shooter
            case INDEXING:
            
                /*
                if (mPeriodicIO.upper_break) {
                    mInstance.setState(Indexer.WantedAction.NONE);
                }

                if (mPeriodicIO.lower_break) {
                    mLastLowerBeamBreak = mPeriodicIO.timestamp;
                    mSeenBall = true;
                }

                while (mPeriodicIO.timestamp - mLastLowerBeamBreak <= kIndexTime && mSeenBall) {
                    mPeriodicIO.demand = kIndexingVoltage;
                }
                mSeenBall = false;
                */

                mPeriodicIO.demand = kIndexingVoltage;

                break;
            // Feeding, pushing balls for shooting into the shooter
            case FEEDING:
                mPeriodicIO.demand = kFeedingVoltage;
                break;
            default:
                System.out.println("Fell through on Indexer states!");
        }

    }

    /**
     * Sets the states based on Wanted Actions
     * @param wanted_state the Wanted Action Enum
     */
    public void setState(WantedAction wanted_state) {
        final State prev_state = mState;

        switch (wanted_state) {
            case NONE:
                mState = State.IDLE;
                break;
            case INDEX:
                mState = State.INDEXING;
                break;
            case FEED:
                mState = State.FEEDING;
                break;
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
     * Gets the instant of the Indexer
     * @return Indexer Instance
     */
    public static synchronized Indexer getInstance() {
        if (mInstance == null) {
            mInstance = new Indexer();
        }
        return mInstance;
    }

    /**
     * Sets the Indexer output to the Smart Dashboard on the Driverstation
     */
    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Lower Break", mPeriodicIO.lower_break);
        SmartDashboard.putBoolean("Upper Break", mPeriodicIO.upper_break);
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

    /**
     * Sets the periodic inputs for the Indexer
     */
    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.current = mMaster.getStatorCurrent();
        mPeriodicIO.lower_break = mLowerBeamBreak.get();
        mPeriodicIO.upper_break = mUpperBeamBreak.get();
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
}
