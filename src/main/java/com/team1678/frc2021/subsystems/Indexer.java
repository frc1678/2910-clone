package com.team1678.frc2021.subsystems;

import java.nio.channels.ReadPendingException;
import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2021.Constants;

import com.team254.lib.drivers.TalonFXFactory;
import com.team1678.frc2021.planners.IndexerMotionPlanner;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer extends Subsystem {

    // Variable Declarations
    private static Indexer mInstance = null;
    private IndexerMotionPlanner mMotionPlanner;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private double mIndexerStart;

    // Slot Proxies
    private final DigitalInput mSlot0Proxy = new DigitalInput(Constants.kSlot0Proxy);
    private final DigitalInput mSlot1Proxy = new DigitalInput(Constants.kSlot1Proxy);
    private final DigitalInput mSlot2Proxy = new DigitalInput(Constants.kSlot2Proxy);
    private final DigitalInput mSlot3Proxy = new DigitalInput(Constants.kSlot3Proxy);
    private final DigitalInput mSlot4Proxy = new DigitalInput(Constants.kSlot4Proxy);

    // Slot state arrays
    private boolean[] mCleanSlots = { false, false, false, false, false };

    private static final boolean[] kFullSlots = {true, true, true, true, true };
    private static final boolean[] kEmptySlots = {false, false, false, false, false };

    // Declare States and Wanted Actions
    public enum WantedAction {
        NONE, INDEX, UNJAM, PREP, ZOOM, SLOW_ZOOM, HELLA_ZOOM, BARF,
    }

    public enum State {
        IDLE, INDEXING, UNJAMMING, PREPPING, ZOOMING, SLOW_ZOOMING, HELLA_ZOOMING, BARFING,
    }

    private int mSlotGoal;
    private final TalonFX mMaster;
    private State mState = State.IDLE;
    private double mInitialTime = 0;
    private double mOffset = 0;
    private boolean mStartCounting = false;
    private boolean mBackwards = false;

    /**
     * The Indexer Utility Class
     */
    private Indexer() {

        // Set up TalonFX for the Falcon500
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kIndexerId);

        mMaster.config_kP(0, Constants.kIndexerKp, Constants.kLongCANTimeoutMs);
        mMaster.config_kI(0, Constants.kIndexerKi, Constants.kLongCANTimeoutMs);
        mMaster.config_kD(0, Constants.kIndexerKd, Constants.kLongCANTimeoutMs);
        mMaster.config_kF(0, Constants.kIndexerKf, Constants.kLongCANTimeoutMs);
        mMaster.config_kP(1, Constants.kIndexerVelocityKp, Constants.kLongCANTimeoutMs);
        mMaster.config_kI(1, Constants.kIndexerVelocityKi, Constants.kLongCANTimeoutMs);
        mMaster.config_kD(1, Constants.kIndexerVelocityKd, Constants.kLongCANTimeoutMs);
        mMaster.config_kF(1, Constants.kIndexerVelocityKf, Constants.kLongCANTimeoutMs);

        mMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mMaster.configMotionCruiseVelocity(Constants.kIndexerMaxVelocity);
        mMaster.configMotionAcceleration(Constants.kIndexerMaxAcceleration);

        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(false);
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);

        mMaster.setSelectedSensorPosition(0, 0, Constants.kLongCANTimeoutMs);
        mMaster.configClosedloopRamp(0.0);

        // Initiate new Indexer Motion Planner
        mMotionPlanner = new IndexerMotionPlanner();
    }

    /**
     * Runs the state machine for the Indexer
     */
    public void runStateMachine() {
        final double now = Timer.getFPGATimestamp();

        switch (mState) {
            case IDLE:
                mPeriodicIO.indexer_control_mode = ControlMode.Velocity;
                mPeriodicIO.indexer_demand = 0;
                break;
            case INDEXING:
                mPeriodicIO.indexer_control_mode = ControlMode.MotionMagic;

                if (!mGeneratedGoal) {
                    mSlotGoal = mMotionPlanner.findNearestOpenSlot(indexer_angle);
                    mGeneratedGoal = true;
                }
                mPeriodicIO.indexer_demand = mMotionPlanner.findAngleGoal(mSlotGoal, indexer_angle, 0);

                if (mMotionPlanner.isAtGoal(mSlotGoal, indexer_angle, 0)) {
                    if (mCleanSlots[mSlotGoal]) {
                        mGeneratedGoal = false;
                        // mSlotGoal = mMotionPlanner.findNearestOpenSlot(indexer_angle, mProxyStatus);
                        // mPeriodicIO.indexer_demand = mMotionPlanner.findAngleGoalToIntake(mSlotGoal,
                        // indexer_angle);
                    }
                }
                break;
            case UNJAMMING:
                mPeriodicIO.indexer_control_mode = ControlMode.Velocity;
                mPeriodicIO.indexer_demand = -Constants.kZoomingVelocity;
                break;
            case PREPPING:
                mPeriodicIO.indexer_control_mode = ControlMode.Velocity;
                mPeriodicIO.indexer_demand = 0;
                break;
                break;
            case ZOOMING:
                mPeriodicIO.indexer_control_mode = ControlMode.Velocity;
                mPeriodicIO.indexer_demand = mBackwards ? -Constants.kZoomingVelocity : Constants.kZoomingVelocity;
                break;
            case SLOW_ZOOMING:
                mPeriodicIO.indexer_control_mode = ControlMode.Velocity;
                mPeriodicIO.indexer_demand = (mBackwards ? -Constants.kZoomingVelocity : Constants.kZoomingVelocity) * 0.3;
                break;
            case HELLA_ZOOMING:
                mPeriodicIO.indexer_control_mode = ControlMode.Velocity;
                mPeriodicIO.indexer_demand = (mBackwards ? -Constants.kZoomingVelocity : Constants.kZoomingVelocity) * 1.5;
                break;
            case BARFING:
                mPeriodicIO.indexer_control_mode = ControlMode.Velocity;
                mPeriodicIO.indexer_demand = (-Constants.kZoomingVelocity) * 2; // TODO Make the intake barf as well
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
            case UNJAM:
                mState = State.UNJAMMING;
                break;
            case PREP:
                mState = State.PREPPING;
                break;
            case ZOOM:
                mState = State.ZOOMING;
                break;
            case SLOW_ZOOM:
                mState = State.SLOW_ZOOMING;
                break;
            case HELLA_ZOOM:
                mState = State.HELLA_ZOOMING;
                break;
            case BARF:
                mState = State.BARFING;
                break;
        }

        if (mState != prev_state && mState != State.BARFING && mState != State.INDEXING) {
            mSlotGoal = mMotionPlanner.findNearestSlot(mPeriodicIO.indexer_angle, mTurret.getAngle());
        }

        if (mState != prev_state && mState == State.UNJAMMING) {
            mIndexerStart = Timer.getFPGATimestamp();
            mBackwards = true;
        }

        if (mState != prev_state && mState == State.ZOOMING) {
            mMaster.configClosedloopRamp(0.2, 0);
        } else if (mState != prev_state) {
            mMaster.configClosedloopRamp(0.0, 0);
        }
    }

    /**
     * Returns the Indexer velocity
     * @return Indexer velocity
     */
    public double getIndexerVelocity() {
        try {
            if (mPeriodicIO.indexer_control_mode == ControlMode.Velocity) {
                return mPeriodicIO.indexer_demand;
            } else {
                return 0;
            }
        } catch (ReadPendingException readError) {
            System.out.println("Unable to read Indexer Velocity");
        }
    }

    /**
     * Checks if the slots are filled
     * @return Are the slots filled
     */
    public synchronized boolean slotsFilled() {
        return Arrays.equals(mCleanSlots, kFullSlots);
    }

    /**
     * Checks if the slots are empty
     * @return Are the slots empty
     */
    public synchronized boolean slotsEmpty() {
        return Arrays.equals(mCleanSlots, kEmptySlots);
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
        SmartDashboard.putString("IndexerControlMode", mPeriodicIO.indexer_control_mode.name());
        SmartDashboard.putNumber("IndexerSetpoint", mPeriodicIO.indexer_demand);
        SmartDashboard.putNumber("IndexerVelocity", mPeriodicIO.indexer_velocity);
        SmartDashboard.putNumber("IndexerOffset", mOffset);

        SmartDashboard.putNumber("SlotNumberGoal", mSlotGoal);

        SmartDashboard.putString("DirtySlots", Arrays.toString(mPeriodicIO.raw_slots));
        SmartDashboard.putString("CleanSlots", Arrays.toString(mCleanSlots));
        SmartDashboard.putBoolean("Snapped", mPeriodicIO.snapped);
    }

    /**
     * Updates the slot status
     * @param indexer_angle Angle of the indexer
     */
    private void updateSlots(double indexer_angle) {
        mCleanSlots = mMotionPlanner.updateSlotStatus(indexer_angle, mPeriodicIO.raw_slots);
    }

    /**
     * Sets the percentage for the open loop
     * @param percentage The Percentage to set the open loop at
     */
    public synchronized void setOpenLoop(double percentage) {
        mPeriodicIO.indexer_control_mode = ControlMode.PercentOutput;
        mPeriodicIO.indexer_demand = percentage;
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
        mPeriodicIO.raw_slots[0] = mSlot0Proxy.get();
        mPeriodicIO.raw_slots[1] = mSlot1Proxy.get();
        mPeriodicIO.raw_slots[2] = mSlot2Proxy.get();
        mPeriodicIO.raw_slots[3] = mSlot3Proxy.get();
        mPeriodicIO.raw_slots[4] = mSlot4Proxy.get();
        mPeriodicIO.indexer_current = mMaster.getStatorCurrent();
    }

    /**
     * Sets the PeriodicIO needed for the Indexer
     */
    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        private boolean[] raw_slots = { false, false, false, false, false };

        public double indexer_velocity;
        public double indexer_current;
        public boolean snapped;

        // OUTPUTS
        public ControlMode indexer_control_mode = ControlMode.PercentOutput;
        public double indexer_demand;
    }
}
