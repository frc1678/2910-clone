package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2021.Constants;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;

import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.TimeDelayedBoolean;

import com.team254.lib.util.ReflectingCSVWriter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Intake extends Subsystem {

    private static double kIntakingVoltage = 4.0;
    private static double kIdleVoltage = 0;
    private static double mCurrent;

    private static Intake mInstance;
    private TimeDelayedBoolean mIntakeSolenoidTimer = new TimeDelayedBoolean();

    private Solenoid mDeployOut;
    private Solenoid mDeployIn;

    public enum WantedAction {
        NONE, INTAKE, REVERSE, RETRACT,
    }

    public enum State {
        IDLE, INTAKING, REVERSING, RETRACTING,
    }

    private State mState = State.IDLE;

    private static PeriodicIO mPeriodicIO = new PeriodicIO();
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private final TalonFX mMaster;
    private final TalonFX mSlave;

    private Intake() {
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kMasterIntakeRollerId);
        mDeployOut = Constants.makeSolenoidForId(Constants.kDeployOutSolenoidId);
        mDeployIn = Constants.makeSolenoidForId(Constants.kDeployInSolenoidId);
        mSlave = TalonFXFactory.createPermanentSlaveTalon(Constants.kSlaverIntakeRollerId, Constants.kMasterIntakeRollerId);
        mSlave.setInverted(true);
    }

    public static synchronized Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    @Override
    public void stop() {
        mMaster.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void zeroSensors() {
        // Zero sensors?
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
                synchronized (Intake.this) {
                    runStateMachine();

                }
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.IDLE;
                stop();
            }
        });
    }

    public void runStateMachine() {
        switch (mState) {
        case IDLE:
            mPeriodicIO.demand = kIdleVoltage;
            break;
        case INTAKING:
            mPeriodicIO.demand = kIntakingVoltage;
            mPeriodicIO.deploy_out = true;
            mPeriodicIO.deploy_in = false;
            break;
        case REVERSING:
            mPeriodicIO.demand = -kIntakingVoltage;
            break;
        case RETRACTING:
            mPeriodicIO.demand = kIdleVoltage;
            mPeriodicIO.deploy_out = false;
            mPeriodicIO.deploy_in = true;
            break;
        default:
            System.out.println("Fell through on Intake states!");
        }
    }

    public synchronized void setOpenLoop(double percentage) {
        mPeriodicIO.demand = percentage;
    }

    public double getVoltage() {
        return mPeriodicIO.demand;
    }

    public double getCurrent() {
        return mCurrent;
    }

    public synchronized State getState() {
        return mState;
    }

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
        case NONE:
            mState = State.IDLE;
            break;
        case INTAKE:
            mState = State.INTAKING;
            break;
        case REVERSE:
            mState = State.REVERSING;
            break;
        case RETRACT:
            mState = State.RETRACTING;
            break;
        }

    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.intake_out = mIntakeSolenoidTimer.update(mPeriodicIO.deploy_out & !mPeriodicIO.deploy_in, 0.2);
        mCurrent = mPeriodicIO.current;
        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);
        mDeployOut.set(mPeriodicIO.deploy_out);
        mDeployIn.set(mPeriodicIO.deploy_in);
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putNumber("Intake Current", mPeriodicIO.current);
        SmartDashboard.putNumber("Intake Demand", mMaster.getMotorOutputVoltage());
        SmartDashboard.putString("Intake State", mState.toString());
        SmartDashboard.putBoolean("Intake Deploy Goal", mPeriodicIO.deploy_out & !mPeriodicIO.deploy_in);
        SmartDashboard.putBoolean("Intake Deploy Actual", mPeriodicIO.intake_out);
        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double current;
        public boolean intake_out;

        // OUTPUTS
        public double demand;
        public boolean deploy_out;
        public boolean deploy_in;
    }
}

