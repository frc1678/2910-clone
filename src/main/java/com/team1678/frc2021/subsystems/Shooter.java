package com.team1678.frc2021.subsystems;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;
import com.team254.lib.util.ReflectingCSVWriter;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.Util;

import java.util.ArrayList;

public class Shooter extends Subsystem {
    private static Shooter mInstance;

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private final TalonFX mMaster;
    private final TalonFX mSlave;
    private final TalonFX mOverhead;

    private boolean mRunningManual = false;

    private static double kFlywheelVelocityConversion = 400.0 / 2048.0;
    private static double kOverheadVelocityConversion = 400.0 / 2048.0;

    private static double kFlywheelTolerance = 200.0;
    private static double kOverheadTolerance = 200.0;

    private Shooter() {
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kMasterFlywheelID);
        mSlave = TalonFXFactory.createPermanentSlaveTalon(Constants.kSlaveFlywheelID, Constants.kMasterFlywheelID);
        mSlave.setInverted(true);
        mOverhead = TalonFXFactory.createDefaultTalon(Constants.kHoodRollerID);

        // flywheel motor configs
        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(true); //TODO: check value
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);
        
        mMaster.config_kP(0, Constants.kShooterP, Constants.kLongCANTimeoutMs);
        mMaster.config_kI(0, Constants.kShooterI, Constants.kLongCANTimeoutMs);
        mMaster.config_kD(0, Constants.kShooterD, Constants.kLongCANTimeoutMs);
        mMaster.config_kF(0, Constants.kShooterF, Constants.kLongCANTimeoutMs);
        mMaster.config_IntegralZone(0, (int) (200.0 / kFlywheelVelocityConversion));
        mMaster.selectProfileSlot(0, 0);

        // flywheel master current limit
        SupplyCurrentLimitConfiguration curr_lim = new SupplyCurrentLimitConfiguration(true, 40, 100, 0.02);
        mMaster.configSupplyCurrentLimit(curr_lim);

        // overhead motor configs
        mOverhead.set(ControlMode.PercentOutput, 0);
        mOverhead.setInverted(false); //TODO: check value
        mOverhead.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mOverhead.enableVoltageCompensation(true);
        
        mOverhead.config_kP(0, Constants.kShooterP, Constants.kLongCANTimeoutMs);
        mOverhead.config_kI(0, Constants.kShooterI, Constants.kLongCANTimeoutMs);
        mOverhead.config_kD(0, Constants.kShooterD, Constants.kLongCANTimeoutMs);
        mOverhead.config_kF(0, Constants.kShooterF, Constants.kLongCANTimeoutMs);
        mOverhead.config_IntegralZone(0, (int) (200.0 / kFlywheelVelocityConversion));
        mOverhead.selectProfileSlot(0, 0);
 
        // feedback sensor        
        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mMaster.configClosedloopRamp(0.2);
        mOverhead.set(ControlMode.PercentOutput, 0);
        mOverhead.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mOverhead.configClosedloopRamp(0.2);
    }

    public synchronized static Shooter mInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putNumber("Flywheel Velocity", mPeriodicIO.flywheel_velocity);
        SmartDashboard.putNumber("Flywheel Voltage", mPeriodicIO.flywheel_voltage);
        SmartDashboard.putNumber("Flywheel Current", mPeriodicIO.flywheel_current);
        SmartDashboard.putNumber("Flywheel Goal", mPeriodicIO.flywheel_demand);
        SmartDashboard.putNumber("Flywheel Temperature", mPeriodicIO.flywheel_temperature);

        SmartDashboard.putNumber("Overhead Velocity", mPeriodicIO.overhead_velocity);
        SmartDashboard.putNumber("Overhead Voltage", mPeriodicIO.flywheel_voltage);
        SmartDashboard.putNumber("Overhead Current", mPeriodicIO.overhead_current);
        SmartDashboard.putNumber("Overhead Goal", mPeriodicIO.overhead_demand);
        SmartDashboard.putNumber("Overhead Temperature", mPeriodicIO.overhead_temperature);

        SmartDashboard.putBoolean("Shooter Spun Up: ", spunUp());
        
        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    @Override
    public void stop() {
        setOpenLoop(0);
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                //startLogging();
            }
            @Override
            public void onLoop(double timestamp) {
            }
            @Override
            public void onStop(double timestamp) {
                stopLogging();
            }
        });
    }

    public synchronized void setOpenLoop(double flywheel) {
        mPeriodicIO.flywheel_demand = flywheel;
        mRunningManual = true;
    }

    public synchronized double getFlywheelVoltage() {
        return mPeriodicIO.flywheel_demand;
    }

    public synchronized double getFlywheelRPM() {
        return mMaster.getSelectedSensorVelocity();
    }

    public synchronized double getFlywheelVelocity() {
        return mPeriodicIO.flywheel_velocity;
    }

    public synchronized double getOverheadVoltage() {
        return mPeriodicIO.overhead_demand;
    }

    public synchronized double getOverheadRPM() {
        return mOverhead.getSelectedSensorVelocity();
    }

    public synchronized double getOverheadVelocity() {
        return mPeriodicIO.overhead_velocity;
    }

    public synchronized boolean spunUp() {
        if (mPeriodicIO.flywheel_demand > 0 && mPeriodicIO.overhead_demand > 0) {
            return Util.epsilonEquals(mPeriodicIO.flywheel_demand, mPeriodicIO.flywheel_velocity, kFlywheelTolerance) &&
                    Util.epsilonEquals(mPeriodicIO.overhead_demand, mPeriodicIO.overhead_velocity, kOverheadTolerance);
        }
        return false;
    }

    public synchronized void setVelocity(double velocity) {
        mPeriodicIO.flywheel_demand = velocity;
        mPeriodicIO.overhead_demand = -velocity; // must be reverse of lower flywheel
        mRunningManual = false;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        
        mPeriodicIO.flywheel_velocity = mMaster.getSelectedSensorVelocity() * kFlywheelVelocityConversion;
        mPeriodicIO.flywheel_voltage = mMaster.getMotorOutputVoltage();
        mPeriodicIO.flywheel_current = mMaster.getStatorCurrent();
        mPeriodicIO.flywheel_temperature = mMaster.getTemperature();

        mPeriodicIO.overhead_velocity = mOverhead.getSelectedSensorVelocity() * kOverheadVelocityConversion;
        mPeriodicIO.overhead_voltage = mOverhead.getMotorOutputVoltage();
        mPeriodicIO.overhead_current = mOverhead.getStatorCurrent();
        mPeriodicIO.overhead_temperature = mOverhead.getTemperature();

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        if (!mRunningManual) {
            mMaster.set(ControlMode.Velocity, mPeriodicIO.flywheel_demand / kFlywheelVelocityConversion, DemandType.ArbitraryFeedForward, Constants.kShooterFlywheel_ff_v);
            mOverhead.set(ControlMode.Velocity, mPeriodicIO.overhead_demand / kOverheadVelocityConversion, DemandType.ArbitraryFeedForward, Constants.kShooterOverhead_ff_v);
        } else {
            mMaster.set(ControlMode.PercentOutput, 0.0);
            mOverhead.set(ControlMode.PercentOutput, 0.0);
        }
    }

    @Override
    public synchronized boolean checkSystem() {
        return true;
    }
    
    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/SHOOTER-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    public static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }

    public static class PeriodicIO {
        //INPUTS
        public double timestamp;
        
        public double flywheel_velocity;
        public double flywheel_voltage;
        public double flywheel_current;
        public double flywheel_temperature;

        public double overhead_velocity;
        public double overhead_voltage;
        public double overhead_current;
        public double overhead_temperature;

        //OUTPUTS
        public double flywheel_demand;
        public double overhead_demand;
    }
}
