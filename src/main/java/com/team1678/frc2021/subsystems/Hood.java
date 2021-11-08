package com.team1678.frc2021.subsystems;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;
import com.team1678.frc2021.subsystems.ServoMotorSubsystem.ControlState;
import com.team254.lib.util.ReflectingCSVWriter;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;

import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.Util;
import com.team2910.lib.control.PidConstants;
import com.team2910.lib.control.PidController;
import com.team2910.lib.math.MathUtils;

import java.util.ArrayList;

import javax.swing.text.Position;

public class Hood extends Subsystem {
    private static Hood mInstance;

    private final TalonFX mMaster;
    private final AnalogEncoder mEncoder;
    protected ControlState mControlState = ControlState.OPEN_LOOP;

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
    private boolean mRunningManual = false;

    private boolean hoodWasReset = false;

    private void resetHoodToAbsolute() {
        // if (hoodWasReset) {
        //     return;
        // }
        double absolute_position = getHoodDegreesToTicks(getTicksToHoodDegrees(mEncoder.getDistance()) - (Constants.kHoodEncoderOffset - Constants.kHoodMinLimit));
        mMaster.setSelectedSensorPosition(absolute_position);
        hoodWasReset = true;
        System.out.println("resetting hood!");
    }

    private Hood() {
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kHoodID);
        
        // flywheel motor configs
        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(Constants.kHoodInvertMotor); //TODO: check value
        mMaster.setNeutralMode(NeutralMode.Brake);
        mMaster.setSensorPhase(false);
        
        mMaster.config_kP(0, Constants.kHoodP, Constants.kLongCANTimeoutMs);
        mMaster.config_kI(0, Constants.kHoodI, Constants.kLongCANTimeoutMs);
        mMaster.config_kD(0, Constants.kHoodD, Constants.kLongCANTimeoutMs);    
        mMaster.selectProfileSlot(0, 0);

        // flywheel master current limit
        SupplyCurrentLimitConfiguration curr_lim = new SupplyCurrentLimitConfiguration(true, 40, 100, 0.02);
        mMaster.configSupplyCurrentLimit(curr_lim);

        // feedback sensor        
        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mMaster.configClosedloopRamp(0.2);

        // config encoder
        mEncoder = new AnalogEncoder(Constants.kHoodAbsoluteEncoderID);
        mEncoder.setDistancePerRotation(33800); // ticks per rotation

    }

    public synchronized static Hood mInstance() {
        if (mInstance == null) {
            mInstance = new Hood();
        }
        return mInstance;
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
                // startLogging();
                
                // set hood falcon position to absolute encoder reset position
                // resetHoodToAbsolute();
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

    public synchronized void setOpenLoop(double demand) {
        mPeriodicIO.setpoint = demand;
        mRunningManual = true;
    }

    public void setNeutralMode(NeutralMode mode) {
        mMaster.setNeutralMode(mode);
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        mPeriodicIO.encoder_position = mEncoder.getDistance();
        mPeriodicIO.motor_position = mMaster.getSelectedSensorPosition();
        mPeriodicIO.hood_angle = getHoodEncoderPosition();

        if (mMaster.getControlMode() == ControlMode.MotionMagic) {
            mPeriodicIO.active_trajectory_position = (int) mMaster.getActiveTrajectoryPosition();

            final int newVel = (int) mMaster.getActiveTrajectoryVelocity();
            if (Util.epsilonEquals(newVel, Constants.kHoodCruiseVelocity, Math.max(1, Constants.kHoodDeadband)) || Util
                    .epsilonEquals(newVel, mPeriodicIO.active_trajectory_velocity, Math.max(1, Constants.kHoodDeadband))) {
                // Mechanism is ~constant velocity.
                mPeriodicIO.active_trajectory_acceleration = 0.0;
            } else {
                // Mechanism is accelerating.
                mPeriodicIO.active_trajectory_acceleration = Math
                        .signum(newVel - mPeriodicIO.active_trajectory_velocity) * Constants.kHoodCruiseAcceleration;
            }
            mPeriodicIO.active_trajectory_velocity = newVel;
        } else {
            mPeriodicIO.active_trajectory_position = Integer.MIN_VALUE;
            mPeriodicIO.active_trajectory_velocity = 0;
            mPeriodicIO.active_trajectory_acceleration = 0.0;
        }
        
        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }

        while (!hoodWasReset) {
            resetHoodToAbsolute();
        }
    }

    // intermediate methods
    public double getTicksToHoodDegrees(double ticks) {
        return ticks / Constants.kHoodGearRatio;
    }

    public double getHoodDegreesToTicks(double degrees) {
        return degrees * Constants.kHoodGearRatio;
    }

    public synchronized double getHoodEncoderPosition() {
        return getTicksToHoodDegrees(mPeriodicIO.encoder_position);
    }

    public synchronized double getHoodAngle() {
        return getTicksToHoodDegrees(mPeriodicIO.motor_position);
    }

    public void setHoodTargetAngle(double setpoint_angle) {
        mPeriodicIO.setpoint = setpoint_angle;
    }

    protected double unitsPerSecondToTicksPer100ms(double units_per_second) {
        return getHoodDegreesToTicks(units_per_second) / 10.0;
    }
    
    public synchronized void setSetpointMotionMagic(double units, double feedforward_v) {
        mPeriodicIO.setpoint = getHoodDegreesToTicks(units);
        mPeriodicIO.feedforward = unitsPerSecondToTicksPer100ms(feedforward_v) * (Constants.kHoodF + Constants.kHoodD / 100.0) / 1023.0;
        if (mControlState != ControlState.MOTION_MAGIC) {
            mMaster.selectProfileSlot(0, 0);
            mControlState = ControlState.MOTION_MAGIC;
        }
    }

    public synchronized void setSetpointMotionMagic(double units) {
        setSetpointMotionMagic(units, 0.0);
    }

    @Override
    public void writePeriodicOutputs() {
        if (mControlState == ControlState.MOTION_MAGIC) {
            mMaster.set(ControlMode.MotionMagic, mPeriodicIO.setpoint, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.feedforward);
        } else if (mControlState == ControlState.OPEN_LOOP) {
            mMaster.set(ControlMode.PercentOutput, mPeriodicIO.setpoint, DemandType.ArbitraryFeedForward, 0.0);
        } else {
            mMaster.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward, 0.0);
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

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putNumber("Hood Encoder Position (ticks)", mPeriodicIO.encoder_position);
        SmartDashboard.putNumber("Hood Encoder Position (degrees)", getHoodEncoderPosition());
        SmartDashboard.putNumber("Hood Motor Position (ticks)", mPeriodicIO.motor_position);
        SmartDashboard.putNumber("Hood Motor Position (degrees)", mPeriodicIO.motor_position / Constants.kHoodGearRatio);
        SmartDashboard.putNumber("Hood Angle", getHoodAngle());
        SmartDashboard.putNumber("Hood Goal", mPeriodicIO.setpoint);
        
        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    public static Hood getInstance() {
        if (mInstance == null) {
            mInstance = new Hood();
        }
        return mInstance;
    }

    public static class PeriodicIO {
        //INPUTS
        public double timestamp;
        public double encoder_position;
        public double motor_position;
        public double hood_angle;
        public int active_trajectory_position; // ticks
        public int active_trajectory_velocity; // ticks/100ms
        public double active_trajectory_acceleration; // ticks/100ms/s
        public double feedforward;

        //OUTPUTS
        public double setpoint;
    }
}
