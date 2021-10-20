package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.subsystems.Canifier;
import com.team1678.lib.math.Conversions;
import com.team254.lib.drivers.MotorChecker;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.team254.lib.drivers.BaseTalonChecker;

import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogEncoder;

import java.util.ArrayList;

public class Hood extends ServoMotorSubsystem {
    private static Hood mInstance;

    private AnalogEncoder mEncoder;
    private final double kHoodEncoderStartingPos = 0.0;

    public synchronized static Hood getInstance() {
        if (mInstance == null) {
            mInstance = new Hood(Constants.kHoodConstants);
        }
        return mInstance;
    }

    private Hood(final ServoMotorSubsystemConstants constants) {
        super(constants);
        // mMaster.setSelectedSensorPosition((int) unitsToTicks(17.66));
        mEncoder = new AnalogEncoder(Constants.kHoodConstants.kAbsoluteEncoderID);
    }

    public synchronized double getAngle() {
        return getPosition();
    }

    public synchronized boolean getAtGoal() {
        return Util.epsilonEquals(getAngle(), getSetpoint(), 5.0);
    }

    public synchronized boolean getTucked() {
        return Util.epsilonEquals(getAngle(), Constants.kHoodConstants.kMinUnitsLimit, 5.0);
    }

    private void getHoodReset() {
        int absolutePosition = (int) kHoodEncoderStartingPos;
        mMaster.setSelectedSensorPosition(absolutePosition);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        super.writePeriodicOutputs();
    }

    @Override
    public synchronized void readPeriodicInputs() {
        SmartDashboard.putNumber("Hood Encoder Position", mEncoder.getDistance());
        super.readPeriodicInputs();
    }

    @Override
    public boolean checkSystem() {
        return BaseTalonChecker.checkMotors(this, new ArrayList<MotorChecker.MotorConfig<BaseTalon>>() {
            private static final long serialVersionUID = -716113039054569446L;

            {
                add(new MotorChecker.MotorConfig<>("master", mMaster));
            }
        }, new MotorChecker.CheckerConfig() {
            {
                mRunOutputPercentage = 0.5;
                mRunTimeSec = 1.0;
                mCurrentFloor = 0.1;
                mRPMFloor = 90;
                mCurrentEpsilon = 2.0;
                mRPMEpsilon = 200;
                mRPMSupplier = () -> mMaster.getSelectedSensorVelocity();
            }
        });
    }

    

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();
    }

    public void setCoastMode() {
        mMaster.setNeutralMode(NeutralMode.Coast);
    }
}
