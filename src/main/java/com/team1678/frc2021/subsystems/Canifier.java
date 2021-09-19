package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;
import com.team1678.frc2021.Constants;

public class Canifier extends Subsystem {
    private static Canifier mInstance;
    private CANifier mCanifier;
    private PeriodicInputs mPeriodicInputs;

    private Canifier() {
        mCanifier = new CANifier(Constants.kCanifierId);
        mCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_1_General, 100, Constants.kLongCANTimeoutMs);
        mCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 2, Constants.kLongCANTimeoutMs);
        mPeriodicInputs = new PeriodicInputs();
    }

    public static synchronized Canifier getInstance() {
        if (mInstance == null) {
            mInstance = new Canifier();
        }
        return mInstance;
    }

    public synchronized boolean getHoodLimit() {
        return mPeriodicInputs.hood_limit_;
    }

    public int getDeviceId() {
        return mCanifier.getDeviceID();
    }

    @Override
    public synchronized void readPeriodicInputs() {
        CANifier.PinValues pins = new CANifier.PinValues();
        mCanifier.getGeneralInputs(pins);

        mPeriodicInputs.indexer_limit_ = !pins.SDA;
        mPeriodicInputs.hood_limit_ = !pins.SPI_MOSI_PWM1;

    }

    public synchronized CANifier getCanifier() {
        return mCanifier;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
    }

    @Override
    public void stop() {

    }

    @Override
    public void zeroSensors() {
        mPeriodicInputs = new PeriodicInputs();
    }

    private static class PeriodicInputs {
        public boolean indexer_limit_;
        public boolean hood_limit_;
    }
}
