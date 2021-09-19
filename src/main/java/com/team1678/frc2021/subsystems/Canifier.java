package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.CANifier;

public class Canifier extends Subsystem {
    private static Canifier mInstance;

    private Canifier() {

    }

    public static synchronized Canifier getInstance() {
        if (mInstance == null) {
            mInstance = new Canifier();
        }
        return mInstance;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        CANifier.PinValues pins = new CANifier.PinValues();
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
    }
}
