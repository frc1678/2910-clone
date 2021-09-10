package com.team1678.frc2021.subsystems;

import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;
import com.team254.lib.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure extends Subsystem{

    //Instances
    private static Superstructure mInstance;

    private final Indexer mIndexer = Indexer.getInstance();


    private Rotation2d mFieldRelativeTurretGoal = null;

    private boolean mWantsShoot = false;
    private boolean mWantsSpinUp = false;
    private boolean mWantsUnjam = false;
    private boolean mWantsPreShot = false;

    private boolean mGotSpunUp = false;

    private boolean mEnableIndexer = true;
    private boolean mManualZoom = false;

    /**
     * Gets a superstructure instance
     * @return superstructure instance
     */
    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    }

    /**
     * Utility class
     */
    private Superstructure() {
    }

    public synchronized void enableIndexer(boolean indexer) {
        mEnableIndexer = indexer;
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                // TODO Do stuff
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Superstructure.this) {
                    updateCurrentState();
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    public synchronized void updateCurrentState() {
        // TODO Update state
    }

    public synchronized void setWantUnjam(boolean unjam) {
        mWantsUnjam = unjam;
    }

    public synchronized void setWantShoot() {
        mWantsSpinUp = false;
        mWantsShoot = !mWantsShoot;
        mGotSpunUp = false;
        mWantsPreShot = false;
    }

    public synchronized void setWantPreShot(boolean pre_shot) {
        mWantsSpinUp = false;
        mWantsShoot = false;
        mGotSpunUp = false;
        mWantsPreShot = pre_shot;
    }

    public synchronized void setWantSpinUp() {
        mWantsSpinUp = !mWantsSpinUp;
        mWantsShoot = false;
        mGotSpunUp = false;
        mWantsPreShot = false;
    }

    public synchronized void setManualZoom(boolean zoom) {
        mManualZoom = zoom;
    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return true;
    }
}
