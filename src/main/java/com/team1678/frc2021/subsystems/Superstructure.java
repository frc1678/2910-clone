package com.team1678.frc2021.subsystems;

public class Superstructure extends Subsystem {
    // Required instances
    private final Indexer mIndexer = Indexer.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    private final Hood mHood = Hood.getInstance();

    public enum WantedAction {
        IDLE, PREP, SHOOT, CLIMB
    }

    public enum State {
        NONE, PREPPING, SHOOTING, CLIMBING
    }

    

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub

    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub

    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        return false;
    }
    
    
}
