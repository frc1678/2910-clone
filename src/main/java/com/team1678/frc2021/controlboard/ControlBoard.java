package com.team1678.frc2021.controlboard;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.controlboard.GamepadButtonControlBoard;
import com.team254.lib.geometry.Rotation2d;
import com.team1678.frc2021.controlboard.CustomXboxController.Side;

public class ControlBoard {
    private static ControlBoard mInstance = null;

    private CustomXboxController mController;
    
    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }

        return mInstance;
    }

    private final GamepadButtonControlBoard mButtonControlBoard;

    private ControlBoard() {
        mButtonControlBoard = GamepadButtonControlBoard.getInstance();
    }

    public void reset() {
    }
 
    public boolean getRunIntake() {
        return mButtonControlBoard.getRunIntake();
    }

    public boolean getRetractIntake() {
        return mButtonControlBoard.getRetractIntake();
    }

    public double getJogHood() {
        return mButtonControlBoard.getJogHood();
    }

    // Intake

    public boolean getShoot() {
        return mButtonControlBoard.getShoot();
    }

    public boolean getPreShot() {
        return mButtonControlBoard.getPreShot();
    }

    public boolean getSpinUp() {
        return mButtonControlBoard.getSpinUp();
    }

    public boolean getTuck() {
        return mButtonControlBoard.getTuck() /*|| mDriveControlBoard.getTuck()*/;
    }

    public boolean getUntuck() {
        return mButtonControlBoard.getUntuck();
    }
    public void setRumble(boolean on) {
        mButtonControlBoard.setRumble(on);
    }
}