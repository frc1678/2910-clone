package com.team1678.frc2021.controlboard;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.controlboard.CustomXboxController.Button;
import com.team1678.frc2021.controlboard.CustomXboxController.Side;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Deadband;
import com.team254.lib.util.DelayedBoolean;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class GamepadButtonControlBoard {
    private final double kDeadband = 0.15;

    private int mDPadUp = -1;
    private int mDPadDown = -1;
    private int mDPadRight = -1;
    private int mDPadLeft = -1;

    private final double kDPadDelay = 0.02;
    private DelayedBoolean mDPadValid;

    private static GamepadButtonControlBoard mInstance = null;

    public static GamepadButtonControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadButtonControlBoard();
        }

        return mInstance;
    }

    private final CustomXboxController mController;

    private GamepadButtonControlBoard() {
        mController = new CustomXboxController(Constants.kButtonGamepadPort);
    }

    public double getJogHood() {
        double jog = mController.getJoystick(CustomXboxController.Side.LEFT, CustomXboxController.Axis.Y);
        if (Deadband.inDeadband(jog, kDeadband)) {
            return 0.0;
        }
        return (jog - kDeadband * Math.signum(jog));
    }

    public boolean getWantHoodScan() {
        return mController.getButton(CustomXboxController.Button.L_JOYSTICK);
    }

    public boolean getTestSpit() {
        return mController.getController().getStickButtonReleased(Hand.kRight);
    }

    public void setRumble(boolean on) { //TODO: all 5 power cells indexed
        mController.setRumble(on);
    }

    /*
    public boolean getSpinUp() {
        return mController.getController().getAButtonPressed();
    }
    */

    public boolean getTuck() {
        return mController.getButton(Button.X);
    }

    public boolean getUntuck() {
        return mController.getButton(Button.START);
    }

    public boolean getShoot() {
        return mController.getController().getYButtonPressed();
    }

    public boolean getPreShot() {
        return mController.getController().getAButtonPressed();
    }
    
    public boolean getIntake() {
        return mController.getTrigger(CustomXboxController.Side.RIGHT);
    } 
    

    // Intake
    public boolean getRunIntake() {
        return mController.getTrigger(Side.RIGHT);
    }

    public boolean getRetractIntake() {
        return mController.getTrigger(Side.LEFT);
    }

}
