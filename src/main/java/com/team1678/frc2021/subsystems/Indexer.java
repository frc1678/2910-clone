package com.team1678.frc2021.subsystems;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2021.Constants;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;

import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.Util;
import com.team1678.frc2021.planners.IndexerMotionPlanner;

// in case of having a hall effect sensor on the indexer
import com.team1678.lib.util.HallCalibration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Indexer extends Subsystem {

    private static Indexer mInstance = null;
    private IndexerMotionPlanner mMotionPlanner;

    public enum WantedAction {
        NONE, INDEX, PASSIVE_INDEX, PREP, REVOLVE, ZOOM, SLOW_ZOOM, HELLA_ZOOM,
    }

    public enum State {
        IDLE, INDEXING, PASSIVE_INDEXING, PREPPING, REVOLVING, ZOOMING, SLOW_ZOOMING, FEEDING, HELLA_ZOOMING,
    }

    private Indexer() {
    }

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        private boolean[] raw_slots = { false, false, false, false, false };
        public boolean limit_switch;

        public double indexer_angle;
        public double indexer_velocity;
        public double indexer_current;
        public double turret_angle;
        public boolean snapped;

        // OUTPUTS
        public ControlMode indexer_control_mode = ControlMode.PercentOutput;
        public double indexer_demand;
    }
}
