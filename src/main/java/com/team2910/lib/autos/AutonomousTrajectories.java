package com.team2910.lib.autos;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.team1678.frc2021.DriveMotionPlanner;
import com.team2910.lib.control.*;
import com.team2910.lib.math.RigidTransform2;
import com.team2910.lib.math.Rotation2;
import com.team2910.lib.math.Vector2;
import com.team2910.lib.math.spline.CubicHermiteSpline;
import com.team2910.lib.math.spline.Spline;
import com.team2910.lib.util.InterpolatingDouble;
import com.team2910.lib.util.InterpolatingTreeMap;
import com.team2910.lib.util.Side;

import edu.wpi.first.wpilibj.Timer;

public class AutonomousTrajectories {

    private static final double SAMPLE_DISTANCE = 0.1;

    private static final double kMaxVelocity = 180.0;
    private static final double kMaxAccel = 150.0;
    private static final double kMaxDecel = 72.0;
    private static final double kMaxVoltage = 11.0;

    private static Map<Double, Rotation2> rotationMap = new HashMap<>();


    // Points
    private final RigidTransform2 testPoint1 = new RigidTransform2(new Vector2(0.0, 0.0), Rotation2.ZERO);
    private final RigidTransform2 testPoint2 = new RigidTransform2(new Vector2(10.0, 0.0), Rotation2.ZERO);


    // Trajectories
    private Trajectory testPath;
    
    
    public AutonomousTrajectories() {
        MaxAccelerationConstraint maxAccel = new MaxAccelerationConstraint(kMaxAccel, kMaxDecel);
        MaxVelocityConstraint maxVel = new MaxVelocityConstraint(kMaxVelocity);
        // FeedforwardConstraint maxVolts = new FeedforwardConstraint(kMaxVoltage, kMaxVelocity, kMaxAccel);
        TrajectoryConstraint constraints[] = {maxAccel, maxVel};

        testPath = new Trajectory(
                new SimplePathBuilder(testPoint1.getTranslation(), testPoint1.getRotation())
                        .lineTo(testPoint2.getTranslation(), testPoint2.getRotation())
                        .build(),
                constraints, SAMPLE_DISTANCE
        );
    }


    // Trajectory Methods
    public Trajectory getTestPath() {
        return testPath;
    }
    
}
