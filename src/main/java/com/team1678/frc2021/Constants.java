package com.team1678.frc2021;

import java.util.Arrays;
import java.util.List;

import com.team1323.lib.util.InterpolatingDouble;
import com.team1323.lib.util.InterpolatingTreeMap;
import com.team1678.frc2021.subsystems.Limelight.LimelightConstants;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team2910.lib.math.RigidTransform2;
import com.team2910.lib.math.Rotation2;
import com.team2910.lib.math.Vector2;

import edu.wpi.first.wpilibj.Solenoid;

public class Constants {
	/*All distance measurements are in inches, unless otherwise noted.*/

	public static final double kLooperDt = 0.02;
	
	public static final double kEpsilon = 0.0001;
	public static double kSwerveRotationEncoderResolution = 4096;
	
	public static final boolean kIsUsingCompBot = true;
	public static final boolean kIsUsingTractionWheels = true;

	public static final boolean kDebuggingOutput = true;
	
	//Physical Robot Dimensions (including bumpers)
	public static final double kRobotWidth = 24;
	public static final double kRobotLength = 24;
	public static final double kRobotHalfWidth = kRobotWidth / 2.0;
	public static final double kRobotHalfLength = kRobotLength / 2.0;
	public static final double kRobotProbeExtrusion = 4.0;

	public static final double kBallRadius = 6.5;

	public static final int kCanifierId = 23;

	//Drivetrain
	public static final double kDriveSteeringKp = 0.3;
	public static final double kDriveSteeringKi = 0.0;
	public static final double kDriveSteeringKd = 0.0;

	public static final double kDriveTranslationKp = 0.05;
	public static final double kDriveTranslationKi = 0.0;
	public static final double kDriveTranslationKd = 0.0;
	

	 // Indexer
	 public static final int kSlot0Proxy = 1;
	 public static final int kSlot1Proxy = 2;
	 public static final int kSlot2Proxy = 3;
	 public static final int kSlot3Proxy = 4;
	 public static final int kSlot4Proxy = 5;
	 public static final int kIndexerLimitSwitch = 6;
 
	 public static final double kIndexerKp = 0.2;
	 public static final double kIndexerKi = 0.;
	 public static final double kIndexerKd = 0.;
	 public static final double kIndexerKf = .05;
	 public static final double kIndexerVelocityKp = 0.05;
	 public static final double kIndexerVelocityKi = 0.;
	 public static final double kIndexerVelocityKd = 0.;
	 public static final double kIndexerVelocityKf = .05;
	 public static final int kIndexerMaxVelocity = 20000; // ticks / 100ms
	 public static final int kIndexerMaxAcceleration = 40000; // ticks / 100ms / sec
 
	 public static final int kIndexerSlots = 5;
	 public static final int kAnglePerSlot = 360 / kIndexerSlots;
	 public static final double kIndexerDeadband = 2.0; // degrees

	 public static final int kIndexerId = 7; 

	// shooter
	public static final int kMasterFlywheelID = 14;
	public static final int kSlaveFlywheelID = 15;
	public static final int kTriggerWheelID = 10;
	public static final int kTriggerPopoutSolenoidID = 6;
	public static final double kShooterP = 0.2;
	public static final double kShooterI = 0.00004;
	public static final double kShooterD = 0.0;
	public static final double kShooterF = 0.05;
	public static final double kTriggerP = 0.05;
	public static final double kTriggerI = 0.0;
	public static final double kTriggerD = 0.0;
	public static final double kTriggerF = 0.05;

	public static final double kTriggerRPM = 5000.0;

	// limelight
	 public static final LimelightConstants kLimelightConstants = new LimelightConstants();
	 static {
		 kLimelightConstants.kName = "Limelight";
		 kLimelightConstants.kTableName = "limelight";
		 kLimelightConstants.kHeight = 24.5; // inches
		 kLimelightConstants.kTurretToLens = Pose2d.identity();
		 kLimelightConstants.kHorizontalPlaneToLens = Rotation2d.fromDegrees(0.0);
	 }

	 public static final double kHorizontalFOV = 59.6; // degrees
	 public static final double kVerticalFOV = 49.7; // degrees
	 public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
	 public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
	 public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds
 
	 public static final double kMaxTrackerDistance = 15.0;
	 public static final double kMaxGoalTrackAge = 30.0;
	 public static final double kMaxGoalTrackAgeNotTracking = 0.3;
	 public static final double kMaxGoalTrackSmoothingTime = 1.5;
	 public static final double kTrackStabilityWeight = 0.0;
	 public static final double kTrackAgeWeight = 10.0;
	 public static final double kTrackSwitchingWeight = 100.0;
	 public static final boolean kEnableCachedGoal = true;
	 
	 public static final double kCameraFrameRate = 90.0;
	 public static final double kMinStability = 0.5;
	 public static final int kPortPipeline = 0;
	 public static final int kBallPipeline = 2;
	 public static final double kGoalHeight = 90.0;

	 public static final double kInnerGoalDepth = 0;
	 public static final double kHoodToTurret = 4.25; // center of the turret to the axis of rotation of the hood
	 public static final double kLimelightPitchOffset = 17.66; // limelight pitch at hood 0
	 public static final double kAutoAimPredictionTime = 4.0; // lookahead for robot state during aiming


	public static final Vector2 kVehicleToTurretTranslation = new Vector2(3.75, 0);

	public static final double kHoodRadius = 11.904; // radius of hood
 
	
	//Field Landmarks
	public static final RigidTransform2 kRobotLeftStartingPose = new RigidTransform2(new Vector2(48.0 + kRobotHalfLength, 97.0 + kRobotHalfWidth - 162.0), Rotation2.fromDegrees(0));
	public static final RigidTransform2 kRobotRightStartingPose = new RigidTransform2(new Vector2(48.0 + kRobotHalfLength, -(97.0 + kRobotHalfWidth - 162.0)), Rotation2.fromDegrees(0));
	public static final RigidTransform2 kRobotLeftRampExitPose = new RigidTransform2(new Vector2(95.25 + kRobotHalfLength, 97.0 + kRobotHalfWidth - 162.0), Rotation2.fromDegrees(0));
	public static final RigidTransform2 kRobotRightRampExitPose = new RigidTransform2(new Vector2(95.25 + kRobotHalfLength, -(97.0 + kRobotHalfWidth - 162.0)), Rotation2.fromDegrees(0));
	
	public static final double kAutoInverse = 1.0;
	public static final RigidTransform2 kSlalomStartingPose = new RigidTransform2(new Vector2(40.0, -65.0 * kAutoInverse), Rotation2.fromDegrees(0.0));
	public static final RigidTransform2 kAutoNavStartingPose = new RigidTransform2(new Vector2(46.0, 0.0 * kAutoInverse), Rotation2.fromDegrees(0.0));
	public static final RigidTransform2 kGSRedAStartingPose = new RigidTransform2(new Vector2(40.0, 0.0 * kAutoInverse), Rotation2.fromDegrees(0.0));
	public static final RigidTransform2 kGSRedBStartingPose = new RigidTransform2(new Vector2(40.0, 35.0 * kAutoInverse), Rotation2.fromDegrees(0.0));


	//Swerve Calculations Constants (measurements are in inches)
    public static final double kWheelbaseLength = 11;
    public static final double kWheelbaseWidth  = 11;
    public static final double kSwerveDiagonal = Math.hypot(kWheelbaseLength, kWheelbaseWidth);
    
    //Camera Constants
    public static final double kCameraYOffset = 0.0;//0.25
    public static final double kCameraXOffset = kRobotHalfLength;
    public static final double kCameraZOffset = 46.3;
    public static final double kCameraYawAngleDegrees = 0.0;//-12.7
    public static final double kCameraPitchAngleDegrees = -33.3;//13.45
    
    //Goal tracker constants
    public static double kTrackReportComparatorStablityWeight = 1.0;
	public static double kTrackReportComparatorAgeWeight = 1.0;
	public static final double kDefaultCurveDistance = kRobotHalfLength + 36.0;
	public static final double kVisionUpdateDistance = kRobotHalfLength + 75.0;
	public static final double kVisionDistanceStep = 4.0;
	public static final double kClosestVisionDistance = 26.0;//36.0
	public static final double kDefaultVisionTrackingSpeed = 42.0;
	public static final double kCurvedVisionYOffset = 0.375;//1.25

	//Vision Speed Constraint Treemap
	public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kVisionSpeedTreemap = new InterpolatingTreeMap<>();

	public static boolean kDebugSwerve = true;

	public static double kSwerveRotationMotorTicksPerRotation = 2048 * 12.4;

	public static int kIntakeRollerId = 3;
	public static int kDeploySolenoidId = 4;
	public static int kShiftSolenoidId = 5;
	public static int kDeployId = 31;

	// Falcons
	public static final int kFrontRightRotationId = 9;
	public static final int kFrontRightDriveId = 13;

	public static final int kFrontLeftRotationId = 8;
	public static final int kFrontLeftDriveId = 4;
	
	public static final int kBackLeftRotationId = 5;
	public static final int kBackLeftDriveId = 1;
	
	public static final int kBackRightRotationId = 12;
	public static final int kBackRightDriveId = 16;

	// Absolute Encoders (CANCoder)
	public static final int kFrontRightEncoderId = 0;
	public static final int kFrontLeftEncoderId = 1;
	public static final int kBackLeftEncoderId = 2;
	public static final int kBackRightEncoderId = 3;

	// Intake
	public static final int kClimbSolenoidId = 5;
	public static final int kPCMId = 20;

	public static Solenoid makeSolenoidForId(int solenoidId) {
        if (solenoidId < 8) {
            return new Solenoid(kPCMId, solenoidId);
        }
        throw new IllegalArgumentException("Solenoid ID not valid: " + solenoidId);
	}
	
	// Climber
	public static final int kClimberId = 3;

	static{
		kVisionSpeedTreemap.put(new InterpolatingDouble(-6.0), new InterpolatingDouble(24.0));
		kVisionSpeedTreemap.put(new InterpolatingDouble(kClosestVisionDistance), new InterpolatingDouble(24.0));
		kVisionSpeedTreemap.put(new InterpolatingDouble(60.0), new InterpolatingDouble(48.0));
		kVisionSpeedTreemap.put(new InterpolatingDouble(300.0), new InterpolatingDouble(48.0));
	}
    
    //Path following constants
    public static final double kPathLookaheadTime = 0.2;  // seconds to look ahead along the path for steering 0.4
	public static final double kPathMinLookaheadDistance = 15.0;  // inches 24.0 (we've been using 3.0)
    
    //Swerve Speed Constants
    // public static final double kSwerveDriveMaxSpeed = 68808.0 * 1.3;
    // public static final double kSwerveMaxSpeedInchesPerSecond = 15.5 * 12.0;
	// public static final double kSwerveRotationMaxSpeed = 1250.0 * 0.8 * 1.6; //The 0.8 is to request a speed that is always achievable
	// public static final double kSwerveRotation10VoltMaxSpeed = 1350.0;
	// public static final double kSwerveRotationSpeedScalar = ((1.0 / 0.125) - 1.0) / kSwerveMaxSpeedInchesPerSecond;
	
	public static final double kSwerveDriveMaxSpeed = 56000.0;
    public static final double kSwerveMaxSpeedInchesPerSecond = 15.5 * 12.0;
	public static final double kSwerveRotationMaxSpeed  = 21000.0 * .8; //The 0.8 is to request a speed that is always achievable
	public static final double kSwerveRotation10VoltMaxSpeed = 14000.0;
    public static final double kSwerveRotationSpeedScalar = ((1.0 / 0.125) - 1.0) / kSwerveMaxSpeedInchesPerSecond;;
    
    //Swerve Module Wheel Offsets (Rotation encoder values when the wheels are facing 0 degrees)
	public static final int kFrontRightEncoderStartingPos = 45;
	public static final int kFrontLeftEncoderStartingPos = 144;
	public static final int kBackLeftEncoderStartingPos =  289;
	public static final int kBackRightEncoderStartingPos = 62;
	
	//Swerve Module Positions (relative to the center of the drive base)
	public static final Translation2d kVehicleToModuleZero = new Translation2d(kWheelbaseLength/2, -kWheelbaseWidth/2);
	public static final Translation2d kVehicleToModuleOne = new Translation2d(kWheelbaseLength/2, kWheelbaseWidth/2);
	public static final Translation2d kVehicleToModuleTwo = new Translation2d(-kWheelbaseLength/2, kWheelbaseWidth/2);
	public static final Translation2d kVehicleToModuleThree = new Translation2d(-kWheelbaseLength/2, -kWheelbaseWidth/2);
	
	public static final List<Translation2d> kModulePositions = Arrays.asList(kVehicleToModuleZero,
			kVehicleToModuleOne, kVehicleToModuleTwo, kVehicleToModuleThree);
	
	//Scrub Factors
	public static final boolean kSimulateReversedCarpet = false;
	public static final double[] kWheelScrubFactors = new double[]{1.0, 1.0, 1.0, 1.0};
	public static final double kXScrubFactor = 1.0 / (1.0 - (9549.0 / 293093.0));
	public static final double kYScrubFactor = 1.0 / (1.0 - (4.4736 / 119.9336));

	//Voltage-Velocity equation constants {m, b, x-intercept}
	//First set is the positive direction, second set is negative
	public static final double[][][] kVoltageVelocityEquations = new double[][][]{
		{{1.70, -4.39, 2.58}, {1.83, 5.23, -2.85}}, 
		{{1.59, -3.86, 2.42}, {1.43, 3.09, -2.16}}, 
		{{1.53, -3.66, 2.39}, {1.66, 4.15, -2.50}}, 
		{{1.84, -4.70, 2.56}, {1.85, 5.34, -2.89}}};
	
	//Swerve Odometry Constants
	public static final double kSwerveWheelDiameter = 7.667; //inches (actual diamter is closer to 3.87, but secondary algorithm prefers 4.0901) 3.76
	public static final double kSwerveDriveEncoderResolution = 4096.0;
	/** The number of rotations the swerve drive encoder undergoes for every rotation of the wheel. */
	public static final double kSwerveEncoderToWheelRatio = 6.0;
	public static final double kSwerveEncUnitsPerWheelRev = kSwerveDriveEncoderResolution * kSwerveEncoderToWheelRatio;
	public static final double kSwerveEncUnitsPerInch = kSwerveEncUnitsPerWheelRev / (Math.PI * kSwerveWheelDiameter);

	//LED Colors
	public static final List<Double> pink = Arrays.asList(255.0, 20.0, 30.0);
	public static final List<Double> blue = Arrays.asList(0.0, 0.0, 255.0);
	public static final List<Double> red = Arrays.asList(255.0, 0.0, 0.0);
	public static final List<Double> orange = Arrays.asList(255.0, 20.0, 0.0);
	public static final List<Double> yellow = Arrays.asList(255.0, 60.0, 0.0);
	public static final List<Double> green = Arrays.asList(0.0, 255.0, 0.0);
	public static final List<Double> purple = Arrays.asList(255.0, 0.0, 255.0);

	//LED Arrays
	public static final List<List<Double>> rainbow = Arrays.asList(red, orange, yellow, green, blue, pink, purple);

	public static final int kLongCANTimeoutMs = 100;
	public static final int kCANTimeoutMs = 10;
}
