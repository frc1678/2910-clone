package com.team1678.frc2021;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team1323.lib.util.InterpolatingDouble;
import com.team1323.lib.util.InterpolatingTreeMap;
// import com.team1678.frc2021.subsystems.Limelight.LimelightConstants;
import com.team1678.lib.util.SwerveModuleConstants;
import com.team1678.frc2021.subsystems.ServoMotorSubsystem;
import com.team1678.frc2021.subsystems.Limelight.LimelightConstants;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team2910.lib.math.RigidTransform2;
import com.team2910.lib.math.Rotation2;
import com.team2910.lib.math.Vector2;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;

public class Constants {
	/*All distance measurements are in inches, unless otherwise noted.*/

	public static final double kLooperDt = 0.02;
	
	public static final double kEpsilon = 0.0001;
	public static double kSwerveRotationEncoderResolution = 4096;
	
	public static final boolean kIsUsingCompBot = true;
	public static final boolean kIsUsingTractionWheels = true;

	public static final boolean kDebuggingOutput = true;

	// Control Board
	public static final double kJoystickThreshold = 0.2;
	public static final int kButtonGamepadPort = 1;

	//Physical Robot Dimensions (including bumpers)
	public static final double kRobotWidth = 24;
	public static final double kRobotLength = 24;
	public static final double kRobotHalfWidth = kRobotWidth / 2.0;
	public static final double kRobotHalfLength = kRobotLength / 2.0;
	public static final double kRobotProbeExtrusion = 4.0;

	public static final double kBallRadius = 6.5;

	public static final int kCanifierId = 24;

	//Drivetrain
	public static final double kDriveSteeringKp = 0.3;
	public static final double kDriveSteeringKi = 0.0;
	public static final double kDriveSteeringKd = 0.0;

	public static final double kDriveTranslationKp = 0.05;
	public static final double kDriveTranslationKi = 0.0;
	public static final double kDriveTranslationKd = 0.0;
	
	 // Indexer
	 public static final int kLowerBeamBreak = 1;      // TODO Find actual port
	 public static final int kUpperBeamBreak = 2;      // TODO Find actual port
	 public static final int kIndexerLimitSwitch = 6;
 
	 public static final double kIndexerKp = 0.2;
	 public static final double kIndexerKi = 0.;
	 public static final double kIndexerKd = 0.;
	 public static final double kIndexerKf = .05;
	 public static final double kIndexerVelocityKp = 0.05;
	 public static final double kIndexerVelocityKi = 0.;
	 public static final double kIndexerVelocityKd = 0.;
	 public static final double kIndexerVelocityKf = .05;
	 public static final int kIndexerMaxVelocity = 20000; // ticks / 100ms, TODO test for actual value
	 public static final int kIndexerMaxAcceleration = 40000; // ticks / 100ms / sec, TODO test for actual value
 
	 public static final int kIndexerSlots = 5;
	 public static final int kDistancePerSlot = 360 / kIndexerSlots;		// TODO Find Actual Distance
	 public static final int kTotalDistance = 5000;                         // TODO Find Actual Distance

	 public static final int kIndexerId = 15;

	 public static final double kZoomingVelocity = 80.;

	 public static final boolean[] kFullSlots = {true, true, true };
	 public static final boolean[] kEmptySlots = {false, false, false };

	// shooter
	public static final int kMasterFlywheelID = 2;
	public static final int kSlaveFlywheelID = 16;
	public static final int kOverheadRollerID = 1;

	public static final double kShooterFlywheelP = 0.07;
	public static final double kShooterFlywheelI = 0.0;
	public static final double kShooterFlywheelD = 0.0;
	public static final double kShooterFlywheelF = 0.073;

	public static final double kShooterOverheadP = 0.05;
	public static final double kShooterOverheadI = 0.0;
	public static final double kShooterOverheadD = 0.0;
	public static final double kShooterOverheadF = 0.053;
	
	// hood
	public static final int kHoodID = 6;
	public static final int kHoodAbsoluteEncoderID = 3;
	public static final double kHoodEncoderOffset = 11.93; //  17122 ticks // 35.45; // degrees
	public static final boolean kHoodInvertMotor = false;
	public static final double kHoodGearRatio = (2048.0 * 85.3) / 360.0;
	public static final double kHoodP = 0.05;
	public static final double kHoodI = 0.0;
	public static final double kHoodD = 0.0;
	public static final double kHoodF = 0.05;
	public static final double kHoodMinLimit = 15.00; // TODO: Check value with absolute encoder
	public static final double kHoodMaxLimit = 80.00; // TODO: Check value with absolute encoder
	public static final double kHoodCruiseVelocity = 10;
	public static final double kHoodCruiseAcceleration = 10;
	public static final double kHoodDeadband = 0;

	
	// limelight
	 public static final LimelightConstants kLimelightConstants = new LimelightConstants();
	 static {
		 kLimelightConstants.kName = "Limelight";
		 kLimelightConstants.kTableName = "limelight";
		 kLimelightConstants.kHeight = 24.5; // inches
		 kLimelightConstants.kHorizontalPlaneToLens = Rotation2d.fromRadians(34.0);
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

	 public static final double kGoalHeight = 90.0;
	 public static final double kInnerGoalDepth = 29.25;
	 public static final double kInnerGoalToApex = 16.92;
	 public static final double kInnerTargetRangeAngle = Math.toRadians(10.0); 


	 public static final double TARGET_HEIGHT = 98.25;
	 public static final double LIMELIGHT_HEIGHT = 24.1;

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

	public static int kMasterIntakeRollerId = 5;
	public static int kSlaverIntakeRollerId = 11; //TODO Check real ID
	public static int kDeployOutSolenoidId = 7;
	public static int kDeployInSolenoidId = 0;
	public static int kShiftSolenoidId = 5;
	public static int kDeployId = 31;

	// Falcons
	public static final int kFrontRightRotationId = 9;
	public static final int kFrontRightDriveId = 13;

	public static final int kFrontLeftRotationId = 8;
	public static final int kFrontLeftDriveId = 3;
	
	public static final int kBackLeftRotationId = 5;
	public static final int kBackLeftDriveId = 4;
	
	public static final int kBackRightRotationId = 12;
	public static final int kBackRightDriveId = 16;

	// Intake
	public static final int kClimbSolenoidId = 5;
	public static final int kPCMId = 20;

	public static Solenoid makeSolenoidForId(int solenoidId) {
        if (solenoidId < 8) {
            return new Solenoid(kPCMId, solenoidId);
        }
        throw new IllegalArgumentException("Solenoid ID not valid: " + solenoidId);
	}
	
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

	// Swerve
	public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 25;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(15.5);
        public static final double wheelBase = Units.inchesToMeters(15.5);
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = 6.86; //6.86:1
		public static final double angleGearRatio = 12.8; //12.8:1
		
		/* Controller Invert */
        public static final boolean invertXAxis = false;
        public static final boolean invertYAxis = false;
        public static final boolean invertRAxis = false;

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new edu.wpi.first.wpilibj.geometry.Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new edu.wpi.first.wpilibj.geometry.Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new edu.wpi.first.wpilibj.geometry.Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new edu.wpi.first.wpilibj.geometry.Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.22;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0001;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.5;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0001;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; //meters per second
        public static final double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 23;
            public static final double angleOffset = 105;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 13;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 20;
            public static final double angleOffset = 237;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 22;
            public static final double angleOffset = 317;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 14;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 21;
            public static final double angleOffset = 91;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
	}
	
	public static final class SnapConstants {
        public static final double snapKP = 5;
        public static final double snapKI = 0;
        public static final double snapKD = 0;
        public static final double snapTimeout = 0.25;
        public static final double snapEpsilon = 1.0;

        //Constraints for the profiled angle controller (vals stolen from AutoConstants)
        public static final double kMaxAngularSpeedRadiansPerSecond = 4*Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.pow(kMaxAngularSpeedRadiansPerSecond, 2);

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
				kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
				
		public static final TrajectoryConfig defaultConfig = 
			new TrajectoryConfig(
					Constants.AutoConstants.kMaxSpeedMetersPerSecond,
					Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
				.setKinematics(Constants.Swerve.swerveKinematics);
	  }
	  
}
