package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.GenericHID.Hand;
//import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import org.frcteam2910.c2019.RobotMap;
//import org.frcteam2910.c2019.commands.HolonomicDriveCommand;

import frc.robot.RobotMap;
import frc.robot.drivers.SwerveModule4415Mk1;
import frc.robot.subsystems.Superstructure;
import frc.robot.Robot;

import org.frcteam2910.common.control.*;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.subsystems.SwerveDrivetrain;
import org.frcteam2910.common.util.DrivetrainFeedforwardConstants;
import org.frcteam2910.common.util.HolonomicDriveSignal;
import org.frcteam2910.common.util.HolonomicFeedforward;
import org.frcteam2910.common.robot.Utilities;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import org.frcteam2910.common.robot.commands.ZeroFieldOrientedCommand;
import org.frcteam2910.common.robot.input.DPadButton;
import org.frcteam2910.common.robot.input.XboxController;

import java.util.Optional;

public class DrivetrainSubsystem extends SwerveDrivetrain {
    /* 
    * Track width is the distance between the center of 2 tires mounted on same axle
    * Wheel base is the distance between the front and rear axles of a vehicle
    */
    private static final double TRACKWIDTH = 14;
    private static final double WHEELBASE = 14.5;

    // TODO: update. I (Josh) assumes this is Forward velocity * strafe horizontal velocity.....
    private static final double MAX_VELOCITY = 7.0 * 7.0;

    /* TODO: update constraints */
    public static final ITrajectoryConstraint[] CONSTRAINTS = {
            new MaxVelocityConstraint(MAX_VELOCITY),
            new MaxAccelerationConstraint(8.0 * 7.0),
            new CentripetalAccelerationConstraint(20.0 * 7.0)
    };

    // TODO: update offsets from magnet. care of units. it's degrees.
    private static final double FRONT_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(0);
    private static final double FRONT_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(0);
    private static final double BACK_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(0);
    private static final double BACK_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(0);

    
    private static final double FRONT_LEFT_ANGLE_OFFSET_PRACTICE = Math.toRadians(-170.2152486947372);
    private static final double FRONT_RIGHT_ANGLE_OFFSET_PRACTICE = Math.toRadians(-43.55619048306742);
    private static final double BACK_LEFT_ANGLE_OFFSET_PRACTICE = Math.toRadians(-237.47063008637048);
    private static final double BACK_RIGHT_ANGLE_OFFSET_PRACTICE = Math.toRadians(-336.70093128378477);

    /* TODO: update PID constants
    * 
    * private static final PidConstants FOLLOWER_TRANSLATION_CONSTANTS = new PidConstants(0.05, 0.01, 0.0);
    * private static final PidConstants FOLLOWER_ROTATION_CONSTANTS = new PidConstants(0.2, 0.01, 0.0);
    * private static final HolonomicFeedforward FOLLOWER_FEEDFORWARD_CONSTANTS = new HolonomicFeedforward(
    *         new DrivetrainFeedforwardConstants(1.0 / (14.0 * 12.0), 0.0, 0.0)
    * );
    */
    private static final PidConstants FOLLOWER_TRANSLATION_CONSTANTS = new PidConstants(0.0, 0.0, 0.0);
    private static final PidConstants FOLLOWER_ROTATION_CONSTANTS = new PidConstants(0.0, 0.0, 0.0);
    private static final HolonomicFeedforward FOLLOWER_FEEDFORWARD_CONSTANTS = new HolonomicFeedforward(
            new DrivetrainFeedforwardConstants(0.0, 0.0, 0.0)
    );

    /* TODO: Update these constants too. 
    * private static final PidConstants SNAP_ROTATION_CONSTANTS = new PidConstants(0.3, 0.01, 0.0);
    */
    private static final PidConstants SNAP_ROTATION_CONSTANTS = new PidConstants(0.0, 0.0, 0.0);

    private static final DrivetrainSubsystem instance = new DrivetrainSubsystem();

    private SwerveModule4415Mk1[] swerveModules;

    private HolonomicMotionProfiledTrajectoryFollower follower = new HolonomicMotionProfiledTrajectoryFollower(
            FOLLOWER_TRANSLATION_CONSTANTS,
            FOLLOWER_ROTATION_CONSTANTS,
            FOLLOWER_FEEDFORWARD_CONSTANTS
    );

    private PidController snapRotationController = new PidController(SNAP_ROTATION_CONSTANTS);
    private double snapRotation = Double.NaN;

    private double lastTimestamp = 0;

    private final Object lock = new Object();
    private HolonomicDriveSignal signal = new HolonomicDriveSignal(Vector2.ZERO, 0.0, false);
    private Trajectory.Segment segment = null;

    private boolean isCalibrating = false;

    private DrivetrainSubsystem() {
        double frontLeftAngleOffset = FRONT_LEFT_ANGLE_OFFSET_COMPETITION;
        double frontRightAngleOffset = FRONT_RIGHT_ANGLE_OFFSET_COMPETITION;
        double backLeftAngleOffset = BACK_LEFT_ANGLE_OFFSET_COMPETITION;
        double backRightAngleOffset = BACK_RIGHT_ANGLE_OFFSET_COMPETITION;
       // if (Superstructure.getInstance().isPracticeBot()) {
       //     frontLeftAngleOffset = FRONT_LEFT_ANGLE_OFFSET_PRACTICE;
       //     frontRightAngleOffset = FRONT_RIGHT_ANGLE_OFFSET_PRACTICE;
       //     backLeftAngleOffset = BACK_LEFT_ANGLE_OFFSET_PRACTICE;
       //     backRightAngleOffset = BACK_RIGHT_ANGLE_OFFSET_PRACTICE;
       // }

       // SwerveModule4415Mk1(Vector2 modulePosition, double angleOffset,
         //                  TalonSRX angleMotor, CANSparkMax driveMotor, AnalogInput angleEncoder);

        //SwerveModule4415Mk1 frontLeftModule1 = new SwerveModule4415Mk1(new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0), frontLeftAngleOffset, new TalonSRX(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR), driveMotor, angleEncoder);
        
        SwerveModule4415Mk1 frontLeftModule = new SwerveModule4415Mk1(
                new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                frontLeftAngleOffset,
                //new Spark(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR),
                new TalonSRX(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR),
                new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless)
        );
        frontLeftModule.setName("Front Left");

        SwerveModule4415Mk1 frontRightModule = new SwerveModule4415Mk1(
                new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                frontRightAngleOffset,
                new TalonSRX(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR),
                new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless)
        );
        frontRightModule.setName("Front Right");

        SwerveModule4415Mk1 backLeftModule = new SwerveModule4415Mk1(
                new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                backLeftAngleOffset,
                new TalonSRX(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR),
                new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless)
        );
        backLeftModule.setName("Back Left");

        SwerveModule4415Mk1 backRightModule = new SwerveModule4415Mk1(
                new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                backRightAngleOffset,
                new TalonSRX(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR),
                new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless)
        );
        backRightModule.setName("Back Right");

        swerveModules = new SwerveModule4415Mk1[]{
                frontLeftModule,
                frontRightModule,
                backLeftModule,
                backRightModule,
        };

        snapRotationController.setInputRange(0.0, 2.0 * Math.PI);
        snapRotationController.setContinuous(true);
        snapRotationController.setOutputRange(-0.2, 0.2);
    }

    public void setSnapRotation(double snapRotation) {
        synchronized (lock) {
            this.snapRotation = snapRotation;
        }
    }

    public void stopSnap() {
        synchronized (lock) {
            this.snapRotation = Double.NaN;
        }
    }

    @Override
    public void holonomicDrive(Vector2 translation, double rotation, boolean fieldOriented) {
        synchronized (lock) {
            this.signal = new HolonomicDriveSignal(translation, rotation, fieldOriented);
        }
    }

    @Override
    public synchronized void updateKinematics(double timestamp) {
        super.updateKinematics(timestamp);

        double dt = timestamp - lastTimestamp;
        lastTimestamp = timestamp;

        double localSnapRotation;
        synchronized (lock) {
            localSnapRotation = snapRotation;
        }
        RigidTransform2 currentPose = new RigidTransform2(
                getKinematicPosition(),
                getGyroscope().getAngle()
        );

        Optional<HolonomicDriveSignal> optSignal = follower.update(currentPose, getKinematicVelocity(),
                getGyroscope().getRate(), timestamp, dt);
        HolonomicDriveSignal localSignal;

        if (optSignal.isPresent()) {
            localSignal = optSignal.get();

            synchronized (lock) {
                segment = follower.getLastSegment();
            }
        } else {
            synchronized (lock) {
                localSignal = this.signal;
            }
        }

        //if (false) {
        if (Math.abs(localSignal.getRotation()) < 0.1 && Double.isFinite(localSnapRotation)) {
            snapRotationController.setSetpoint(localSnapRotation);

            localSignal = new HolonomicDriveSignal(localSignal.getTranslation(),
                    snapRotationController.calculate(getGyroscope().getAngle().toRadians(), dt),
                    localSignal.isFieldOriented());
        } else {
            synchronized (lock) {
                snapRotation = Double.NaN;
            }
        }

        //HolonomicDriveSignal testSignal = new HolonomicDriveSignal(new Vector2(0.0, 0.0), 0.0, false);

        super.holonomicDrive(localSignal.getTranslation(), localSignal.getRotation(), localSignal.isFieldOriented());
        //super.holonomicDrive(testSignal.getTranslation(), testSignal.getRotation(), testSignal.isFieldOriented());
    }

    @Override
    public void outputToSmartDashboard() {
        super.outputToSmartDashboard();

        HolonomicDriveSignal localSignal;
        Trajectory.Segment localSegment;
        synchronized (lock) {
            localSignal = signal;
            localSegment = segment;
        }

        SmartDashboard.putNumber("Drivetrain Follower Forwards", localSignal.getTranslation().x);
        SmartDashboard.putNumber("Drivetrain Follower Strafe", localSignal.getTranslation().y);
        SmartDashboard.putNumber("Drivetrain Follower Rotation", localSignal.getRotation());
        SmartDashboard.putBoolean("Drivetrain Follower Field Oriented", localSignal.isFieldOriented());

        if (follower.getCurrentTrajectory().isPresent() && localSegment != null) {
            SmartDashboard.putNumber("Drivetrain Follower Target Angle", localSegment.rotation.toDegrees());

            Vector2 position = getKinematicPosition();

            SmartDashboard.putNumber("Drivetrain Follower X Error", localSegment.translation.x - position.x);
            SmartDashboard.putNumber("Drivetrain Follower Y Error", localSegment.translation.y - position.y);
            SmartDashboard.putNumber("Drivetrain Follower Angle Error", localSegment.rotation.toDegrees() - getGyroscope().getAngle().toDegrees());
        }

        SmartDashboard.putBoolean("Drivetrain is Calibrating to Magnets", DrivetrainSubsystem.getInstance().isCalibrating);

        for (SwerveModule4415Mk1 module : swerveModules) {
            SmartDashboard.putNumber(String.format("%s Module Drive Current Draw", module.getName()), module.getDriveCurrent());
            SmartDashboard.putBoolean(String.format("%s Module is at Steering Reset Limit", module.getName()), module.isSteeringAtLimit());
        }
    }

    public static DrivetrainSubsystem getInstance() {
        return instance;
    }

    @Override
    public SwerveModule[] getSwerveModules() {
        return swerveModules;
    }

    @Override
    public Gyroscope getGyroscope() {
        return Superstructure.getInstance().getGyroscope();
    }

    @Override
    public double getMaximumVelocity() {
        return 0;
    }

    @Override
    public double getMaximumAcceleration() {
        return 0;
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new HolonomicDriveCommand());
    }

    @Override
    public void stop() {
        super.stop();
        synchronized (lock) {
            snapRotation = Double.NaN;
        }
    }

    public HolonomicMotionProfiledTrajectoryFollower getFollower() {
        return follower;
    }

    public void resetModulesToMagnet() {

    }

    public void resetDriveKinematics() {
        for (SwerveModule4415Mk1 module : swerveModules) {
            module.resetKinematics();
        }
    }
}