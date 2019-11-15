package frc.robot.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Vector2;

import static org.frcteam2910.common.robot.Constants.CAN_TIMEOUT_MS;

public class SwerveModule4415Mk1 extends SwerveModule {
    /**
     * NEO / SparkMAX for drive
     * 775 / Talon SRX for rotation with CRTE mag encoder
     * Hall effect sensor limit wired to Spark for full revolution detection
     * future: hall effect wired to talon
     */

    /*
     * The default drive encoder rotations per unit.
     * TODO: update this too
     */
    // public static final double DEFAULT_DRIVE_ROTATIONS_PER_UNIT = (1.0 / (4.0 * Math.PI)) * (60.0 / 15.0) * (18.0 / 26.0) * (42.0 / 14.0);
    public static final double DEFAULT_DRIVE_ROTATIONS_PER_UNIT = 0.2;

    /* TODO: update this */
    public static final double STEERING_ENCODER_TICKS_PER_ROTATION = 15018.67;

    // TODO: Update this
    // private static final PidConstants ANGLE_CONSTANTS = new PidConstants(0.5, 0.0, 0.0001);
    private static final PidConstants ANGLE_CONSTANTS = new PidConstants(0.1, 0.0, 0.0);

    private static final double CAN_UPDATE_RATE = 50.0;

    private final double angleOffset;
    private final double ticksOffset;

    //private Spark steeringMotor;
    private TalonSRX steeringMotor;
    private CANDigitalInput angleResetLimit;
    private CANDigitalInput otherLimtiSwith;
    //private AnalogInput angleEncoder;

    private CANSparkMax driveMotor;
    private CANEncoder driveEncoder;

    private final Object canLock = new Object();
    private double driveDistance = 0.0;
    private double drivePercentOutput = 0.0;
    private double driveVelocity = 0.0;
    private double driveCurrent = 0.0;

    private double driveEncoderRotationsPerUnit = DEFAULT_DRIVE_ROTATIONS_PER_UNIT;

    /*
     * All CAN operations are done in a separate thread to reduce latency on the control thread
     */
    private Notifier canUpdateNotifier = new Notifier(() -> {
        double driveRotations = driveEncoder.getPosition();
        synchronized (canLock) {
            driveDistance = driveRotations * (1.0 / driveEncoderRotationsPerUnit);
        }

        double driveRpm = driveEncoder.getVelocity();
        synchronized (canLock) {
            driveVelocity = driveRpm * (1.0 / 60.0) * (1.0 / driveEncoderRotationsPerUnit);
        }

        double localDriveCurrent = driveMotor.getOutputCurrent();
        synchronized (canLock) {
            driveCurrent = localDriveCurrent;
        }

        double localDrivePercentOutput;
        synchronized (canLock) {
            localDrivePercentOutput = drivePercentOutput;
        }
        driveMotor.set(localDrivePercentOutput);
    });

    private PidController angleController = new PidController(ANGLE_CONSTANTS);

    /**
     * @param modulePosition The module's offset from the center of the robot's center of rotation
     * @param angleOffset    An angle in radians that is used to offset the angle encoder
     * @param angleMotor     The motor that controls the module's angle
     * @param driveMotor     The motor that drives the module's wheel
     * @param angleEncoder   The analog input for the angle encoder
     */
    public SwerveModule4415Mk1(Vector2 modulePosition, double angleOffsetRadians,
                           TalonSRX angleMotor, CANSparkMax driveMotor) {
        super(modulePosition);
        // initialize member variables
        this.angleOffset = angleOffsetRadians;
        this.ticksOffset = angleOffsetRadians;
        this.steeringMotor = angleMotor;
        // this.angleEncoder = angleEncoder;
        this.driveMotor = driveMotor;
        this.driveEncoder = new CANEncoder(driveMotor);
        
        // angle reset hall effect sensor is wired to the spark 
        // TODO: normally closed or open
        this.angleResetLimit = new CANDigitalInput(
                driveMotor, 
                CANDigitalInput.LimitSwitch.kForward, 
                CANDigitalInput.LimitSwitchPolarity.kNormallyOpen
        );
        
        // disable the limit switch for the spark drive
        this.angleResetLimit.enableLimitSwitch(false);
        // TODO: normally closed or open
        this.otherLimtiSwith = new CANDigitalInput(
                driveMotor, 
                CANDigitalInput.LimitSwitch.kReverse, 
                CANDigitalInput.LimitSwitchPolarity.kNormallyOpen
        );
        this.otherLimtiSwith.enableLimitSwitch(false);

        // config drive motor 
        this.driveMotor.setSmartCurrentLimit(60);
        this.driveMotor.enableVoltageCompensation(12);
        this.driveMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
        this.driveMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
        this.driveMotor.setIdleMode(IdleMode.kBrake);
        
        // Config angle motor
        this.steeringMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, CAN_TIMEOUT_MS);
        this.steeringMotor.setSelectedSensorPosition(0);
        this.steeringMotor.configFeedbackNotContinuous(true, CAN_TIMEOUT_MS);
        this.steeringMotor.setSensorPhase(false);
        this.steeringMotor.config_kP(0, 30, CAN_TIMEOUT_MS);
        this.steeringMotor.config_kI(0, 0.001, CAN_TIMEOUT_MS);
        this.steeringMotor.config_kD(0, 200, CAN_TIMEOUT_MS);
        this.steeringMotor.config_kF(0, 0, CAN_TIMEOUT_MS);
        this.steeringMotor.setNeutralMode(NeutralMode.Brake);
 

        angleController.setInputRange(0.0, 2.0 * Math.PI);
        angleController.setContinuous(true);
        angleController.setOutputRange(-0.5, 0.5);

        canUpdateNotifier.startPeriodic(1.0 / CAN_UPDATE_RATE);
    }

    @Override
    protected double readAngle() {
        /*
        double angle = (1.0 - angleEncoder.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI + angleOffset;
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle; */

        // 1.0 - 
        
        SmartDashboard.putNumber(String.format("%s Encoder Postion", getName()), steeringMotor.getSelectedSensorPosition(0));
        
        double angle = ( 1 / STEERING_ENCODER_TICKS_PER_ROTATION ) 
                        * ( steeringMotor.getSelectedSensorPosition() % STEERING_ENCODER_TICKS_PER_ROTATION ) 
                        * 2.0 * Math.PI 
                        + angleOffset;
        angle %= (2.0 * Math.PI);
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }

    @Override
    protected double readDistance() {
        synchronized (canLock) {
            return driveDistance;
        }
    }

    protected double readVelocity() {
        synchronized (canLock) {
            return driveVelocity;
        }
    }

    protected double readDriveCurrent() {
        double localDriveCurrent;
        synchronized (canLock) {
            localDriveCurrent = driveCurrent;
        }

        return localDriveCurrent;
    }

    @Override
    public double getCurrentVelocity() {
        return readVelocity();
    }

    @Override
    public double getDriveCurrent() {
        return readDriveCurrent();
    }

    @Override
    protected void setTargetAngle(double angle) {
        angleController.setSetpoint(angle);
    }

    @Override
    protected void setDriveOutput(double output) {
        synchronized (canLock) {
            this.drivePercentOutput = output;
        }
    }

    @Override
    public void updateState(double dt) {
        super.updateState(dt);
        //steeringMotor.set(angleController.calculate(getCurrentAngle(), dt));
        steeringMotor.set(ControlMode.PercentOutput, angleController.calculate(getCurrentAngle(), dt));
    }

    public void setDriveEncoderRotationsPerUnit(double driveEncoderRotationsPerUnit) {
        synchronized (canLock) {
            this.driveEncoderRotationsPerUnit = driveEncoderRotationsPerUnit;
        }
    }

    public boolean isSteeringAtLimit() {
        return this.angleResetLimit.get() || this.otherLimtiSwith.get();
    }

    public boolean resetToMagnet()  {
        if (this.isSteeringAtLimit()) {
            this.steeringMotor.set(ControlMode.PercentOutput, 0);
            return true;
        }
        else {
            this.steeringMotor.set(ControlMode.PercentOutput, 0.1);
            return false;
        }
    }
}
