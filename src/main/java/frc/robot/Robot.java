/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import org.frcteam2910.c2019.autonomous.AutonomousSelector;
//import org.frcteam2910.c2019.autonomous.AutonomousTrajectories;
//import org.frcteam2910.c2019.subsystems.*;
//import org.frcteam2910.c2019.vision.api.Gamepiece;

import frc.robot.subsystems.*;

import org.frcteam2910.common.robot.drivers.Limelight;
import org.frcteam2910.common.robot.drivers.NavX;
import org.frcteam2910.common.robot.subsystems.SubsystemManager;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final double UPDATE_DT = 5e-3; // 5 ms

    private final SubsystemManager subsystemManager = new SubsystemManager(
            DrivetrainSubsystem.getInstance()
    );

    private static final OI oi = new OI();

    //private AutonomousTrajectories autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem.CONSTRAINTS);
    //private AutonomousSelector autonomousSelector = new AutonomousSelector(autonomousTrajectories);

    private Command autonomousCommand = null;

    public Robot() {
        //oi.bindButtons(autonomousSelector);
    }

    public static OI getOi() {
        return oi;
    }

    @Override
    public void robotInit() {
        SmartDashboard.putBoolean("Limelight Calibration Mode", false);

        subsystemManager.enableKinematicLoop(UPDATE_DT);
    }

    @Override
    public void robotPeriodic() {
        subsystemManager.outputToSmartDashboard();

        SmartDashboard.putNumber("Robot Angle",
                Superstructure.getInstance().getGyroscope().getAngle().toDegrees());
        SmartDashboard.putNumber("Gyro Pitch",
                Math.toDegrees(Superstructure.getInstance().getGyroscope().getAxis(NavX.Axis.ROLL)));
    }

    @Override
    public void teleopInit() {
//        if (autonomousCommand != null) {
//            autonomousCommand.cancel();
//            autonomousCommand = null;
//        }
    }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
      /*  
      if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        autonomousCommand = autonomousSelector.getCommand();
        autonomousCommand.start();
        */
    }

    @Override
    public void autonomousPeriodic() {
        //Scheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
//        if (autonomousCommand != null) {
//            autonomousCommand.cancel();
//            autonomousCommand = null;
//        }
//        Scheduler.getInstance().removeAll();
    }

    @Override
    public void disabledPeriodic() {
    }
}