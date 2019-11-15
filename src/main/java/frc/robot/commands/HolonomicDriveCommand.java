/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public class HolonomicDriveCommand extends Command {
    public HolonomicDriveCommand() {
        requires(DrivetrainSubsystem.getInstance());
    }

    @Override
    protected void execute() {
        boolean ignoreScalars = Robot.getOi().primaryController.getLeftBumperButton().get();

        double forward = Robot.getOi().primaryController.getLeftYAxis().get(true);
        double strafe = Robot.getOi().primaryController.getLeftXAxis().get(true);
        double rotation = Robot.getOi().primaryController.getRightXAxis().get(true, ignoreScalars);

        boolean robotOriented = Robot.getOi().primaryController.getXButton().get();
        boolean reverseRobotOriented = Robot.getOi().primaryController.getYButton().get();

        Vector2 translation = new Vector2(forward, strafe);

        if (reverseRobotOriented) {
            robotOriented = true;
            translation = translation.rotateBy(Rotation2.fromDegrees(180.0));
        }

        DrivetrainSubsystem.getInstance().holonomicDrive(translation, rotation, !robotOriented);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}