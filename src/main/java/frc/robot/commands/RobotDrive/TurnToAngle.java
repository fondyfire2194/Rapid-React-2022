/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands.RobotDrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Pref;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.RevDrivetrain;

/**
 * A command that will turn the robot to the specified angle using a motion
 * profile.
 */
public class TurnToAngle extends PIDCommand {

  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive              The drive subsystem to use
   */
  public TurnToAngle(RevDrivetrain drive, double targetAngleDegrees) {

    super(
        new PIDController(drive.kTurnP, drive.kTurnI, drive.kTurnD),
        // Close loop on heading
        drive::getHeading,
        // Set reference to target
        targetAngleDegrees,
        // Pipe output to turn robot
        output -> drive.rotate(output),
        // Require the drive
        drive);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is
    // stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);
    getController().setIntegratorRange(-.01, .01);
  }

  // @Override

  // public void execute() {

  // SmartDashboard.putNumber("turnp", getController().getPeriod());

  // if (Math.abs(getController().getPositionError()) >
  // Pref.getPref("dRTurnkizLim"))

  // getController().setI(0);

  // else

  // getController().setI(Pref.getPref("dRTurnkiz"));

  // }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}