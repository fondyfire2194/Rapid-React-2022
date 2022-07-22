/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands.RobotDrive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.RevDrivetrain;

/**
 * A command that will turn the robot to the specified angle using a motion
 * profile.
 */
public class TurnToHideAngle extends PIDCommand {

  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive              The drive subsystem to use
   */

  Debouncer inPosnDebouncer = new Debouncer(.5, DebounceType.kRising);

  public TurnToHideAngle(RevDrivetrain drive) {

    super(
        new PIDController(drive.kTurnP, drive.kTurnI, drive.kTurnD),
        // Close loop on heading
        drive::getHeading,
        // Set reference to target
        drive::getHideTurnAngle,
        // Pipe output to turn robot
        output -> drive.rotate(-output),
        // Require the drive
        drive);

    getController().setSetpoint(drive.getHideTurnAngle()); // Set the controller to be continuous
                                                    // (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is
    // stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);
    getController().setIntegratorRange(-1., 1.);

  }

  @Override
  public void initialize() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    getController().reset();
    SmartDashboard.putNumber("ROBTRN", getController().getSetpoint());
    

  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return inPosnDebouncer.calculate(getController().atSetpoint());

  }
}