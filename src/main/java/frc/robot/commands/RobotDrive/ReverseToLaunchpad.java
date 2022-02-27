/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.RobotDrive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pref;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;

public class ReverseToLaunchpad extends CommandBase {
  /**
   * Creates a new DriveStraightJoystick.
   */
  private final RevDrivetrain m_drive;
  private double startAngle;
  private DoubleSupplier forward;
  private DoubleSupplier rotate;
  private IntakesSubsystem m_intake;

  public ReverseToLaunchpad(RevDrivetrain drive, DoubleSupplier forward, DoubleSupplier rotate, IntakesSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_intake = intake;
    this.forward = forward;
    this.rotate = rotate;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if ( m_intake.rearll.getIsTargetFound() && forward.getAsDouble() < 0) {

      m_drive.arcadeDrive(forward.getAsDouble(), (m_intake.rearll.getdegRotationToTarget()) * Pref.getPref("dRPuKp"));

    }

    else {
      m_drive.arcadeDrive(forward.getAsDouble(), rotate.getAsDouble());

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
