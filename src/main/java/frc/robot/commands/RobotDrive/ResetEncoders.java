/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.RobotDrive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RevDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ResetEncoders extends InstantCommand {
  private final RevDrivetrain drive;

  public ResetEncoders(RevDrivetrain m_robotDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = m_robotDrive;
    // addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetEncoders();
    drive.leftTargetPosition = 0;
    drive.rightTargetPosition = 0;
  }
}
