// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RobotDrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RevDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SaveGetSavedPose extends InstantCommand {
  private RevDrivetrain m_drive;

  private int m_choice;

  public SaveGetSavedPose(RevDrivetrain drive, int choice) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_choice = choice;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    switch (m_choice) {

      case 0:
        m_drive.saveCurrentPose();

        break;

      case 1:
        SmartDashboard.putString("SavedPose", m_drive.getSavedPose().toString());

        break;

      case 2:

        SmartDashboard.putString("CurrentPose", m_drive.getPose().toString());

        break;

      case 3:

        m_drive.setCurrentPose(m_drive.getSavedPose());

        break;

      default:
        break;
    }
  }
}
