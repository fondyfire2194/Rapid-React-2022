// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RobotDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RevDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetRobotPose extends InstantCommand {
  private RevDrivetrain m_drive;
  private Pose2d m_pose;

  public SetRobotPose(RevDrivetrain drive, Pose2d pose) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_pose = pose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_drive.setCurrentPose(m_pose);
  }
}
