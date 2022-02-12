// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Vision.AngleSolver;
import frc.robot.Vision.RawContoursV2;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://d;ocs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EndHubLog extends InstantCommand {
  AngleSolver m_as;
  public EndHubLog(AngleSolver as) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_as=as;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_as.endLog=true;
  }
}
