// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.PipelinesConstants;
import frc.robot.Vision.LimeLight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FindTargetFail extends InstantCommand {
  String m_message;
private LimeLight m_ll;
  public FindTargetFail(String message,LimeLight ll) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_message = message;
    m_ll=ll;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    SmartDashboard.putString("Message", m_message);

    m_ll.setPipeline(PipelinesConstants.ledsOffPipeline);
    
    super.cancel();
    
  }
}
