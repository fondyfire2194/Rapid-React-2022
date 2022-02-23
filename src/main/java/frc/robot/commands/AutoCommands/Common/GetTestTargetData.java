// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Vision.RawContoursV2;
import frc.robot.Vision.VisionReferenceTarget;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GetTestTargetData extends InstantCommand {
  private RawContoursV2 m_rcv2;
  private VisionReferenceTarget m_vrt;
  public GetTestTargetData(VisionReferenceTarget vt) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_vrt=vt;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      
    m_vrt.getTestTargetData();
  }
}
