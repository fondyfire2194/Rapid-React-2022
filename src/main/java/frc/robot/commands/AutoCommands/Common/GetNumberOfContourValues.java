// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Vision.RawContoursV2;


public class GetNumberOfContourValues extends SequentialCommandGroup {
  /** Creates a new VerifyAndGetTarget. */

  public GetNumberOfContourValues(RawContoursV2 rcv2) {

    addCommands(new GetVisionValues(rcv2), new GetVisionValues(rcv2),
        new GetVisionValues(rcv2), new GetVisionValues(rcv2),
        new GetVisionValues(rcv2), new GetVisionValues(rcv2),
        new GetVisionValues(rcv2), new GetVisionValues(rcv2),
        new GetVisionValues(rcv2), new GetVisionValues(rcv2),
        new GetVisionValues(rcv2));

        

  }
}
