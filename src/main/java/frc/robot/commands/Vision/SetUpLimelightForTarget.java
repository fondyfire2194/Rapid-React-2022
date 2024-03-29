// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.LimelightControlMode.LedMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetUpLimelightForTarget extends ParallelCommandGroup {
  /** Creates a new SetUpLimelightForTarget. */

  public SetUpLimelightForTarget(LimeLight limelight,int pipeline, boolean on) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new LimelightLeds(limelight, LedMode.kpipeLine),
        new LimelightSetPipeline(limelight, pipeline), new UseVision(limelight, on));
  }
}
