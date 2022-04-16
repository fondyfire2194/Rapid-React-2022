// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.RawContoursV2;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AcquireTarget extends SequentialCommandGroup {

        /**
         * Creates a new Acquire target.
         * 
         * this will be run when about to shoot. A set of 5 readings will be taken of
         * tx,ty and areas
         * 
         * The medians of these values will be taken and used in future decisions
         * 
         * 
         * 
         * 
         * 
         */

        public AcquireTarget(LimeLight ll, RevTiltSubsystem tilt, RevTurretSubsystem turret, RawContoursV2 rcv2) {
                // Add your commands in the addCommands() call, e.g.
                // addCommands(new FooCommand(), new BarCommand());

                addCommands(new GetMedianOfContourValues(rcv2),

                                new SortLtoRData(rcv2),

                                new CalculateTarget(rcv2));

        }

}
