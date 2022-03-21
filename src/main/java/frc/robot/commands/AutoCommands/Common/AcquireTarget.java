// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PipelinesConstants;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.RawContoursV2;
import frc.robot.commands.Turret.PositionTurretToVision;
import frc.robot.commands.Turret.TurretWaitForStop;
import frc.robot.commands.Vision.SetUpLimelightForTarget;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AcquireTarget extends SequentialCommandGroup {

        /** Creates a new Acquire target. */

        public AcquireTarget(LimeLight ll, RevTiltSubsystem tilt, RevTurretSubsystem turret, RawContoursV2 rcv2) {
                // Add your commands in the addCommands() call, e.g.
                // addCommands(new FooCommand(), new BarCommand());

                /**
                 * Camera is fixed so should see a target on either or possibly both no and 2x
                 * zoom
                 * Start with 2x and if no target or a target with bounding box height > set
                 * 
                 * value (tbd) then change to no zoom and check.
                 * 
                 * 
                 * 
                 * 
                 */

                addCommands(new SetUpLimelightForTarget(ll, PipelinesConstants.x2Zoom320240, false),

                                new ConditionalCommand(new GetVisionValues(rcv2),

                                                new SetUpLimelightForTarget(ll, PipelinesConstants.noZoom960720,
                                                                false),

                                                () -> ll.getBoundingBoxHeightInX2ZoomRange()),

                                new ConditionalCommand(new GetVisionValues(rcv2), new FindTargetFail("No Target", ll),

                                                () -> ll.getBoundingBoxHeightInNoZoomRange()),

                                new PositionTurretToVision(turret, ll, ll.getdegRotationToTarget()),

                                new TurretWaitForStop(turret), new GetNumberOfContourValues(rcv2),

                                new CalculateTarget(rcv2));

        }

}
