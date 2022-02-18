// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OI;

import java.util.Map;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Vision.AngleSolver;
import frc.robot.Vision.GetTarget;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.RawContoursV2;
import frc.robot.commands.Shooter.EndHubLog;
import frc.robot.commands.Shooter.LogHubTarget;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

/** Add your docs here. */
public class HubVisionShuffleboard {

        private HttpCamera LLFeed;

        public HubVisionShuffleboard(LimeLight ll, RawContoursV2 rCV2, AngleSolver as, GetTarget getTarget,
                        RevTurretSubsystem turret,
                        RevTiltSubsystem tilt, RevShooterSubsystem shooter, boolean isMatch) {

                // /**
                // *
                // * Vision
                // *
                // */

                if (rCV2.lookForTarget && !isMatch) {

                        ShuffleboardLayout contourPX = Shuffleboard.getTab("HubVision")
                                        .getLayout("allXY", BuiltInLayouts.kList).withPosition(0, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "TOP")); //

                        contourPX.addString("LtoRTx", () -> rCV2.getLCRTx());
                        contourPX.addString("LtoRTy", () -> rCV2.getLCRTy());
                        contourPX.addString("LtoRTxAngle", () -> as.getLCRTxAngle());
                        contourPX.addString("LtoRTyAngle", () -> as.getLCRTyAngle());
                        contourPX.addString("LtoRArea", () -> rCV2.getLCRArea());
                        contourPX.addNumber("AreaRatLR", () -> rCV2.getLRAreaRatio());
                        contourPX.addNumber("TX", () -> ll.get("tx"));
                        contourPX.addNumber("TY", () -> ll.get("ty"));

                        ShuffleboardLayout contourDist = Shuffleboard.getTab("HubVision")
                                        .getLayout("Dist", BuiltInLayouts.kList).withPosition(4, 0)
                                        .withSize(2, 2).withProperties(Map.of("Label position", "TOP")); //

                        contourDist.addNumber("LDist", () -> as.leftDistance);
                        contourDist.addNumber("CDist", () -> as.centerDistance);
                        contourDist.addNumber("RDist", () -> as.rightDistance);

                        ShuffleboardLayout testContourPX = Shuffleboard.getTab("HubVision")
                                        .getLayout("testX", BuiltInLayouts.kList).withPosition(2, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "TOP")); //

                        testContourPX.addNumber("Test Con#", () -> rCV2.getTestContourNumber());
                        testContourPX.addNumber("Test Tx", () -> rCV2.getTestTargetTx());
                        testContourPX.addNumber("Test Ty", () -> rCV2.getTestTargetTy());
                        testContourPX.addNumber("Test Tx Angle", () -> as.getTestTxAngle());
                        testContourPX.addNumber("Test Ty Angle", () -> as.getTestTyAngle());

                        testContourPX.addNumber("Test Area", () -> rCV2.getTestTargetArea());

                }

                if (getTarget.getTarget &&!isMatch) {
                        ShuffleboardLayout targetValues = Shuffleboard.getTab("HubVision")
                                        .getLayout("bullseye", BuiltInLayouts.kList).withPosition(8, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "TOP")); // labels)

                        targetValues.add("StartLog",
                                        new LogHubTarget(as, getTarget, tilt, turret, ll));
                        targetValues.add("StopLog", new EndHubLog(as));

                        targetValues.addNumber("AreaAngle", () -> getTarget.getTargetAngle());
                        targetValues.addNumber("AreaX", () -> getTarget.getTargetX());

                }

                if (RobotBase.isReal() && !isMatch) {

                        LLFeed = new HttpCamera("limelight",
                                        "http://limelight.local:5800/stream.mjpg");
                        ShuffleboardTab driverDisplayTab = Shuffleboard.getTab("HubVision");
                        driverDisplayTab.add("Limelight", LLFeed)
                                        .withWidget(BuiltInWidgets.kCameraStream).withPosition(4, 2)
                                        .withSize(4, 2).withProperties(Map.of("Show Crosshair", false,
                                                        "Show Controls", true));//

                }
        }

}
