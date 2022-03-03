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

import frc.robot.Vision.LimeLight;
import frc.robot.Vision.RawContoursV2;
import frc.robot.Vision.Set2XZoomValues;
import frc.robot.Vision.SetNoZoomValues;
import frc.robot.Vision.VisionReferenceTarget;
import frc.robot.commands.AutoCommands.Common.CalculateTarget;
import frc.robot.commands.AutoCommands.Common.CalculateTestTarget;
import frc.robot.commands.AutoCommands.Common.AcquireTarget;
import frc.robot.commands.AutoCommands.Common.GetNumberOfContourValues;
import frc.robot.commands.AutoCommands.Common.GetVisionValues;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

/** Add your docs here. */
public class HubVisionShuffleboard {

        private HttpCamera LLFeed;

        public HubVisionShuffleboard(LimeLight ll, RawContoursV2 rcv2,
                        VisionReferenceTarget vrt, RevTurretSubsystem turret,
                        RevTiltSubsystem tilt, RevShooterSubsystem shooter, boolean isMatch) {

                // /**
                // *
                // * Vision
                // * && !isMatch
                // */

                if (true) {

                        ShuffleboardLayout contourPX = Shuffleboard.getTab("HubVision")
                                        .getLayout("allXY", BuiltInLayouts.kList).withPosition(0, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "TOP")); //

                        contourPX.addString("LtoRTx", () -> rcv2.getLCRTx());
                        contourPX.addString("LtoRTy", () -> rcv2.getMedLCRTy());
 
                        contourPX.addString("LtoRTxMed", () -> rcv2.getMedLCRTx());
                        contourPX.addString("LtoRTyAngle", () -> rcv2.getLCRTyAngle());
                        contourPX.addString("LtoRTxMedAngle", () -> rcv2.getLCRTxMedAngle());
            
                        // contourPX.addNumber("AreaRatLR", () -> rcv2.getLRAreaRatio());
                        contourPX.addNumber("TX", () -> ll.get("tx"));
                        contourPX.addNumber("TY", () -> ll.get("ty"));

        
                        ShuffleboardLayout contourDist = Shuffleboard.getTab("HubVision")
                                        .getLayout("Dist", BuiltInLayouts.kList).withPosition(4, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "TOP")); //

                        //contourDist.add("GetVisionData", new GetVisionValues(rcv2));

                        contourDist.add("Get6", new GetNumberOfContourValues(rcv2));

                        contourDist.add("CalcTestTarget", new CalculateTestTarget(vrt));

                        contourDist.add("CalcTarget", new CalculateTarget(rcv2));

                        contourDist.add("SetNoZoom", new SetNoZoomValues(rcv2, ll));

                        contourDist.add("Set2XZoom", new Set2XZoomValues(rcv2, ll));
                
                        contourDist.add("AcquireTarget", new AcquireTarget(ll, tilt, turret, rcv2, shooter));


                        contourPX.addString("LtoRMedArea", () -> rcv2.getLCRMedianArea());

                        ShuffleboardLayout testContourPX = Shuffleboard.getTab("HubVision")
                                        .getLayout("testX", BuiltInLayouts.kList).withPosition(2, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "TOP")); //

                        testContourPX.addNumber("Test Con#", () -> vrt.getTestContourNumber());
                        testContourPX.addNumber("Test Tx", () -> vrt.getTestTargetTx());
                        testContourPX.addNumber("Test Ty", () -> vrt.getTestTargetTy());
                        testContourPX.addNumber("Test Tx Angle", () -> vrt.getTestTxAngle());
                        testContourPX.addNumber("Test Ty Angle", () -> vrt.getTestTyAngle());

                        testContourPX.addNumber("Test Area", () -> vrt.getTestTargetArea());

                }
                // &&!isMatch
                if (true) {
                        ShuffleboardLayout targetValues = Shuffleboard.getTab("HubVision")
                                        .getLayout("bullseye", BuiltInLayouts.kList).withPosition(8, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "TOP")); // labels)

                
                        targetValues.addNumber("AreaX", () -> rcv2.targetValue);
                        targetValues.addNumber("AreaAngle", () -> rcv2.targetAngle);
                     
                        targetValues.addNumber("WeightedX", () -> rcv2.weightedTargetValue);
                        targetValues.addNumber("WeightedAngle", () -> rcv2.weightedTargetAngle);
                        

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
