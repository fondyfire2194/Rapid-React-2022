// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Pref;

/** Add your docs here. */
public class GetTarget {

  // private double x_1;
  // private double x_2;
  // private double x_3;
  // private double y_1;
  // private double y_2;
  // private double y_3;;
  // private double a;
  // private double b;
  // private double c;
  // private double h;
  // private double k;

  private int targetValue = 0;
  private double targetAngle;

  public boolean getTarget = true;

  public boolean pxtoaworld;

  private RawContoursV2 m_rCV2;
  private AngleSolver m_as;

  private int x;

  public double quadAngle;
  public double matthewMethod;

  public GetTarget(RawContoursV2 rcV2, AngleSolver as) {
    m_rCV2 = rcV2;
    m_as = as;
  }

  public void runCalcs() {
    getTarget = true;
  //  getContourData();
    targetValue = calculateTargetX();
    targetAngle = getTargetAngle(targetValue);

    getTarget = false;

  }

  // public void getContourData() {

  //   x_1 = m_rCV2.getLeftTx();
  //   x_2 = m_rCV2.getCenterTx();
  //   x_3 = m_rCV2.getRightTx();
  //   y_1 = m_rCV2.getLeftTy();
  //   y_2 = m_rCV2.getCenterTy();
  //   y_3 = m_rCV2.getRightTy();

  // }
  // private int matthewMethodGetX() {
  //   double leftArea = m_rCV2.getLeftArea();
  //   double rightArea = m_rCV2.getRightArea();

  //   double secondArea = Math.max(leftArea, rightArea);
  //   double centerArea = m_rCV2.getCenterArea();

  //   int centerX = m_rCV2.getCenterTx();
  //   int secondX = m_rCV2.getLeftTx();
  //   double side = 1;
  //   if (secondArea == rightArea) {
  //     secondX = m_rCV2.getRightTx();
  //     side = -1;
  //   }

  //   int differenceOfX = Math.abs(centerX-secondX);
  //   double ratio = ((centerArea+secondArea) / 2) / centerArea;
  //   int target = (int)(secondX + (differenceOfX * ratio * side));
  //   return target;

  // }


  public int calculateTargetX() {

    double leftArea = m_rCV2.getLeftArea();
    double rightArea = m_rCV2.getRightArea();

    double secondArea = Math.max(leftArea, rightArea);
    double centerArea = m_rCV2.getCenterArea();

    double ratio = (centerArea - secondArea) / centerArea;
    SmartDashboard.putNumber("target r", ratio);

    int secondX = m_rCV2.getLeftTx();
    if (secondArea == rightArea) {
      secondX = m_rCV2.getRightTx();
    }

    int halfOffset = (m_rCV2.getCenterTx() - secondX) / 2;
    double offsetX = halfOffset * ratio * Pref.getPref("HubTgtGn");
    int midpoint = (secondX + m_rCV2.getCenterTx()) / 2;
    int targetX = midpoint + (int) offsetX;
    // int targetX = (int)(Math.abs(centerX-secondX)*(ratio)*side + (centerX));
    // int targetX = (int) (centerX*ratio + secondX*(1-ratio));
    // int targetX = (int) (centerX * ratio);

    return targetX;
  }

  public double getTargetAngle(int xValue) {
    double xAsNx = (xValue - 159.5) / 160;

    double temp = 0;

    double x = 0;
    double nx = 0;
    double ax = 0;// rads

    nx = xAsNx;
    x = nx * (m_as.vpw / 2);
    ax = Math.atan2(1, x);

    temp = Units.radiansToDegrees(ax);
    temp -= 90;
    return -temp;
  }

  
  // private int[] solveQuadratic() {

  //   // solving for a,b,c in y = ax^2 + bx + c
  //   a = y_1 / ((x_1 - x_2) * (x_1 - x_3)) + y_2 / ((x_2 - x_1) * (x_2 - x_3))
  //       + y_3 / ((x_3 - x_1) * (x_3 - x_2));

  //   b = -y_1 * (x_2 + x_3) / ((x_1 - x_2) * (x_1 - x_3)) - y_2 * (x_1 + x_3) / ((x_2 - x_1) * (x_2 - x_3))
  //       - y_3 * (x_1 + x_2) / ((x_3 - x_1) * (x_3 - x_2));

  //   c = y_1 * x_2 * x_3 / ((x_1 - x_2) * (x_1 - x_3)) + y_2 * x_1 * x_3 / ((x_2 - x_1) * (x_2 - x_3))
  //       + y_3 * x_1 * x_2 / ((x_3 - x_1) * (x_3 - x_2));

  //   // solving for the vertex h, k
  //   h = -b / (2 * a);
  //   k = -1 * ((b * b) - (4 * a * c)) / (4 * a);

  //   SmartDashboard.putNumber("Center X", h);
  //   SmartDashboard.putNumber("Center Y", k);

  //   if (h < 0)
  //     h = 0;
  //   if (h > 320)
  //     h = 319;
  //   if (k < 0)
  //     k = 0;
  //   if (k > 240)
  //     k = 239;
  //   return new int[] { (int) h, (int) k };

  // }

  // public double getCenterX() {
  //   return h;
  // }

  // public double getCenterY() {
  //   return k;
  // }

  // public double getC() {
  //   return h;
  // }

  // public double getAVert() {
  //   return a;
  // }

  // public double getBVert() {
  //   return b;
  // }

  // public double getCVert() {
  //   return c;
  // }

  public int getTargetX() {
    return targetValue;

  }

  public double getTargetAngle() {
    return targetAngle;
  }
}
