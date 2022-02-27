// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class VisionReferenceTarget {

    private RawContoursV2 m_rcv2;

    private LimeLight m_ll;

    public int testTargetIndex;

    private int testTargetTx;

    private int testTargetTy;

    private double testTargetTa;

    public boolean testTargetInUse = true;

    double testTxVPValue;

    double testTyVPValue;

    public double[] testAngles = { 0, 0 };

    public VisionReferenceTarget(RawContoursV2 rcv2, LimeLight ll) {
        m_rcv2 = rcv2;
        m_ll = ll;

    }

    public void getTestTargetData() {

         testTargetIndex = 0;
        testTxVPValue = 0;
        testTxVPValue = 0;
        testAngles[0] = 0;
        testAngles[1] = 0;
        testTargetTx = 0;
        testTargetTy = 0;
        testTargetTa = 0;
        testTargetIndex = getLowestContourIndex();

        if (testTargetIndex > 4) {

            testTxVPValue = getTesttxvp(testTargetIndex);

            testTxVPValue = getTesttyvp(testTargetIndex);

          //  testAngles = getTestTargetVPAngles();



            testTargetTx = (int) (((1 + (m_ll.get(m_rcv2.horCoord + String.valueOf(testTargetIndex)))) / 2)
                    * m_rcv2.active_IMG_WIDTH);

            testTargetTy = (int) (((1 + (m_ll.get(
                    m_rcv2.vertCoord + String.valueOf(testTargetIndex)))) / 2)
                    * m_rcv2.active_IMG_HEIGHT);


                    testTargetTa = 1000. * m_ll.get("ta" + String.valueOf(testTargetIndex));
 
 testAngles[0]= m_rcv2.getTargetAngle(testTargetTx);
 
                }
    }

    private int getLowestContourIndex() {

        double temp = 0;
        double min = m_rcv2.active_IMG_HEIGHT;
        int lowest = 0;
        int i = 0;

        for (i = 0; i < m_rcv2.maxPossibleContours; i++) {
            temp = (((1 + (m_ll.get(m_rcv2.vertCoord + String.valueOf(i)))) / 2) * m_rcv2.active_IMG_WIDTH);
            if (temp < min) {
                min = temp;
                lowest = i;
            }
        }

        return lowest;
    }

    private double getTesttxvp(int contour) {
        double temp = 0;

        double vpw2 = m_rcv2.active_vpw / 2;

        temp = vpw2 * m_ll.get(m_rcv2.horCoord + String.valueOf(contour));

        return temp;
    }

    private double getTesttyvp(int contour) {
        double temp = 0;
        double vph2 = m_rcv2.active_vph / 2;

        temp = vph2 * m_ll.get(m_rcv2.vertCoord + String.valueOf(contour));

        return temp;
    }

    public double[] getTestTargetVPAngles() {

        double[] temp = { 0, 0 };

        double x = 0;

        double ax = 0;// rads

        x = testTxVPValue;

        SmartDashboard.putNumber("testtxvp", x);

        ax = Math.atan2(1, x);

        temp[0] = Units.radiansToDegrees(ax);

        double y = testTyVPValue;

        double ay = Math.atan2(1, y);

        temp[1] = Units.radiansToDegrees(ay);

        temp[0] -= 90;

        temp[0] = -Math.round(temp[0] * 1000) / 1000.;

        temp[1] -= 90;

        temp[1] = -Math.round(temp[1] * 1000) / 1000.;

        return temp;
    }

    public double getTestTxAngle() {
        return testAngles[0];
    }

    public double getTestTyAngle() {
        return testAngles[1];
    }

    private double getnX(double x) {
        return ((x - 159.5)) / 160;
    }

    public int getTestContourNumber() {
        return testTargetIndex;
    }

    public int getTestTargetTx() {
        return testTargetTx;
    }

    public int getTestTargetTy() {
        return testTargetTy;
    }

    public double getTestTargetArea() {
        return testTargetTa;
    }

}