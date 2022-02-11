// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class AngleSolver {

    private RawContoursV2 m_rcv2;

    private LimeLight m_ll;

    private double HFOV = 59.6;
    private double VFOV = 45.7;
    double vpw = 2 * Math.tan(Units.degreesToRadians(HFOV / 2));
    private double vph = 2 * Math.tan(Units.degreesToRadians(VFOV / 2));
    private double[] lrTxAngles = new double[3];

    private double[] lrTyAngles = new double[3];

    private double[] testAngles = { 0, 0 };

    private double HUB_VISION_DIAMETER = Units.inchesToMeters(53.375);
    private double HUB_CIRCUMFERENCE = Math.PI * HUB_VISION_DIAMETER;
    private double VISION_TAPE_LENGTH = Units.inchesToMeters(5);
    private double VISION_TAPE_WIDTH = Units.inchesToMeters(2);
    private double VISION_TAPE_GAP_LENGTH = Units.inchesToMeters(5.5);

    private double ANGLE_BETWEEN_SAME_POINTS_OF_VISION_TAPE = 360 / 16;

    private double VISION_TAPE_GAP_CONTAINED_ANGLE = VISION_TAPE_GAP_LENGTH * 360 / HUB_CIRCUMFERENCE;

    private double VISION_TAPE_WIDTH_CONTAINED_ANGLE = VISION_TAPE_LENGTH * 360 / HUB_CIRCUMFERENCE;

    private double VISION_TAPE_BOTTOM_HEIGHT = Units.inchesToMeters(101.675);

    private double CAMERA_HEIGHT = Units.inchesToMeters(30);

    private double CAMERA_ANGLE_TO_HORIZONTAL = 10;

    private double targetDistance;

    public boolean endLog;

    public SimpleCSVLogger hubLogger;

    public boolean lock3Contours;

    public boolean logInProgress;

    public double leftDistance;

    public double centerDistance;

    public double rightDistance;

    public double[] rightXY = { 0, 0 };

    public double[] leftXY = { 0, 0 };

    public double angleShift;

    public AngleSolver(RawContoursV2 rcv2, LimeLight ll) {

        m_rcv2 = rcv2;
        m_ll = ll;
        hubLogger = new SimpleCSVLogger();
    }

    /**
     * Angle calculations are based on the LimeLight documentation
     * 
     * See the section on turning pixels into angles
     * 
     * Angles are used by the tilt and turret to move to and lock on target
     * 
     * 
     * 
     * @return
     */
    public void runValues() {

        lrTxAngles = getTxAngles();

        lrTyAngles = getTyAngles();

        testAngles = getTestTargetAngles();

        leftDistance = getTargetDistance(getTxAngles()[0]);
        centerDistance = getTargetDistance(getTxAngles()[1]);
        rightDistance = getTargetDistance(getTxAngles()[2]);

        rightXY = getTargetXY(m_rcv2.getRightTx(), rightDistance);
        leftXY = getTargetXY(m_rcv2.getLeftTx(), leftDistance);

        angleShift = getAngleShift(rightXY, leftXY);

    }

    private double[] getTxAngles() {

        double[] temp = { 0, 0, 0 };

        double x = 0;
        double nx = 0;
        double ax = 0;// rads

        nx = m_ll.get(m_rcv2.horCoord + String.valueOf(m_rcv2.areasLRTxIndex[0]));
        x = nx * (vpw / 2);

        ax = Math.atan2(1, x);
        temp[0] = Units.radiansToDegrees(ax);

        nx = m_ll.get(m_rcv2.horCoord + String.valueOf(m_rcv2.areasLRTxIndex[1]));
        x = nx * (vpw / 2);
        ax = Math.atan2(1, x);
        SmartDashboard.putNumber("mxxxx", ax);
        SmartDashboard.putNumber("mnxxxx", nx);

        temp[1] = Units.radiansToDegrees(ax);

        nx = m_ll.get(m_rcv2.horCoord + String.valueOf(m_rcv2.areasLRTxIndex[2]));
        x = nx * (vpw / 2);
        ax = Math.atan2(1, x);

        temp[2] = Units.radiansToDegrees(ax);

        temp[0] -= 90;
        temp[1] -= 90;
        temp[2] -= 90;

        temp[0] = Math.round(temp[0] * 1000) / 1000.;
        temp[1] = Math.round(temp[1] * 1000) / 1000.;
        temp[2] = Math.round(temp[2] * 1000) / 1000.;

        return temp;
    }

    private double[] getTyAngles() {
        double[] temp = { 0, 0, 0 };

        double y = 0;
        double ny = 0;
        double ay = 0;// rads

        ny = m_ll.get(m_rcv2.vertCoord + String.valueOf(m_rcv2.areasLRTxIndex[0]));
        y = ny * (vph / 2);
        ay = Math.atan2(1, y);
        temp[0] = Units.radiansToDegrees(ay);

        ny = m_ll.get(m_rcv2.horCoord + String.valueOf(m_rcv2.areasLRTxIndex[1]));
        y = ny * (vph / 2);
        ay = Math.atan2(1, y);
        temp[1] = Units.radiansToDegrees(ay);

        ny = m_ll.get(m_rcv2.vertCoord + String.valueOf(m_rcv2.areasLRTxIndex[2]));
        y = ny * (vph / 2);
        ay = Math.atan2(1, y);
        temp[2] = Units.radiansToDegrees(ay);

        temp[0] -= 90;
        temp[1] -= 90;
        temp[2] -= 90;

        temp[0] = Math.round(temp[0] * 1000) / 1000;
        temp[1] = Math.round(temp[1] * 1000) / 1000;
        temp[2] = Math.round(temp[2] * 1000) / 1000;

        return temp;
    }

    private double[] getTestTargetAngles() {

        double[] temp = { 0, 0 };

        double x = 0;
        double nx = 0;
        double ax = 0;// rads

        nx = m_ll.get("tx" + String.valueOf(m_rcv2.testTargetIndex));

        x = nx * (vpw / 2);

        ax = Math.atan2(1, x);

        temp[0] = Units.radiansToDegrees(ax);

        temp[0] -= 90;

        temp[0] = Math.round(temp[0] * 1000) / 1000.;

        double ny = m_ll.get("ty" + String.valueOf(m_rcv2.testTargetIndex));
        double y = ny * (vph / 2);

        double ay = Math.atan2(1, y);

        temp[1] = Units.radiansToDegrees(ay);

        temp[1] -= 90;

        temp[1] = Math.round(temp[1] * 1000) / 1000.;

        return temp;
    }

    private double[] getRawCrosshairAngles() {
        double[] temp = { 0, 0 };
        // double cx = m_ll.get("cx0");
        double cx = -.1;// m_ll.get("tx0");
        double c = cx * (vpw / 2);
        SmartDashboard.putNumber("c", c);
        double acx = Math.atan2(1, c);
        temp[0] = Units.radiansToDegrees(acx);
        temp[0] -= 90;

        // double cy = m_ll.get("cy0");

        double cy = -.5;
        c = cy * (vph / 2);
        double acy = Math.atan2(1, c);
        temp[1] = Units.radiansToDegrees(acy);
        if (temp[1] > 90)
            temp[1] -= 90;
        return temp;

    }

    // METERS
    private double getTargetDistance(double ty) {

        double distance = 0;
        double h1 = CAMERA_HEIGHT;
        double h2 = VISION_TAPE_BOTTOM_HEIGHT + VISION_TAPE_WIDTH / 2;
        double a1 = Units.degreesToRadians(ty);
        double a2 = Units.degreesToRadians(CAMERA_ANGLE_TO_HORIZONTAL);

        distance = (h2 - h1) / Math.tan(a1 + a2);

        return distance;
    }

    private double[] getTargetXY(double tx, double distance) {
        double[] temp = { 0, 0 };
        temp[0] = distance * Math.sin(Units.degreesToRadians(tx));
        temp[1] = distance * Math.cos(Units.degreesToRadians(tx));
        return temp;
    }

    private double getAngleShift(double[] rpt, double[] lpt) {

        double xDiff = rpt[0] - lpt[0];// opp side
        double yTotal = rpt[1] + lpt[1];// adj side
        double temp = Math.atan(xDiff / yTotal);
        return Units.radiansToDegrees(temp);

    }

    public double getLeftArea() {
        return m_rcv2.getLeftArea();
    }

    public double getCenterArea() {
        return m_rcv2.getCenterArea();
    }

    public double getRightArea() {
        return m_rcv2.getRightArea();
    }

    public double getTestTargetArea() {
        return m_rcv2.getTestTargetArea();
    }

    public void setCLock3Contours(boolean on) {
        m_rcv2.setLockContours(on);
    }

    public double getTestTxAngle() {
        return testAngles[0];
    }

    public double getTestTyAngle() {
        return testAngles[1];
    }

    public double getLeftTxAngle() {
        return lrTxAngles[0];
    }

    public double getCenterTxAngle() {
        return lrTxAngles[1];
    }

    public double getRightTxAngle() {
        return lrTxAngles[2];
    }

    public double getLeftTyAngle() {
        return lrTyAngles[0];
    }

    public double getCenterTyAngle() {
        return lrTyAngles[1];
    }

    public double getRightTyAngle() {
        return lrTyAngles[2];
    }

    public String getLCRTxAngle() {

        return String.valueOf(getLeftTxAngle() + " ," + String.valueOf(getCenterTxAngle()) + " ,"
                + String.valueOf(getRightTxAngle()));
    }

    public String getLCRTyAngle() {
        return String.valueOf(getLeftTyAngle() + " ," + String.valueOf(getCenterTyAngle()) + " ,"
                + String.valueOf(getRightTyAngle()));

    }

    public double getTargetDistance() {
        return targetDistance;
    }

}
