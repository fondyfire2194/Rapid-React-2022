// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import java.util.Arrays;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class RawContoursV2 {

    int IMG_WIDTH = 320;
    int IMG_HEIGHT = 240;

    private double HFOV;// = 30;// 59.6;
    private double VFOV;// = 23;// 45.7;
    double vpw;
    double vph;

    private boolean showDebug = false;

    private boolean showData = true;

    // public boolean lock3Contours = false;

    public int[] lTRIndex = { 0, 1, 2 };

    public double[] lToRAreas = { 0, 0, 0 };

    public int[] ltoRTxValues = { 0, 0, 0 };

    private int[] ltoRTyValues = { 0, 0, 0 };

    public double[] lrTxAngles = new double[3];

    public double[] lrTyAngles = new double[3];

    double[] lRTxVpValues = { 0, 0, 0 };

    double[] lRTyVpValues = { 0, 0, 0 };

    int maxPossibleContours = 6;

    public boolean lookForTarget = true;

    private LimeLight m_ll;

    private boolean cameraAt90 = false;

    String horCoord;

    String vertCoord;

    public boolean areasValid;

    // public double[][] areaSample = new double[3][3];

    // public int[][] tXSample = new int[3][3];

    public double[] medianAreas = { 0, 0, 0 };

    public double[] areas0_1_2 = { 0, 0, 0 };

    public double[] medianTx = new double[3];

    public MedianFilter ltx = new MedianFilter(5);

    public MedianFilter ctx = new MedianFilter(5);

    public MedianFilter rtx = new MedianFilter(5);

    public MedianFilter larea = new MedianFilter(5);

    public MedianFilter carea = new MedianFilter(5);

    public MedianFilter rarea = new MedianFilter(5);

    public boolean areaGood;

    public boolean txGood;

    public boolean runAll;
    public boolean runTX = false;
    public boolean runAreas = false;
    public boolean runLRAreas = false;

    public boolean runTXDone = false;
    public boolean runAreasDone = false;
    public boolean runLRAreasDone = false;
    public double startTime;
    public int targetValue = 0;
    public double targetAngle;
    public int weightedTargetValue;

    public double[] contourTx = { 0, 0, 0 };
    public double[] contourTy = { 0, 0, 0 };

    public double[] skew = { 0, 0, 0 };

    public double weightedTargetAngle;
    public boolean isFound;
    public double targetAngle2;

    public double[] shortSide = { 0, 0, 0 };

    public double[] longSide = { 0, 0, 0 };

    public RawContoursV2(LimeLight ll) {

        m_ll = ll;

        horCoord = "tx";
        vertCoord = "ty";
        IMG_WIDTH = 320;
        IMG_HEIGHT = 240;
        HFOV = 59.6;
        VFOV = 45.7;

        if (cameraAt90) {
            horCoord = "ty";
            vertCoord = "tx";
            IMG_WIDTH = 240;
            IMG_HEIGHT = 320;
            HFOV = 45.7;
            VFOV = 59.6;
        }
        HFOV = 30;
        vpw = 2 * Math.tan(Units.degreesToRadians(HFOV / 2));

        vph = 2 * Math.tan(Units.degreesToRadians(VFOV / 2));

    }

    public void display() {

        if (showData)

            displayData();

        if (showDebug)

            debugDisplay();
    }

    /**
     * call getAreaData to get 3 largest areaa always 0,1,2
     * 
     * 
     * 
     * 
     */
    public void getAreaData() {

        shortSide = getShort();
        longSide = getLong();
        skew = getSkew();

        areas0_1_2[0] = m_ll.get("ta0") * 1000;

        areas0_1_2[1] = m_ll.get("ta1") * 1000;

        areas0_1_2[2] = m_ll.get("ta2") * 1000;

        areas0_1_2[0] = Math.round(areas0_1_2[0]);
        areas0_1_2[1] = Math.round(areas0_1_2[1]);
        areas0_1_2[2] = Math.round(areas0_1_2[2]);

        SmartDashboard.putNumber("CON0O", areas0_1_2[0]);
        SmartDashboard.putNumber("CON1O", areas0_1_2[1]);
        SmartDashboard.putNumber("CON2O", areas0_1_2[2]);

    }

    // sort areas by size in

    public void getMedianAreas() {

        medianAreas[0] = larea.calculate(areas0_1_2[0]);
        medianAreas[1] = carea.calculate(areas0_1_2[1]);
        medianAreas[2] = rarea.calculate(areas0_1_2[2]);

    }

    public void getTxValues() {

        contourTx[0] = (((1 + (m_ll.get(horCoord + String.valueOf(0)))) / 2) * IMG_HEIGHT);
        contourTx[1] = (((1 + (m_ll.get(horCoord + String.valueOf(1)))) / 2) * IMG_HEIGHT);
        contourTx[2] = (((1 + (m_ll.get(horCoord + String.valueOf(2)))) / 2) * IMG_HEIGHT);

    }

    public void getMedianTX() {

        medianTx[0] = (int) ltx.calculate(contourTx[0]);
        medianTx[1] = (int) ctx.calculate(contourTx[1]);
        medianTx[2] = (int) rtx.calculate(contourTx[2]);
    }

    public void getTyValues() {

        contourTy[0] = (((1 + (m_ll.get(vertCoord + String.valueOf(0)))) / 2) * IMG_HEIGHT);
        contourTy[1] = (((1 + (m_ll.get(vertCoord + String.valueOf(1)))) / 2) * IMG_HEIGHT);
        contourTy[2] = (((1 + (m_ll.get(vertCoord + String.valueOf(2)))) / 2) * IMG_HEIGHT);

    }

    public int[] getHubVisionData() {

        lTRIndex[0] = 0;
        lTRIndex[1] = 1;
        lTRIndex[2] = 2;

        double swD;
        int swI;

        lToRAreas = medianAreas;

        getTyValues();

        ltoRTyValues[0] = (int) contourTy[0];
        ltoRTyValues[1] = (int) contourTy[1];
        ltoRTyValues[2] = (int) contourTy[2];

        getTyValues();

        int range = contourTx.length;
        int j = 0;
        int i = 0;
        for (i = 0; i < (range - 1); i++) {

            for (j = 0; j < range - i - 1; j++) {

                if (medianTx[j] > medianTx[j + 1]) {

                    swD = medianTx[j + 1];

                    medianTx[j + 1] = medianTx[j];

                    medianTx[j] = swD;

                    swI = ltoRTyValues[j + 1];

                    ltoRTyValues[j + 1] = ltoRTyValues[j];

                    ltoRTyValues[j] = swI;

                    swD = lToRAreas[j + 1];

                    lToRAreas[j + 1] = lToRAreas[j];

                    lToRAreas[j] = swD;

                    swD = skew[j + 1];

                    skew[j + 1] = skew[j];

                    skew[j] = swD;

                    swI = lTRIndex[j + 1];

                    lTRIndex[j + 1] = lTRIndex[j];

                    lTRIndex[j] = swI;

                }
            }
        }

        SmartDashboard.putNumberArray("ltri", showAsDoubleArray(lTRIndex));

        return lTRIndex;
    }

    public double[] getShort() {
        double[] temp = { 0, 0, 0 };

        temp[0] = m_ll.get("tshort0");
        temp[1] = m_ll.get("tshort1");
        temp[1] = m_ll.get("tshort2");

        temp[0] = Math.round(temp[0] * 1000) / 1000.;
        temp[1] = Math.round(temp[1] * 1000) / 1000.;
        temp[2] = Math.round(temp[2] * 1000) / 1000.;

        return temp;
    }

    public double[] getLong() {
        double[] temp = { 0, 0, 0 };

        temp[0] = m_ll.get("tlong0");
        temp[1] = m_ll.get("tlong1");
        temp[1] = m_ll.get("tlong2");

        temp[0] = Math.round(temp[0] * 1000) / 1000.;
        temp[1] = Math.round(temp[1] * 1000) / 1000.;
        temp[2] = Math.round(temp[2] * 1000) / 1000.;

        return temp;
    }

    public double[] getSkew() {
        double[] temp = { 0, 0, 0 };

        temp[0] = m_ll.get("ts0");
        temp[1] = m_ll.get("ts1");
        temp[1] = m_ll.get("ts2");

        // temp[0] = Math.round(temp[0] * 100000) / 100000.;
        // temp[1] = Math.round(temp[1] * 100000) / 100000.;
        // temp[2] = Math.round(temp[2] * 100000) / 100000.;

        return temp;
    }

    public double getStartTime() {
        return Timer.getFPGATimestamp();
    }

    public double getEndTime(double startTime) {
        return (Timer.getFPGATimestamp() - startTime);

    }

    public void runAngleValues() {

        getlrtxyData();

        lrTxAngles = getTxVpAngles();

        lrTyAngles = getTyVpAngles();

    }

    public void getlrtxyData() {

        lRTxVpValues = getlrtxvp(lTRIndex);

        lRTyVpValues = getlrtyvp(lTRIndex);

    }

    private double[] getlrtxvp(int[] ltoRIndex) {
        SmartDashboard.putNumberArray("LTXTPI", showAsDoubleArray(lTRIndex));
        double[] temp = { 0, 0, 0 };
        double vpw2 = vpw / 2;

        temp[0] = vpw2 * m_ll.get(horCoord + String.valueOf(ltoRIndex[0]));
        temp[1] = vpw2 * m_ll.get(horCoord + String.valueOf(ltoRIndex[1]));
        temp[2] = vpw2 * m_ll.get(horCoord + String.valueOf(ltoRIndex[2]));

        SmartDashboard.putNumber("X0Rad", temp[0]);
        SmartDashboard.putNumber("X1Rad", temp[1]);
        SmartDashboard.putNumber("X2Rad", temp[2]);

        return temp;

    }

    private double[] getlrtyvp(int[] ltoRIndex) {

        double[] temp = { 0, 0, 0 };
        double vph2 = vph / 2;
        temp[0] = vph2 * m_ll.get(vertCoord + String.valueOf(ltoRIndex[0]));
        temp[1] = vph2 * m_ll.get(vertCoord + String.valueOf(ltoRIndex[1]));
        temp[2] = vph2 * m_ll.get(vertCoord + String.valueOf(ltoRIndex[2]));

        return temp;

    }

    public double[] getTxVpAngles() {

        // SmartDashboard.putNumberArray("LTXAI", showAsDoubleArray(index));

        double[] temp = { 0, 0, 0 };
        double x = 0;
        double ax = 0;// rads

        x = lRTxVpValues[0];

        ax = Math.atan2(1, x);

        temp[0] = Units.radiansToDegrees(ax);

        x = lRTxVpValues[1];

        ax = Math.atan2(1, x);

        temp[1] = Units.radiansToDegrees(ax);

        x = lRTxVpValues[2];

        ax = Math.atan2(1, x);

        temp[2] = Units.radiansToDegrees(ax);

        temp[0] = 90 - temp[0];

        temp[1] = 90 - temp[1];

        temp[2] = 90 - temp[2];

        temp[0] = Math.round(temp[0] * 1000) / 1000.;
        temp[1] = Math.round(temp[1] * 1000) / 1000.;
        temp[2] = Math.round(temp[2] * 1000) / 1000.;

        return temp;
    }

    public double[] getTyVpAngles() {

        double[] temp = { 0, 0, 0 };
        double y = 0;
        double ay = 0;// rads

        y = lRTyVpValues[0];

        ay = Math.atan2(1, y);

        temp[0] = Units.radiansToDegrees(ay);

        y = lRTyVpValues[1];

        ay = Math.atan2(1, y);

        temp[1] = Units.radiansToDegrees(ay);

        y = lRTyVpValues[2];

        ay = Math.atan2(1, y);

        temp[2] = Units.radiansToDegrees(ay);

        temp[0] = 90 - temp[0];

        temp[1] = 90 - temp[1];

        temp[2] = 90 - temp[2];

        temp[0] = Math.round(temp[0] * 1000) / 1000.;
        temp[1] = Math.round(temp[1] * 1000) / 1000.;
        temp[2] = Math.round(temp[2] * 1000) / 1000.;

        return temp;
    }

    public void runTarget() {
        targetValue = calculateTargetX();
        weightedTargetValue = weightedAverageX();

        targetAngle = getTargetAngle(targetValue);
        weightedTargetAngle = getTargetAngle(weightedTargetValue);
    }

    public int calculateTargetX() {

        double leftArea = medianAreas[0];

        double centerArea = medianAreas[1];

        double rightArea = medianAreas[2];

        double secondArea = Math.max(leftArea, rightArea);

        double ratio = (centerArea - secondArea) / centerArea;

        SmartDashboard.putNumber("target r", ratio);

        int secondX = (int) medianTx[0];

        if (secondArea == rightArea) {

            secondX = (int) medianTx[2];
        }

        int halfOffset = ((int) medianTx[1] - secondX) / 2;

        double offsetX = halfOffset * ratio * 10;// Pref.getPref("HubTgtGn");

        int midpoint = (secondX + (int) medianTx[1]) / 2;

        int targetX = midpoint + (int) offsetX;

        return targetX;
    }

    public int weightedAverageX() {

        double leftArea = medianAreas[0];

        double centerArea = medianAreas[1];

        double rightArea = medianAreas[2];

        double totaArea = leftArea + centerArea + rightArea;

        int leftX = (int) medianTx[0];
        int centerX = (int) medianTx[1];
        int rightX = (int) medianTx[2];

        double leftWeight = leftX * leftArea;
        double centerWeight = centerX * centerArea;
        double rightWeight = rightX * rightArea;

        return (int) ((leftWeight + centerWeight + rightWeight) / totaArea);
    }

    public double getTargetAngle(int xValue) {

        double xAsNx = (xValue - ((IMG_WIDTH / 2) - .5)) / IMG_WIDTH;

        double temp = 0;

        double x = 0;
        double nx = 0;
        double ax = 0;// rads

        nx = xAsNx;

        x = nx * (vpw / 2);

        ax = Math.atan2(1, x);

        temp = Units.radiansToDegrees(ax);

        temp -= 90;

        return -temp;
    }

    public double calcTargetFromAngles() {

        double targetAngle2 = 0;

        double centerAngle = getCenterTxAngle();

        double leftAngle = getLeftTxAngle();

        double rightAngle = getRightTxAngle();

        double rightToCenter = 0;

        double leftToCenter = 0;

        rightToCenter = getRightTxAngle() - getCenterTxAngle();

        leftToCenter = getCenterTxAngle() - getLeftTxAngle();

        SmartDashboard.putNumber("LtoC", leftToCenter);

        SmartDashboard.putNumber("RtoC", rightToCenter);

        double sinCenter = Math.sin(Units.degreesToRadians(centerAngle))

                * Math.cos(Units.degreesToRadians(getCenterTyAngle()));

        double sinRight = Math.sin(Units.degreesToRadians(rightAngle))

                * Math.cos(Units.degreesToRadians(getRightTyAngle()));

        double sinLeft = Math.sin(Units.degreesToRadians(leftAngle))

                * Math.cos(Units.degreesToRadians(getLeftTyAngle()));

        double lcSinAve = (sinCenter + sinLeft) / 2;

        double crSinAve = (sinCenter + sinRight) / 2;

        double gapToUse = crSinAve;

        if (Math.abs(rightToCenter) < Math.abs(leftToCenter))

            gapToUse = lcSinAve;

        targetAngle2 = Units.radiansToDegrees(Math.asin(gapToUse));

        return targetAngle2;
    }

    private double[] showAsDoubleArray(int[] values) {

        int[] temp = values;

        double[] tempA = new double[6];

        for (int i = 0; i < temp.length; i++) {
            tempA[i] = (double) temp[i];

        }
        double[] tempB = Arrays.copyOfRange(tempA, 0, temp.length);

        return tempB;

    }

    public double getLeftArea() {
        return lToRAreas[0];
    }

    public double getCenterArea() {
        return lToRAreas[1];
    }

    public double getRightArea() {
        return lToRAreas[2];
    }

    public double getLRAreaRatio() {
        return lToRAreas[3];
    }

    public int getLeftTx() {
        return (int) medianTx[0];
    }

    public int getCenterTx() {
        return (int) medianTx[1];
    }

    public int getRightTx() {
        return (int) medianTx[2];
    }

    public int getLeftTy() {
        return (int) ltoRTyValues[0];
    }

    public int getCenterTy() {
        return (int) ltoRTyValues[1];
    }

    public int getRightTy() {
        return (int) ltoRTyValues[2];
    }

    // public void setLockContours(boolean on) {
    // lock3Contours = on;
    // }

    public String getLCRTx() {

        return String.valueOf(getLeftTx() + " ," + String.valueOf(getCenterTx()) + " ," + String.valueOf(getRightTx()));
    }

    public String getMedLCRTx() {

        return String.valueOf(medianTx[0] + " ," + String.valueOf(medianTx[1]) + " ," + String.valueOf(medianTx[2]));
    }

    public String getLCRTy() {
        return String.valueOf(getLeftTy() + " ," + String.valueOf(getCenterTy()) + " ," + String.valueOf(getRightTy()));

    }

    public String getLCRArea() {

        return String.valueOf(
                getLeftArea() + " ," + String.valueOf(getCenterArea()) + " ," + String.valueOf(getRightArea()));
    }

    public String getLCRMedianArea() {

        return String.valueOf(
                medianAreas[0] + " ," + String.valueOf(medianAreas[1]) + " ," + String.valueOf(medianAreas[2]));
    }

    public String getLCRShortSide() {

        return String.valueOf(
                shortSide[0] + " ," + String.valueOf(shortSide[1]) + " ," + String.valueOf(shortSide[2]));
    }

    public String getLCRLongSide() {
        return String.valueOf(
                longSide[0] + " ," + String.valueOf(longSide[1]) + " ," + String.valueOf(longSide[2]));
    }

    public String getLCRSkew() {
        return String.valueOf(
                skew[0] + " ," + String.valueOf(skew[1]) + " ," + String.valueOf(skew[2]));
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

    public String getLCRTxMedAngle() {

        return String.valueOf(getLeftTxAngle() + " ," + String.valueOf(getCenterTxAngle()) + " ,"
                + String.valueOf(getRightTxAngle()));
    }

    public String getLCRTyAngle() {
        return String.valueOf(getLeftTyAngle() + " ," + String.valueOf(getCenterTyAngle()) + " ,"
                + String.valueOf(getRightTyAngle()));

    }

    public String getXYPoints(double[] points) {
        return String.valueOf(points[0]) + "," + String.valueOf(points[1]);
    }

    boolean getLRAreasEqual(double tol) {
        return Math.abs(getLeftArea() - getRightArea()) < tol;
    }

    public boolean getLRTyValuesEqual(double tol) {
        return Math.abs(getLeftTy() - getRightTy()) < tol;
    }

    public boolean getTiltOnTarget(int tol) {
        int tiltAngle = 0;
        return Math.abs(tiltAngle - getCenterTy()) < tol;
    }

    public boolean getTurrettOnTarget(int tol) {
        int turretAngle = 0;
        return Math.abs(turretAngle - getCenterTx()) < tol;
    }

    public boolean OKToShoot() {
        int tol = 1;
        return getTurrettOnTarget(tol) && getTiltOnTarget(tol) && getLRAreasEqual(tol);
    }

    private void displayData() {

        SmartDashboard.putNumber("Left Area", getLeftArea());
        SmartDashboard.putNumber("Center Area", getCenterArea());
        SmartDashboard.putNumber("Right Area", getRightArea());

        SmartDashboard.putNumber("Left Tx", getLeftTx());
        SmartDashboard.putNumber("Center Tx", getCenterTx());
        SmartDashboard.putNumber("Right Tx", getRightTx());

        SmartDashboard.putNumber("Left Ty", getLeftTy());
        SmartDashboard.putNumber("Center Ty", getCenterTy());
        SmartDashboard.putNumber("Right Ty", getRightTy());

        SmartDashboard.putBoolean("LRAreasSame", getLRAreasEqual(1));
        SmartDashboard.putBoolean("LRTYsSame", getLRTyValuesEqual(5));

        SmartDashboard.putBoolean("TiltOnTarget", getTiltOnTarget(1));
        SmartDashboard.putBoolean("TurretOntargt", getTurrettOnTarget(1));
        SmartDashboard.putBoolean("OkToShoot", OKToShoot());

    }

    private void debugDisplay() {

        double[] tempIndex = showAsDoubleArray(lTRIndex);
        SmartDashboard.putNumberArray("LRAreasIndex", tempIndex);

    }

    public boolean isDone() {
        return false;
    }

}