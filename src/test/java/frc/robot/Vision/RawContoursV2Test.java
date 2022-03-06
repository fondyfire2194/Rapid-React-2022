package frc.robot.Vision;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;

import frc.robot.SmartDashboard;

public class RawContoursV2Test {

	@Mock
	LimeLight limelight;

	RawContoursV2 rcv2;

	// other possible related test classes
	//GetAreaTXData

	@BeforeAll
	static public void beforeAll() {
		MockDash.IS_PRINT=false;
	}


	@Test
	public void testConstructor() {
		// TEST
		rcv2 = new RawContoursV2(limelight);
		// CONFIRM
		assertTrue(true, "If we get here then we can construct a tests instance.");
	}
	@Test
	public void testCameraPortraitSettings() {
		// SETUP
		RawContoursV2.cameraAt90 = true;
		// TEST
		rcv2 = new RawContoursV2(limelight);

		// CONFIRM
        assertEquals("ty", rcv2.horCoord);
        assertEquals("tx", rcv2.vertCoord);
        assertEquals(RawContoursV2.NO_ZOOM_IMG_HEIGHT, rcv2.active_IMG_WIDTH);
        assertEquals(RawContoursV2.NO_ZOOM_IMG_WIDTH, rcv2.active_IMG_HEIGHT);
        assertEquals(RawContoursV2.NO_ZOOM_CAMERA_VFOV, rcv2.activeHFOV);
        assertEquals(RawContoursV2.NO_ZOOM_CAMERA_HFOV, rcv2.activeVFOV);
	}
	@Test
	public void testCameraLandscapeSettings() {
		// SETUP
		RawContoursV2.cameraAt90 = false;
		// TEST
		rcv2 = new RawContoursV2(limelight);

		// CONFIRM
        assertEquals("tx", rcv2.horCoord);
        assertEquals("ty", rcv2.vertCoord);
        assertEquals(RawContoursV2.NO_ZOOM_IMG_WIDTH, rcv2.active_IMG_WIDTH);
        assertEquals(RawContoursV2.NO_ZOOM_IMG_HEIGHT, rcv2.active_IMG_HEIGHT);
        assertEquals(RawContoursV2.NO_ZOOM_CAMERA_HFOV, rcv2.activeHFOV);
        assertEquals(RawContoursV2.NO_ZOOM_CAMERA_VFOV, rcv2.activeVFOV);

//        System.out.println(rcv2.active_vpw);
//        System.out.println(rcv2.active_vph);
	}
	@Test
	public void testCameraLandscapeViewPort() {
		// SETUP
		double expectedVPH = 0.843;
		double expectedVPW = 1.145;
		double delta = 0.001;
		RawContoursV2.cameraAt90 = false;
		// TEST
		rcv2 = new RawContoursV2(limelight);
		// CONFIRM
		assertEquals(expectedVPH, rcv2.active_vph, delta);
		assertEquals(expectedVPW, rcv2.active_vpw, delta);
	}
	@Test
	public void testCameraPortraitViewPort() {
		// SETUP
		double expectedVPW = 0.843;
		double expectedVPH = 1.145;
		double delta = 0.001;
		RawContoursV2.cameraAt90 = true;
		// TEST
		rcv2 = new RawContoursV2(limelight);
		// CONFIRM
		assertEquals(expectedVPH, rcv2.active_vph, delta);
		assertEquals(expectedVPW, rcv2.active_vpw, delta);
	}

	@Test
	public void testCalculateTargetX() {
		// SETUP
		RawContoursV2.cameraAt90 = true;
		rcv2 = new RawContoursV2(limelight);
		// the SmartDashboard needs to be mocked during tests
		rcv2.dash = new MockDash();

		// set up three close area differences similar to live testing
		double area1 = 211;
		double area2 = 222;
		double area3 = 233;
		rcv2.medianAreas = new double[]{area1, area3, area2};

		// set up some x positions for the three largest tape targets
		double tx1 = 110;
		double tx2 = 150;
		double tx3 = 190;
		rcv2.medianTx = new double[]{tx1, tx2, tx3};

		// TEST
		int targetX = rcv2.calculateTargetX();

		// CONFIRM
		int expected = 161;
		assertEquals(expected, targetX);
	}

	@Test
	public void testWeightedAverageX() {
		// SETUP
		RawContoursV2.cameraAt90 = true;
		rcv2 = new RawContoursV2(limelight);
		// the SmartDashboard needs to be mocked during tests
		rcv2.dash = new MockDash();

		// set up three close area differences similar to live testing
		double area1 = 211;
		double area2 = 222;
		double area3 = 233;
		rcv2.medianAreas = new double[]{area1, area3, area2};

		// set up some x positions for the three largest tape targets
		double tx1 = 110;
		double tx2 = 150;
		double tx3 = 190;
		rcv2.medianTx = new double[]{tx1, tx2, tx3};

		// TEST
		int targetX = rcv2.weightedAverageX();

		// CONFIRM
		int expected = 151; //using (int)((area1*tx1+area2*tx2+area3*tx3)/(area1+area2+area3));
		assertEquals(expected, targetX, 1); // allow for rounding differences with delta 1px
	}
	@Test
	public void testMathAtan() {
		double angleAtan = Math.atan(1/1.81);
		double angleAtan2 = Math.atan2(1,1.81);
		assertEquals(angleAtan2, angleAtan);
	}

	@Test
	public void testGetTargetAnglePortrait() {
		// SETUP
		RawContoursV2.cameraAt90 = true; // horizontal orientation, x is from 1 to 240
		rcv2 = new RawContoursV2(limelight);
		// the SmartDashboard needs to be mocked during tests
		rcv2.dash = new MockDash();

		// TEST
		// in portrait orientation center point is 120 for zoom and 720/2
		int center = RawContoursV2.NO_ZOOM_IMG_HEIGHT/2;
		int pxOffCenter = 60;

		double targetAngleToCenterFromRight = rcv2.getTargetAngle(center+(pxOffCenter-1));
		// When the image is to the right of center then this is a left turn to target

		double targetAngleToCenterFromLeft = rcv2.getTargetAngle(center-pxOffCenter);
		// When the image is to the left of center then this is a right turn to target

		// CONFIRM
		double delta = 0.05;
		double expectedTurnForLeft  = - pxOffCenter * 0.0335 * 2;// this is a guess
		assertEquals(expectedTurnForLeft, targetAngleToCenterFromLeft, delta);
		double expectedTurnForRight = - expectedTurnForLeft;
		assertEquals(expectedTurnForRight, targetAngleToCenterFromRight, delta);
	}

	@Test
	public void testGetTargetAngleLandscape() {
		// SETUP
		RawContoursV2.cameraAt90 = false; // horizontal orientation, x is from 1 to 240
		rcv2 = new RawContoursV2(limelight);
		// the SmartDashboard needs to be mocked during tests
		rcv2.dash = new MockDash();

		// TEST
		// in landscape orientation center point is 120 for zoom and 720/2
		int center = RawContoursV2.NO_ZOOM_IMG_WIDTH/2;
		int pxOffCenter = 1;

		// somewhere we are not taking a
		double targetAngleToCenterFromRight = rcv2.getTargetAngle(center+(pxOffCenter-1));
		// When the image is to the right of center then this is a left turn to target

		double targetAngleToCenterFromLeft = rcv2.getTargetAngle(center-pxOffCenter);
		// When the image is to the left of center then this is a right turn to target

		// CONFIRM
		double delta = 0.05;
		double expectedTurnForLeft  = - pxOffCenter * 0.0342 * 2;// this is a guess
		assertEquals(expectedTurnForLeft, targetAngleToCenterFromLeft, delta);
		double expectedTurnForRight = - expectedTurnForLeft;
		assertEquals(expectedTurnForRight, targetAngleToCenterFromRight, delta);
	}
}



class MockDash extends SmartDashboard {
	static boolean IS_PRINT = true;

	boolean isPrintIt = IS_PRINT;

	public MockDash() {
	}
	public MockDash(boolean printIt) {
		isPrintIt = printIt;
	}

	@Override
	public void putNumber(String label, double value) {
		if (isPrintIt)
			System.out.println(label + "=" + value);
	}
	@Override
	public void putNumberArray(String label, double[] values) {
		String value = "";
		String sep = "";
		for (double dvalue : values) {
			value += sep;
			value += dvalue;
			sep = ", ";
		}
		if (isPrintIt)
			System.out.println(label + "=" + value);
	}
	@Override
	public void putBoolean(String label, boolean value) {
		if (isPrintIt)
			System.out.println(label + "=" + value);
	}
}
