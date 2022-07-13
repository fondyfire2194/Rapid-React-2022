/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Collection;
/**
 * Add your docs here.
 */
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Vector;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;

public class Pref {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Collection<String> v;
  private static Enumeration<String> e;
  private static String tempString;
  private static double tempDouble;

  public static HashMap<String, Double> prefDict = new HashMap<>();

  static {

    // Tilt velocity

    prefDict.put("tiVKp", .004);
    prefDict.put("tiVKi", .0005);
    prefDict.put("tiVKd", 0.6);
    prefDict.put("tiVKiz", 0.75);

    prefDict.put("tiVTune", 0.);

    // tilt position

    prefDict.put("tiPkp", .001);
    prefDict.put("tiPki", .00);
    prefDict.put("tiPkd", 0.);
    prefDict.put("tiPkiz", 0.9);
    prefDict.put("tiPTune", 0.);

    // Turret velocity

    prefDict.put("tuVKp", .002);
    prefDict.put("tuVKi", .0000);
    prefDict.put("tuVKd", 0.);
    prefDict.put("tuVKiz", 1.5);
    prefDict.put("tuVMaxV", 100.);

    prefDict.put("tuVTune", 0.);

    // Turret Position

    prefDict.put("tuPkp", .001);
    prefDict.put("tuPki", .001);
    prefDict.put("tuPkd", 0.);
    prefDict.put("tuPkiz", 0.9);
    prefDict.put("tuPTune", 0.);

    // Turret Lock

    prefDict.put("tuLkp", .001);
    prefDict.put("tuLki", .00);
    prefDict.put("tuLkd", 0.);
    prefDict.put("tuLkiz", 0.9);
    prefDict.put("tuLTune", 0.);

    // shooter velocity

    prefDict.put("sHff", .00018);// =1/maxRPM = 1/(5700
    prefDict.put("sHkp", .0003);
    prefDict.put("sHkI", 0.);
    prefDict.put("sHkd", 0.);
    prefDict.put("sHkiz", 0.);
    prefDict.put("sHTune", 0.);

    prefDict.put("shRPM", 100.);

    // Drive

    prefDict.put("dRKff", .22);
    prefDict.put("dRKp", .1);
    prefDict.put("dRKi", .0);
    prefDict.put("dRKd", .0);
    prefDict.put("dRKiz", .0);
    prefDict.put("dRStKp", .15);
    prefDict.put("dRacc", .1);

    prefDict.put("dRTurnkP", .004);
    prefDict.put("dRTurnkD", .0);
    prefDict.put("dRTurnkI", .0);
    prefDict.put("dRTurnkIz", .0);
    prefDict.put("dRTurnkIzLim", 5.);

    prefDict.put("dRTune", 0.);

    prefDict.put("dRPur", 0.35);

    prefDict.put("kpdv", 0.35);

    prefDict.put("trajkp", .000001);

    // left pickup

    prefDict.put("trajVelLPU", 1.5);

    prefDict.put("trajAccLPU", 1.);

    // left hide

    prefDict.put("trajVelLH", 1.5);

    prefDict.put("trajAccLH", 1.);

    // center pickup 1

    prefDict.put("trajVelCPU1", 1.5);

    prefDict.put("trajAccCPU1", 1.);

    // center hide

    prefDict.put("trajVelCH", 1.5);

    prefDict.put("trajAccCH", 1.);

    // center pickup 3

    prefDict.put("trajVelCPU3", 1.5);

    prefDict.put("trajAccCPU3", 1.);

    // center shoot 3

    prefDict.put("trajVelCSH3", 1.5);

    prefDict.put("trajAccCSH3", 1.);

    // Intake

    prefDict.put("IntakeSpeed", 0.75);

    prefDict.put("LowRollIntakeRPM", 250.);

    prefDict.put("LowRollReleaseRPM", 500.);

    prefDict.put("TopRollShootRPM", 500.);

    prefDict.put("Rollers_kP", .0008);

    prefDict.put("Rollers_kD", .015);

    prefDict.put("CargoDetectValue", 1000.);

    prefDict.put("LowRollStopTimeRed", .2);

    prefDict.put("LowRollStopTimeBlue", .2);

    prefDict.put("ClimbArmDown", -.75);

    prefDict.put("ClimbArmUp", .75);

    prefDict.put("ArcadeTurnKp", .4);

    // camera

    prefDict.put("HubTgtGn", 10.);

    // left start

    prefDict.put("autLRtctPt", Units.inchesToMeters(-51.));

    prefDict.put("autLShootPt", Units.inchesToMeters(0.));

    prefDict.put("autLTilt", 5.);

    prefDict.put("autLTu", 0.);

    prefDict.put("autLRPM", 3500.);

    prefDict.put("autLTShootPt", Units.inchesToMeters(-20.));

    // right start

    prefDict.put("autRRPM", 3500.);

    prefDict.put("autRTilt", 5.);

    prefDict.put("autRTu", 20.);

    prefDict.put("autRRtctPt", Units.inchesToMeters(-51.));

    prefDict.put("autRShootPt", Units.inchesToMeters(0.));

    // center start

    prefDict.put("autCRPM", 3500.);

    prefDict.put("autCTilt", 5.);

    prefDict.put("autCTu", -17.);

    prefDict.put("autCRtctPt", Units.inchesToMeters(-51.));

    prefDict.put("autCShootPt", Units.inchesToMeters(-6.));

    // teteop at hub

    prefDict.put("teleHubTilt", 2.8);

    prefDict.put("teleHubRPM", 3300.);

    // teleop at tarmac

    prefDict.put("teleTarTilt", 13.); // 6 before

    prefDict.put("teleTarRPM", 3400.); // 4000 before

  }

  public static void ensureRioPrefs() {
    // init();
    deleteUnused();
    addMissing();
  }

  public static void deleteUnused() {
    v = new Vector<String>();
    v = Preferences.getKeys();
    // v = (Vector<String>) RobotContainer.prefs.getKeys();
    String[] myArray = v.toArray(new String[v.size()]);

    for (int i = 0; i < v.size(); i++) {
      boolean doNotDelete = myArray[i].equals(".type");

      if (!doNotDelete && !prefDict.containsKey(myArray[i]) && Preferences.containsKey(myArray[i])) {
        Preferences.remove(myArray[i]);
      }
    }

  }

  public static void addMissing() {

    Iterator<Map.Entry<String, Double>> it = prefDict.entrySet().iterator();
    while (it.hasNext()) {
      Map.Entry<String, Double> pair = it.next();
      tempString = pair.getKey();
      tempDouble = pair.getValue();
      if (!Preferences.containsKey((tempString)))
        Preferences.setDouble(tempString, tempDouble);
    }
  }

  public static double getPref(String key) {
    if (prefDict.containsKey(key))
      return Preferences.getDouble(key, prefDict.get(key));
    else
      return 0;
  }

  public static void deleteAllPrefs(Preferences Preferences) {
    edu.wpi.first.wpilibj.Preferences.removeAll();
  }

}
