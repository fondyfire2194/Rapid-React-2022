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

    // Tilt smart motion

    prefDict.put("tIKp", .001);
    prefDict.put("tIKi", .001);
    prefDict.put("tIKd", 0.);
    prefDict.put("tIKiz", 2.);
    prefDict.put("tIMaxV", 550.);// 1000 deg per min
    prefDict.put("tIMaxA", 150.);// deg per min/sec
    prefDict.put("tITune", 0.);

    // tilt vel
    prefDict.put("tIKpv", .006);
    prefDict.put("tIKiv", .001);
    prefDict.put("tIKdv", 0.);
    prefDict.put("tIKizv", 1.);
    prefDict.put("tIMaxVv", 50.);// these are in revs(deg)per second
    prefDict.put("tIMaxAv", 150.);// deg per min/sec
    prefDict.put("tITunev", 0.);

    // Tilt Lock

    prefDict.put("TiLkP", 8.);
    prefDict.put("TiLkI", .001);
    prefDict.put("TiLkD", 0.5);
    prefDict.put("TiLkIZ", 0.8);
    prefDict.put("tiLTune", 0.);
    prefDict.put("TiltLockAdd", 0.);

    // Turret smart motion
    prefDict.put("tURKp", .00015);
    prefDict.put("tURKi", .0001);
    prefDict.put("tURKd", .0002);
    prefDict.put("tURKiz", 1.);
    prefDict.put("tURMaxV", 250.);// deg/sec motor
    prefDict.put("tURMaxA", 500.);// deg/sec motor
    prefDict.put("tURTune", 0.);

    // turret vel
    // Turret smart motion
    prefDict.put("tURKpv", .001);
    prefDict.put("tURKiv", .000);
    prefDict.put("tURKdv", .000);
    prefDict.put("tURKizv", 1.);
    prefDict.put("tURMaxVv", 250.);// deg/sec motor
    prefDict.put("tURMaxAv", 500.);// deg/sec motor
    prefDict.put("tURTunev", 0.);

    // Turret Lock

    prefDict.put("TuLkP", 16.0);
    prefDict.put("TuLkI", .001);
    prefDict.put("TuLkD", 0.);
    prefDict.put("TuLkIZ", 0.9);
    prefDict.put("tuLTune", 0.);

    // shooter velocity

    prefDict.put("sHff", .018);// =1/maxMPS = 1/(5700/60 * .638) .016
    prefDict.put("sHkp", .02);
    prefDict.put("sHkI", 1.);
    prefDict.put("sHkd", 100.);
    prefDict.put("sHkiz", 500.);
    prefDict.put("sHTune", 0.);

    // Drive

    prefDict.put("dRKff", .22);
    prefDict.put("dRKp", .1);
    prefDict.put("dRKi", .0);
    prefDict.put("dRKd", .0);
    prefDict.put("dRKiz", .0);
    prefDict.put("dRStKp", .15);
    prefDict.put("dRacc", .1);

    prefDict.put("dRTune", 0.);

    // camera

    prefDict.put("LimelightHeight", .66);
    prefDict.put("LimelightMaxHeight", .686);

    prefDict.put("HubTgtGn", 10.);

    // Set to 1 before power up to log
    prefDict.put("LogTilt", 0.);
    prefDict.put("LogTurret", 0.);
    prefDict.put("LogShoot", 0.);
    prefDict.put("LogHub", 0.);

    // Shuffleboard

    prefDict.put("IsMatch", 1.);
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
