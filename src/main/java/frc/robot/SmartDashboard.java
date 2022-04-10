package frc.robot;

public class SmartDashboard {

	public SmartDashboard() {
	}

	public void putNumber(String label, double value) {
		edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
		.putNumber(label, value);
	}

	public void putNumberArray(String label, double[] values) {
		edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
		.putNumberArray(label, values);
	}

	public void putBoolean(String label, boolean value) {
		edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
		.putBoolean(label, value);
	}



}
