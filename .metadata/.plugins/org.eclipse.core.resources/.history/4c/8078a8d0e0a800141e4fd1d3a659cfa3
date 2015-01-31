package org.usfirst.frc.team1458.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.communication.UsageReporting;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.util.BoundaryException;
import edu.wpi.first.wpilibj.AccumulatorResult;

public class Infrared extends edu.wpi.first.wpilibj.SensorBase {
	private double rate;
	private double distance;
	AnalogInput analogInput;

	public Infrared(int i) {
		analogInput = new AnalogInput(i);
	}

	
	public void update() {
		distance = analogInput.getVoltage()*1;
	}
	
	public double getDistance() {
		update();
		return distance;
	}

	
}