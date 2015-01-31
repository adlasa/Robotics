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
	private int curveType;
	AnalogInput analogInput;

	public Infrared(int i, int curveType) {
		analogInput = new AnalogInput(i);
		this.curveType = curveType;
	}

	public void update() {
		distance = curve(analogInput.getVoltage(), curveType);
	}

	public double getDistance() {
		update();
		return distance;
	}

	// 0: Raw voltage, 1: 20-150cm curve, 2: 10-80cm curve
	private double curve(double voltage, int curveType) {
		switch (curveType) {
		case 0: {
			return voltage;
		}
		case 1: {
			return 1 / (voltage - 0);// change 1 to whatever
		}
		case 2: {
			return 1 / (voltage - 0);// change 1 & 0 to whatever
		}
		}

		return 1 / 0;
	}

}