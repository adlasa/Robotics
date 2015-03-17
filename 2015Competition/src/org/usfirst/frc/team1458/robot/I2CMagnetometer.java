package org.usfirst.frc.team1458.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.communication.UsageReporting;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.util.BoundaryException;
import edu.wpi.first.wpilibj.AccumulatorResult;

public class I2CMagnetometer extends edu.wpi.first.wpilibj.SensorBase {

	private double angle;
	I2C m_i2c;
	private byte[] rawInput = new byte[6];
	private final int address = 0x0e;//specific to magnetometer
	private double[] axes = new double[3]; // x y z
	byte[] buffer = new byte[1];

	/**
	 * Initialise the magnetometer, should only be run by the constructor.
	 */
	private void initMagnetometer() {
		//checks to see if it is null
		if (m_i2c == null) {
			System.out.println("Null m_i2c");
		}
		
		// interrogate the buffer to make sure that we are not trying to read the values before they are
		// ready to be read, so we're getting a gibberish value
		m_i2c.read(0x07, 1, buffer);
		if ((int) buffer[0] != 0xc4) {
			System.out.println("Something has gone terribly wrong.");
			System.out.println(buffer[0]);
		} else {
			System.out.println("Found who am i");
		}

		//settings for rate and measuring data
		m_i2c.write(0x10, 0b00011001);
		m_i2c.write(0x11, 0b10000000);

		//calibration (done in code, so set to 0)
		m_i2c.write(0x09, 0b00000000);
		m_i2c.write(0x0a, 0b00000000);
		m_i2c.write(0x0b, 0b00000000);
		m_i2c.write(0x0c, 0b00000000);
		m_i2c.write(0x0d, 0b00000000);
		m_i2c.write(0x0e, 0b00000000);

		//starting values for max and min that should allow for comparison while being easily exceeded
		SmartDashboard.putNumber("X Max", 0);
		SmartDashboard.putNumber("Y Max", 0);
		SmartDashboard.putNumber("Z Max", 0);
		SmartDashboard.putNumber("X Min", 10000);
		SmartDashboard.putNumber("Y Min", 10000);
		SmartDashboard.putNumber("Z Min", 10000);

		//initial update
		update();
		zero();

	}

	/**
	 * Magnetometer constructor, address is precoded
	 */
	public I2CMagnetometer() {
		// m_analog = new AnalogChannel(channel);
		m_i2c = new I2C(I2C.Port.kOnboard, address);
		// m_channelAllocated = true;
		initMagnetometer();
	}

	/**
	 * Return the actual angle in degrees that the robot is currently facing.
	 *
	 * The angle is based on the values, it is NOT continuous.
	 *
	 * @return the current heading of the robot in degrees. This heading is
	 *         based on the magnetometer.
	 */
	public double getAngle() {
		// update();
		// do math stuff to make more accurate;
		return angle;
	}

	public void zero() {
		SmartDashboard.putNumber("X Max", 0);
		SmartDashboard.putNumber("Y Max", 0);
		SmartDashboard.putNumber("Z Max", 0);
		SmartDashboard.putNumber("X Min", 10000);
		SmartDashboard.putNumber("Y Min", 10000);
		SmartDashboard.putNumber("Z Min", 10000);
	}
	/**
	 * Returns whether the magnetometer is ready to output new data. Currently set to automatically return true.
	 * @return Always true;
	 */
	
	public boolean isReady() {
		return true;
		//m_i2c.read(0x0, 1, buffer);
		//return (((buffer[0]>>3)&1)==1);
	}

	public void update() {
		//get values from magnetometer
		m_i2c.read(0x01, 6, rawInput);
		// convert it into number format
		//for loop for each axis
		for(int i = 0; i<3; i++) {
			//sets int to first byte
			int f = (int) rawInput[0+2*i];
			//shifts so room for second byte
			f *= 256;
			//because of how 1st digit in 2nd byte is interpreted as a sign rather than 128, have to have if statement
			if((int)rawInput[1+2*i]<0) {
				f+=(256+(int)rawInput[1+2*i]);
			} else {
				f+=(int)rawInput[1+2*i];
			}
			//set axis to calculated value
			axes[i] = f;
			//axes[f] = 0.8*rateAdjust(i)+0.2*axes[f];
		}

		axes[0]/=-1;//flip x to be right
		
		
//calibration
		axes[0]-=270; //maggie 1247, margaret 270
		axes[1]-=1025; //maggie 1186, margaret 1025
		//axes[2]-=0;
		axes[0]/=298; //maggie 239, margaret 298
		axes[1]/=340; //maggie 241, margaret 340
		//axes[2]/=1;
		
//output the data
		SmartDashboard.putNumber("X field", axes[0]);
		SmartDashboard.putNumber("Y field", axes[1]);
		SmartDashboard.putNumber("Z field", axes[2]);

		if(SmartDashboard.getNumber("X Max",0)<axes[0]) {
			SmartDashboard.putNumber("X Max", axes[0]);
		}
		if(SmartDashboard.getNumber("X Min",10000)>axes[0]) {
			SmartDashboard.putNumber("X Min", axes[0]);
		}
		if(SmartDashboard.getNumber("Y Max",0)<axes[1]) {
			SmartDashboard.putNumber("Y Max", axes[1]);
		}
		if(SmartDashboard.getNumber("Y Min",10000)>axes[1]) {
			SmartDashboard.putNumber("Y Min", axes[1]);
		}
		if(SmartDashboard.getNumber("Z Max",0)<axes[2]) {
			SmartDashboard.putNumber("Z Max", axes[2]);
		}
		if(SmartDashboard.getNumber("Z Min",10000)>axes[2]) {
			SmartDashboard.putNumber("Z Min", axes[2]);
		}
		
		SmartDashboard.putNumber("X Raw field", axes[0]);
		SmartDashboard.putNumber("Y Raw field", axes[1]);
		SmartDashboard.putNumber("Z Raw field", axes[2]);
		//assuming that positive x is to the right of the robot, positive y is straight forward
		angle = Math.atan(axes[1]/axes[0]);
		if(axes[0]<0) {
			angle+=Math.PI;
		}
		angle*=(180/Math.PI);

		angle+=90;//can shift as desired so 0-360 crossover (which screws up magnetometer) is to the rear left of robot
		angle = Math.round(angle * 10) / 10.0;
	}
}