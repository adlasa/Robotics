package org.usfirst.frc.team1458.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Gyro;

public class RobotFunctions {
	
	boolean hMode = false;
	Victor rightDriveFront = new Victor(1);
	Victor rightDriveRear = new Victor(0);
	Victor leftDriveFront = new Victor(3);
	Victor leftDriveRear = new Victor(2);
	Talon hDrive = new Talon(4);
	Talon armLeft = new Talon(7);
	Talon armRight = new Talon(6);
	Talon elevator = new Talon(5);
	
	RobotFunctions() {
		
	}
	
	public void tankdrive(double rightPower, double leftPower)
	{
		rdrive(rightPower);
		ldrive(leftPower);
	
	}
	
	public void rdrive(double power) {
		rightDriveFront.set(-power);
		rightDriveRear.set(-power);
		hDrive.set(0);
	}
	public void ldrive(double power) {
		leftDriveFront.set(power);
		leftDriveRear.set(power);
		hDrive.set(0);
	}
	public void hdrive(double power) {
		hDrive.set(power);
		rightDriveFront.set(0);
		rightDriveRear.set(0);
		leftDriveFront.set(0);
		leftDriveRear.set(0);
	}
	public void aLeftdrive(double power) {
		armLeft.set(power);
	}
	public void aRightdrive(double power) {
		armRight.set(power);
	}
	
	public void edrive(double power) {
		elevator.set(power);
	}
	
}