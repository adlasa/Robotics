package org.usfirst.frc.team1458.robot;

public class Levels {
	
	public double getHeight(MainLevel m, LevelMode l, CarryObject c, LevelMod d) {
		return m.getHeight()+l.getHeight()+c.getHeight()+d.getHeight();
	}
	
	public Levels() {
		
	}
	
	public enum MainLevel {
		ONE(0.0),
		TWO(1.0),
		THREE(2.0),
		FOUR(3.0);
		
		private double height;
		
		private MainLevel(double height) {
			this.height = height;
		}
		public double getHeight() {
			return this.height;
		}
	}
	public enum LevelMode {
		FLOOR(0.0),
		PLATFORM(0.2),
		STEP(0.9);
		
		private double height;
		
		private LevelMode(double height) {
			this.height = height;
		}
		public double getHeight() {
			return this.height;
		}
	}
	public enum CarryObject {
		TOTE(0.0),
		CONTAINER(1.3);
		
		private double height;
		
		private CarryObject(double height) {
			this.height = height;
		}
		public double getHeight() {
			return this.height;
		}
	}
	public enum LevelMod {
		LIP(0.0),
		CARRY(0.4),
		GRAB(0.2),
		DISENGAGE(-0.2);
		
		private double height;
		
		private LevelMod(double height) {
			this.height = height;
		}
		public double getHeight() {
			return this.height;
		}
	}
}