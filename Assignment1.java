import lejos.hardware.BrickFinder;
import lejos.hardware.Keys;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.Color;
import lejos.robotics.ColorDetector;
import lejos.robotics.ColorIdentifier;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.filter.MeanFilter;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;
import java.lang.Math;
//import javafx.scene.paint.Color;
import java.util.Arrays;

public class Assignment1 {
	static EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.A);
	static EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.B);
	private static EV3ColorSensor Leftsensor = new EV3ColorSensor(SensorPort.S1);
	private static EV3ColorSensor Rightsensor = new EV3ColorSensor(SensorPort.S3);
	//private static SampleProvider LeftbrightnessSensorMode = Leftsensor.getRGBMode();
	//private static SampleProvider RightbrightnessSensorMode = Rightsensor.getRGBMode();
	public static void main(String[] args) {
		//initialise brick and sensors needed
		EV3 ev3brick = (EV3) BrickFinder.getLocal();
		Keys buttons = ev3brick.getKeys();
		TextLCD lcddisplay = ev3brick.getTextLCD();
		//initialise ultrasonic sensor
		/*
		Port portS2 = ev3brick.getPort("S2");
		EV3UltrasonicSensor sonicSensor = new EV3UltrasonicSensor(portS2);
		SampleProvider sonicdistance = sonicSensor.getDistanceMode();
		*/
		lejos.hardware.port.Port portS2 = ev3brick.getPort("S2");
		EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(portS2);
		SampleProvider ultrasonicdistance = ultrasonicSensor.getDistanceMode();
		SampleProvider averageUltra = new MeanFilter(ultrasonicdistance, 5);
		float[] ultrasample = new float[averageUltra.sampleSize()];
		
		//Distance IR
		lejos.hardware.port.Port portS4 = ev3brick.getPort("S4");
		EV3IRSensor IRSensor = new EV3IRSensor(portS4);
		SampleProvider IRdistance = IRSensor.getDistanceMode();
		SampleProvider averageIR = new MeanFilter(IRdistance, 5);
		float[] IRsample = new float[averageIR.sampleSize()];

		//set modes of the color sensors
		Leftsensor.setCurrentMode("ColorID");
		Leftsensor.getColorIDMode();
		Rightsensor.setCurrentMode("ColorID");
		Rightsensor.getColorIDMode();
		//creating the movepilot
		Wheel wheel1 = WheeledChassis.modelWheel(LEFT_MOTOR, 5.425).offset(-6.25);
		Wheel wheel2 = WheeledChassis.modelWheel(RIGHT_MOTOR, 5.425).offset(6.25);
		
		Chassis chassis = new WheeledChassis(new Wheel[] {wheel1,wheel2}, WheeledChassis.TYPE_DIFFERENTIAL);
		MovePilot pilot = new MovePilot(chassis);
		
		
		pilot.setAngularSpeed(20);
		pilot.setLinearSpeed(15);
		
		LCD.drawString("Init",  2,  5);
		LCD.setAutoRefresh(false);
		buttons.waitForAnyPress();
		
		int lastrotatevalue = 0;
		//below makes the robot execute the tasks
		while(buttons.getButtons() != Keys.ID_ESCAPE) {
			//get samples, each sensor takes two samples and compares them to eliminate some error
			//in the color sensors
			float[] sampleLeft = getSample(Leftsensor);
			float[] sampleLeft2 = getSample(Leftsensor);
			float[] sampleRight = getSample(Rightsensor);
			float[] sampleRight2 = getSample(Rightsensor);
			IRdistance.fetchSample(IRsample, 0);
			ultrasonicdistance.fetchSample(ultrasample, 0);
			

			while(!colorName((int)sampleLeft[0]).equals(colorName((int) sampleLeft2[0]))){
				sampleLeft = getSample(Leftsensor);
				sampleLeft2 = getSample(Leftsensor);
			}
			while(!colorName((int)sampleRight[0]).equals(colorName((int) sampleRight2[0]))){
				sampleRight = getSample(Leftsensor);
				sampleRight2 = getSample(Leftsensor);
			}
			//checks if there is any objects close to the robot, goes around if there is

			if (ultrasample[0] < 0.1) {
				pilot.rotate(90);
				IRdistance.fetchSample(IRsample, 0);
				//while loop to go by the object
				while(IRsample[0] < 50) {
					pilot.travel(1);
					IRdistance.fetchSample(IRsample, 0);
				}
				pilot.travel(20);
				pilot.rotate(-90);
				pilot.travel(15);
				IRdistance.fetchSample(IRsample, 0);
				LCD.drawString("IR " + IRsample[0], 2, 2);
				// while loop to go around the object
				while(IRsample[0] < 80) {
					LCD.drawString("IR " + IRsample[0], 1, 1);
					pilot.travel(1);
					IRdistance.fetchSample(IRsample, 0);
				}
				pilot.travel(20);
				pilot.rotate(-90);
				pilot.travel(10);
				float[] sampleRightwhile = getSample(Rightsensor);
				float[] sampleLeftwhile = getSample(Leftsensor);
				while(!BlackOrNot(sampleLeftwhile) && !BlackOrNot(sampleRightwhile)) {
					pilot.travel(1);
					sampleLeftwhile = getSample(Leftsensor);
					sampleRightwhile = getSample(Rightsensor);
				}
				pilot.travel(10);
				pilot.rotate(90);
		}

			//if one of the sensors reads red, then it moves forward and checks again 
			//if only one is red then it carrys on
			if (isRed(sampleRight) || isRed(sampleLeft)) {
				pilot.travel(1);
				sampleLeft = getSample(Leftsensor);
				sampleRight = getSample(Rightsensor);
				if (isRed(sampleLeft)   && isRed(sampleRight)) {
					while(true){
						pilot.stop();
					}
				}
			}
			// if left sensor reads green rotates 90 degrees to the left
			// checks that the right sensor also doesnt read green
			if (isGreen(sampleLeft)) {
				pilot.travel(2);
				sampleRight = getSample(Rightsensor);
				if (isGreen(sampleRight)) {
					pilot.rotate(180);
				} else {
					pilot.travel(8);
					pilot.rotate(-90);
					pilot.travel(5);
				}
				
				
			}
			//checks if the right sensor reaads green , if left sensor also doesnt read green
			// it turns right
			// if both are green it 180s
			if (isGreen(sampleRight)) {
				pilot.travel(2);
				sampleLeft = getSample(Leftsensor);
				if (isGreen(sampleLeft)) {
					pilot.rotate(180);
				} else {
				pilot.travel(8);
				pilot.rotate(90);
				pilot.travel(5);
				}
				
			}
			//if the right sensor reads black the robot rotates to the right
			if (!BlackOrNot(sampleLeft) && BlackOrNot(sampleRight)) {
				// roate a direction right
				int rotatevalue = 15;
				pilot.rotate(rotatevalue);
				pilot.travel(0.3);
				 lastrotatevalue = rotatevalue;
			}
			//if left sensor reads black the robot turns left
			if (BlackOrNot(sampleLeft) && !BlackOrNot(sampleRight)) {
				// roate other direction
				int rotatevalue = -15;
				pilot.rotate(rotatevalue);
				pilot.travel(0.3);
				lastrotatevalue = rotatevalue;
			}
			//if both sensors read not black, the robot moves forwards
			if (BlackOrNot(sampleLeft) && BlackOrNot(sampleRight)) {
				pilot.travel(1);
				float[] sampleLeftBlack = getSample(Leftsensor);
				float[] sampleRightBlack = getSample(Rightsensor);
				if (isRed(sampleRightBlack) || isRed(sampleLeftBlack) || isGreen(sampleLeftBlack) || isGreen(sampleRightBlack)) {
					pilot.arcForward(5);
				 
			} else {
				boolean online = false;
				int direction = 0;
				while(online == false) {
					int rotatevalue = 0;
					
					if (lastrotatevalue < 0) {
						rotatevalue = -15;
						direction = -1;
					} else if (lastrotatevalue >= 0) {
						rotatevalue = 15;
						direction = 1;
					}
					pilot.rotate(rotatevalue);
					float[] testleft = getSample(Leftsensor);
					float[] testright = getSample(Rightsensor);
					if (!BlackOrNot(testleft) && !BlackOrNot(testright)) {
						online = true;
					} else {
						online = false;
					}
				}
				pilot.rotate(5*direction);
			}
			}
			//if both are black the robot moves forwards slightly
			if(!BlackOrNot(sampleLeft) && !BlackOrNot(sampleRight)) {
				pilot.travel(1);
			}
			
		}
		ultrasonicSensor.close();
		IRSensor.close();
	}
	
	public static String colorName(int color)
	{//gives a string value to the int read from the sensor
		switch (color)
		{
			case Color.NONE:
				return "None";
				
			case Color.BLACK:
				return "Black";
				
			case Color.BLUE:
				return "Blue";
				
			case Color.BROWN:
				return "Brown";
				
			case Color.CYAN:
				return "Cyan";
				
			case Color.DARK_GRAY:
				return "Dark Gray";
				
			case Color.GRAY:
				return "Gray";
				
			case Color.GREEN:
				return "Green";
				
			case Color.LIGHT_GRAY:
				return "Light Gray";
				
			case Color.MAGENTA:
				return "Magenta";
				
			case Color.ORANGE:
				return "Orange";
				
			case Color.PINK:
				return "Pink";
				
			case Color.RED:
				return "Red";
				
			case Color.WHITE:
				return "White";
				
			case Color.YELLOW:
				return "Yellow";
		}
		
		return "";
	}
	
	public static void USSensor(SampleProvider sonicdistance, MovePilot pilot, EV3ColorSensor leftsensor) {
		float[] sample = new float[sonicdistance.sampleSize()];
		sonicdistance.fetchSample(sample, 0);
		//makes the robot go around an object
		if(sample[0] <  0.1) {
			
			pilot.rotate(80);
			pilot.travel(40);
			pilot.rotate(-90);
			pilot.travel(60);
			pilot.rotate(-90);
			returnToLine(leftsensor, pilot);	
		}
		
	}	
	public static void returnToLine(EV3ColorSensor leftsensor, MovePilot pilot) {
		//makes the robot go forwards until it reaches the line and readjusts
		float[] sampleLeft = getSample(leftsensor);
		while(colorName((int) sampleLeft[0]) != "Black") {
			pilot.travel(2);
			sampleLeft = getSample(leftsensor);	
		}
		pilot.travel(5);
		pilot.rotate(-90);
		
	}
	
	public static boolean isRed(float[] sample) {
		//returns true if the sensor reads red
		if(colorName((int) sample[0]) == "Red") {
			return true;
		} else {
			return false;
		}
	}
	public static boolean isGreen(float[] sample) {
		//returns true if the sensor reads green
		if(colorName((int) sample[0]) == "Green") {
			return true;
		} else {
			return false;
		}	
	}

	public static boolean BlackOrNot (float[] sample) {
		//returns true if the sensor reads black
		if(colorName((int) sample[0]) == "Black") {
			return true;
		} else {
			return false;
		}
    }
	
	public static float[] getSample(EV3ColorSensor sensor) {
		//gets a sample from the given sensor
		float[] sample = new float[1];
		sensor.fetchSample(sample, 0);
		return sample;
	

	}
}
