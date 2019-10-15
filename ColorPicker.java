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
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.Color;
import lejos.robotics.ColorDetector;
import lejos.robotics.ColorIdentifier;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;
import java.lang.Math;
public class ColorPicker {
	private static EV3ColorSensor Leftsensor = new EV3ColorSensor(SensorPort.S1);
	private static EV3ColorSensor Rightsensor = new EV3ColorSensor(SensorPort.S3);
	private static SampleProvider LeftbrightnessSensorMode = Leftsensor.getRGBMode();
	private static SampleProvider RightbrightnessSensorMode = Rightsensor.getRGBMode();
	public static void main(String[] args) {
		EV3 ev3brick = (EV3) BrickFinder.getLocal();
		
		Keys buttons = ev3brick.getKeys();
		
		TextLCD lcddisplay = ev3brick.getTextLCD(); 
		
		Leftsensor.setFloodlight(false);
		Rightsensor.setFloodlight(false);
		LCD.drawString("Init",  2,  2);
		LCD.setAutoRefresh(false);
		SampleProvider brightnessSensorMode1 = Leftsensor.getRGBMode();
		SampleProvider brightnessSensorMode2 = Rightsensor.getRGBMode();
		float[] sampleLeft = new float[brightnessSensorMode1.sampleSize()];
		float[] sampleRight = new float[brightnessSensorMode2.sampleSize()];
		Color c = new Color((int)(sampleLeft[0] *255), (int)(sampleLeft[1] *255), (int)(sampleLeft[2] * 255));
		buttons.waitForAnyPress();
		while(buttons.getButtons() != Keys.ID_ESCAPE) {
			brightnessSensorMode1.fetchSample(sampleLeft, 0);
			brightnessSensorMode2.fetchSample(sampleRight, 0);
			c = new Color((int)(sampleLeft[0] *255), (int)(sampleLeft[1] *255), (int)(sampleLeft[2] * 255));
			LCD.drawString("COLORNAME : " + colorName(c.getColor()), 0, 0);
			LCD.drawString("Left R: " + sampleLeft[0], 1, 1);
			LCD.drawString("Left G: " + sampleLeft[1], 1, 2);
			LCD.drawString("Left B: " + sampleLeft[2], 1, 3);
			LCD.drawString("Right R: " + sampleRight[0], 1, 4);
			LCD.drawString("Right G: " + sampleRight[1], 1, 5);
			LCD.drawString("Right B: " + sampleRight[2], 1, 6);
			//LCD.drawString("color: " + sampleLeft.getColor(), 1, 7);
			Delay.msDelay(100);
			Delay.msDelay(100);
			Delay.msDelay(100);
			Delay.msDelay(100);
			Delay.msDelay(100);
			Delay.msDelay(100);
			Delay.msDelay(100);
			
		}
	}
	
	public static String colorName(int color)
	{
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
}
