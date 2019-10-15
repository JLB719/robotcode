import lejos.hardware.BrickFinder;
import lejos.hardware.Keys;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;

public class TouchSensor extends EV3TouchSensor{
	static EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.A);
	static EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.B);
	
	public TouchSensor(Port port) {
		super(port);
	}
	
	public boolean isPressed() {
		float[] sample = new float[1];
		fetchSample(sample, 0);
		return sample[0] != 0;
	}
	/*public static void main(String[] args) {
		EV3 ev3brick = (EV3) BrickFinder.getLocal();
		Keys buttons = ev3brick.getKeys();
		Port portS3 = ev3brick.getPort("S3");
		TextLCD lcddisplay = ev3brick.getTextLCD();
		EV3TouchSensor touchSensor = new EV3TouchSensor(portS3);
		SensorMode toucher = touchSensor.getTouchMode();
		float[] samplevalue = new float[toucher.sampleSize()];
		lcddisplay.clear();
		while (buttons.getButtons() != Keys.ID_ESCAPE) {
			toucher.fetchSample(samplevalue, 0);
			lcddisplay.drawString("value: " + samplevalue[0], 0, 0);
			Delay.msDelay(20);
		}
		
	}*/
}
