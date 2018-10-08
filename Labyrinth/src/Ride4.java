import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Ride4 {

	RegulatedMotor motorL;
	RegulatedMotor motorR;
	RegulatedMotor motorSmall;

	EV3UltrasonicSensor distanceSensor;
	EV3TouchSensor touch;
	EV3GyroSensor gyro;
	SensorMode touchM;
	SampleProvider distanceSampler;
	SampleProvider listenSampler;
	SampleProvider gyroSampler;

	
	double distance;

	float[] sampleDistance;
	float[] sampleTouch;
	float[] sampleGyro;
	
	Desk desk; 

	public Ride4() {
		motorL = new EV3LargeRegulatedMotor(MotorPort.C);
		motorR = new EV3LargeRegulatedMotor(MotorPort.B);
		motorSmall = new EV3LargeRegulatedMotor(MotorPort.A);
		motorL.synchronizeWith(new RegulatedMotor[] { motorR });
		touch = new EV3TouchSensor(SensorPort.S2);
		distanceSensor = new EV3UltrasonicSensor(SensorPort.S3);
		gyro = new EV3GyroSensor(SensorPort.S4);
		distanceSampler = distanceSensor.getDistanceMode();
		sampleDistance = new float[distanceSampler.sampleSize()];
		touchM = touch.getTouchMode();
		sampleTouch = new float[touchM.sampleSize()];
		gyroSampler = gyro.getAngleAndRateMode();
		sampleGyro = new float[gyroSampler.sampleSize()];
		desk = new Desk(distanceSensor);

		goToLabyrinth();
		motorL.close();
		motorR.close();
		distanceSensor.close();
		touch.close();
	}

	private void goToLabyrinth() {
		motorSmall.resetTachoCount();
		motorSmall.rotate(-90); // nastavi US sensor doprava
		while (true) {
			rideWhile();
			rotate();
		}
	}

	/*
	 * Jede, dokud: 
	 * -neni kam zatocit 
	 * -nenarazi 
	 * -jel by znovu, kde uz byl
	 */
	private void rideWhile() {
		int help;
		motorL.resetTachoCount();
		
		distanceSampler.fetchSample(sampleDistance, 0);
		help = desk.control();
		if (help == 0) {
			motorsForward();
		}
		
		distanceSampler.fetchSample(sampleDistance, 0);
		if ((sampleDistance[0] > 0.3) && (help == 0)) { //prejede ke stene; uz se to pocita do prejezdu
			Delay.msDelay(800);
		}
		
		do {
			help = desk.control();
		} while ((motorL.getTachoCount() < 150) && (help == 0));
		
		if (help == 0) {
			measure();
			desk.move();
		}
		
		help = desk.control();
		desk.turnOffLight();
		motorL.resetTachoCount();
		System.out.println("desk1 " + desk.getRow() + " " + desk.getColumn());
		
		do {
			if (motorL.getTachoCount() > 550) { //puvodne 573
				measure();
				desk.move();
				help = desk.control();
				desk.turnOffLight();
				System.out.println("desk2 " + desk.getRow() + " " + desk.getColumn());
				motorL.resetTachoCount();
			}
			distanceSampler.fetchSample(sampleDistance, 0);
			touchM.fetchSample(sampleTouch, 0);
		} while ((sampleDistance[0] < 0.3) && (sampleTouch[0] == 0) && (help == 0));
		
		if ((sampleDistance[0] > 0.3) || (help == 1)) {
			Delay.msDelay(700);
			desk.turnOffLight();
		}
		
		motorsStop();
		motorL.resetTachoCount();
	}

	/*
	 * zatoci doprava nebo doleva, kdyz:
	 * -to doprava nejde
	 * -vpravo uz byl
	 */
	private void rotate() {
		int help;
		int help1;
		int angle = -90 - motorSmall.getTachoCount();
		motorSmall.rotate(angle); // otoci US senzor doprava
		distanceSampler.fetchSample(sampleDistance, 0);
		help = desk.controlPresenceRight();  //bude 1, pokud tam robot jeste nebyl (vpravo)
		help1 = desk.control(); //bude 0, pokud tam robot jeste nebyl (policko pred nim)
		
		if ((sampleDistance[0] > 0.2) && (help == 1)) {
			rotationRight();
		} 
		
		if ((help1 == 1) || (help == 0)) {
			motorSmall.rotate(180); //otoci US senzor doleva
			Delay.msDelay(500);
			distanceSampler.fetchSample(sampleDistance, 0);
			if (sampleDistance[0] > 0.3) {
				rotationLeft();
			} else {
				rotation180();
			}
		}
	}

	/*
	 * dle natoceni US senzoru meri bud nalevo nebo napravo
	 */
	private void measure() {
		if (motorSmall.getTachoCount() > 80) {
			desk.measureLeft();
		} else if (motorSmall.getTachoCount() < -80) {
			desk.measureRight();
		}
	}

	/*
	 * spusti oba motory dopredu
	 */
	private void motorsForward() {
		motorL.startSynchronization();
		motorL.forward();
		motorR.forward();
		motorL.endSynchronization();
	}

	/*
	 * zastavi oba motory
	 */
	private void motorsStop() {
		motorL.startSynchronization();
		motorL.stop();
		motorR.stop();
		motorL.endSynchronization();
	}

	/*
	 * zatoci doleva (gyro)
	 */
	private void rotationLeft() {
		gyro.reset();
		gyroSampler.fetchSample(sampleGyro, 0);
		motorL.startSynchronization();
		motorL.backward();
		motorR.forward();
		motorL.endSynchronization();

		while (sampleGyro[0] < 79) {
			gyroSampler.fetchSample(sampleGyro, 0);
		}

		motorsStop();
		gyro.reset();
		Delay.msDelay(500);

		desk.rotationL();
	}

	/*
	 * zatoci doprava (gyro)
	 */
	private void rotationRight() {
		gyro.reset();
		gyroSampler.fetchSample(sampleGyro, 0);
		motorL.startSynchronization();
		motorL.forward();
		motorR.backward();
		motorL.endSynchronization();

		while (sampleGyro[0] > -73) {
			gyroSampler.fetchSample(sampleGyro, 0);
		}

		motorsStop();
		gyro.reset();
		Delay.msDelay(500);

		desk.rotationR();
	}

	/*
	 * otoci se o 180�
	 */
	private void rotation180() {
		gyro.reset();
		gyroSampler.fetchSample(sampleGyro, 0);
		motorL.startSynchronization();
		motorL.backward();
		motorR.forward();
		motorL.endSynchronization();

		while (sampleGyro[0] < 169) {
			gyroSampler.fetchSample(sampleGyro, 0);
		}

		motorsStop();
		gyro.reset();
		Delay.msDelay(500);

		desk.rotationB();
	}
}