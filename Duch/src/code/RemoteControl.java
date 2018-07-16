package code;

import java.io.*;
import lejos.hardware.Bluetooth;
import lejos.hardware.lcd.LCD;
import lejos.remote.nxt.NXTConnection;
import lejos.remote.nxt.BTConnection;


public class RemoteControl {

    String connected = "Connected";
    String waiting = "Waiting...";
    String closing = "Closing...";
    
    public RemoteControl() {
		while (true) {
			LCD.drawString(waiting,0,0);
			BTConnection connection = (BTConnection) Bluetooth.getNXTCommConnector().waitForConnection(0, NXTConnection.RAW);
			LCD.clear();
			LCD.drawString(connected,0,0);

			DataInputStream dis = connection.openDataInputStream();
			DataOutputStream dos = connection.openDataOutputStream();
		}
	}
    
    

}
  
