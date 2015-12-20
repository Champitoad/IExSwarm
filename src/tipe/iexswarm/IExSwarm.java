/*
 * Copyright (C) 2015 Donato Pablo
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

package tipe.iexswarm;

import android.app.Activity;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Switch;
import android.widget.ProgressBar;
import android.widget.RadioButton;
import java.util.Set;
import java.util.UUID;
import java.lang.Thread;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.ByteBuffer;

public class IExSwarm extends Activity
{
    /* CONSTANTS */

    // Bluetooth
	private final int FAILED = 0, SUCCEEDED = 1; // Messages used by the connection Handler to manage the connection's state
	private final String DEVICE_NAME = "HC-06"; // Remote device's name
	private final UUID MY_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB"); // Identifier of the application for the remote device
    private int REQUEST_ENABLE_BT = 1; // Value used to identify a request to enable the Bluetooth adapter

    // Directions for the accelerometer
	private final int LEFT = 0, RIGHT = 1, STOP = 2;

    // Application tag displayed in logcat
    private static final String TAG = "IExSwarm";
	
	/* VARIABLES */

	private boolean connectionEstablished = false; // Whether the Bluetooth connection is established
	private int yDirection; // Current direction of the accelerometer
    float px = 0; // Previous accelerometer value on x axis

    // Bluetooth
	private BluetoothAdapter bluetoothAdapter;
	private ConnectThread connectThread; // Thread establishing the Bluetooth connection
	private ConnectedThread connectedThread; // Thread managing the established Bluetooth connection

    // Accelerometer
	private SensorManager sensorManager;
	private Sensor accelerometer;

    // GUI
	private TextView connectionState;
    private Switch accelSwitch;
    private ProgressBar xBackwardSpeedProgressBar, xForwardSpeedProgressBar;
    private RadioButton rotateLeftRadioButton, rotateRightRadioButton;
    private Button connectButton, disconnectButton, cancelButton;

	/* METHODS */
	
	@Override
	public void onCreate(Bundle savedInstanceState)
    {
		super.onCreate(savedInstanceState);
        displayConnectionLayout();
		
		bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        if (!bluetoothAdapter.isEnabled()) { 
            Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enableBtIntent, REQUEST_ENABLE_BT);
        }

        sensorManager = (SensorManager)getSystemService(SENSOR_SERVICE); 
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER); 
	}

    protected void onActivityResult(int requestCode, int resultCode, Intent data)
    {   
        if (requestCode == REQUEST_ENABLE_BT && resultCode == RESULT_CANCELED)
            finish();
    }
	
	protected void onResume()
    {
        super.onResume();
		if (connectionEstablished) { 
        	sensorManager.registerListener(sensorEventListener, accelerometer, SensorManager.SENSOR_DELAY_NORMAL);
		}
    }

    protected void onPause()
    {
        super.onPause();
		if (connectionEstablished) { 
        	sensorManager.unregisterListener(sensorEventListener);
		}
    }

    private void displayConnectionLayout()
    {
        setContentView(R.layout.connection);

		connectionState = (TextView)findViewById(R.id.connectionState);
		connectButton = (Button)findViewById(R.id.connectButton);
		cancelButton = (Button)findViewById(R.id.cancelButton);
    }

    private void displayControlsLayout()
    {
        setContentView(R.layout.controls);

        xBackwardSpeedProgressBar = (ProgressBar)findViewById(R.id.xBackwardSpeedProgressBar);
        xBackwardSpeedProgressBar.setRotation(180);
        xForwardSpeedProgressBar = (ProgressBar)findViewById(R.id.xForwardSpeedProgressBar);
        
        rotateLeftRadioButton = (RadioButton)findViewById(R.id.rotateLeftRadioButton);
        rotateRightRadioButton = (RadioButton)findViewById(R.id.rotateRightRadioButton);

        accelSwitch = (Switch)findViewById(R.id.accelSwitch);
    }

	public void onAccelSwitchChanged(View view)
    {
        connectedThread.sendCommand("s");

        if (accelSwitch.isChecked())
            sensorManager.registerListener(sensorEventListener, accelerometer, SensorManager.SENSOR_DELAY_NORMAL); 
        else
        	sensorManager.unregisterListener(sensorEventListener); 

        xBackwardSpeedProgressBar.setProgress(0);
        xForwardSpeedProgressBar.setProgress(0);
        rotateLeftRadioButton.setChecked(false);
        rotateRightRadioButton.setChecked(false);
	}

	public void connect(View view)
    {
		launchConnection();
	}

	public void disconnect(View view)
    {
        stopConnection();
	}

    public void cancel(View view)
    {
        stopConnection();
    }

	private SensorEventListener sensorEventListener = new SensorEventListener()
    {
		public void onAccuracyChanged(Sensor sensor, int accuracy) {}

		public void onSensorChanged(SensorEvent se)
        {
			updateAccel(se.values[0], se.values[1]);
		}
	};
    
	public void updateAccel(float x, float y)
    {
		if (y < -3) {
			if (yDirection != LEFT) {
				connectedThread.sendCommand("l");
                rotateLeftRadioButton.setChecked(true);
                rotateRightRadioButton.setChecked(false);
				yDirection = LEFT;
			}
		} else if (y > 3) {
			if (yDirection != RIGHT) {
				connectedThread.sendCommand("r");
                rotateRightRadioButton.setChecked(true);
                rotateLeftRadioButton.setChecked(false);
				yDirection = RIGHT;
			}
		} else {
			if (yDirection != STOP) {
				connectedThread.sendCommand("s");
                rotateLeftRadioButton.setChecked(false);
                rotateRightRadioButton.setChecked(false);
				yDirection = STOP;
			}
		}
        
        float tmp = x;
        int T = accelerometer.getMinDelay();
        float tau = (float)100 / T * 1000000;
        x = (1 / (tau + T)) * (T * x + tau * px);
        px = tmp;

        if (x > 0) {
            xBackwardSpeedProgressBar.setProgress((int)(Math.abs(x * 50)));
            xForwardSpeedProgressBar.setProgress(0);
        } else {
            xForwardSpeedProgressBar.setProgress((int)(Math.abs(x * 50)));
            xBackwardSpeedProgressBar.setProgress(0);
        }

        connectedThread.sendCommand("v");
        int nx = (int)((x + 12) * 4);
        connectedThread.sendCommand("" + (char)nx);
	}

	public void launchConnection()
    {
		connectButton.setVisibility(View.GONE);
        connectionState.setVisibility(View.VISIBLE);
		connectionState.setText("Connecting...");
        cancelButton.setVisibility(View.VISIBLE);

        boolean deviceFound = false;
		Set<BluetoothDevice> pairedDevices = bluetoothAdapter.getBondedDevices(); 
		for (BluetoothDevice device : pairedDevices) { 
			if (device.getName().equals(DEVICE_NAME)) {
				connectThread = new ConnectThread(device); 
				connectThread.start();
                deviceFound = true;
			}
		}
        if (!deviceFound)
            connectionHandler.sendMessage(connectionHandler.obtainMessage(FAILED)); 
	}

    public void stopConnection()
    {
        connectThread.cancel();
        if (connectedThread != null)
            connectedThread.cancel();

        displayConnectionLayout();
    }
    
	private Handler connectionHandler = new Handler()
    { 
		@Override
		public void handleMessage(Message msg)
        {
            switch (msg.what) {
                case SUCCEEDED:
                    displayControlsLayout();
                    connectionEstablished = true; 
                    sensorManager.registerListener(sensorEventListener, accelerometer, SensorManager.SENSOR_DELAY_NORMAL); 
                    break;
                case FAILED:
                    connectionState.setText("Connection failed");
                    cancelButton.setVisibility(View.GONE);
                    connectButton.setVisibility(View.VISIBLE);
                    connectButton.setText("Retry");
                    break;
            }
		}
	};
    
	private class ConnectThread extends Thread
    {
        boolean cancelled = false; // True when cancel() is called

		private final BluetoothSocket mmSocket;
		private final BluetoothDevice mmDevice;

		public ConnectThread(BluetoothDevice device)
        {
			BluetoothSocket tmp = null;
			mmDevice = device;

			try {
				tmp = mmDevice.createRfcommSocketToServiceRecord(MY_UUID); 
			} catch (IOException e) {}

			mmSocket = tmp;
		}

		public void run()
        { 
			try {
				mmSocket.connect(); 
			} catch (IOException connectException) { 
				try {
					mmSocket.close(); 
				} catch (IOException closeException) {}
                if (cancelled)
                    cancelled = false;
                else
                    connectionHandler.sendMessage(connectionHandler.obtainMessage(FAILED)); 
				return;
			}

			connectedThread = new ConnectedThread(mmSocket); 
			connectedThread.start(); 
			connectionHandler.sendMessage(connectionHandler.obtainMessage(SUCCEEDED)); 
		}

		public void cancel()
        { 
			try {
                connectionEstablished = false;
                cancelled = true;
				mmSocket.close(); 
			} catch (IOException e) {}
		}
	}
    
	private class ConnectedThread extends Thread
    {
		private final BluetoothSocket mmSocket;
		private final OutputStream mmOutStream;

		public ConnectedThread(BluetoothSocket socket)
        {
			mmSocket = socket;
			OutputStream tmpOut = null;

			try {
				tmpOut = socket.getOutputStream(); 
			} catch (IOException e) {}

			mmOutStream = tmpOut;
		}

		public void run() {}
        
		public void sendCommand(String command)
        { 
			try {
				mmOutStream.write(command.getBytes());
			} catch (IOException e) {}
		}
        
		public void write(byte[] bytes)
        {
			try {
				mmOutStream.write(bytes);
			} catch (IOException e) {}
		}

		public void cancel()
        { 
			try {
				mmSocket.close(); 
			} catch (IOException e) {}
		}
	}
}
