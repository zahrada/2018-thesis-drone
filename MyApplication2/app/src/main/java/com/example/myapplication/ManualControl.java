package com.example.myapplication;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Intent;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.Set;
import java.util.UUID;

public class ManualControl extends AppCompatActivity {

    private static final String TAG = "ManualControl";

/*
    BluetoothDevice mBTDevice;
    private BluetoothAdapter bAdapter = BluetoothAdapter.getDefaultAdapter();
    public ArrayList<BluetoothDevice> mBTDevices = new ArrayList<>();
    BluetoothConnectionService mBluetoothConnection;

    private static final UUID MY_UUID_INSECURE =
            UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
*/
    Button backButton;
    TextView manualDataLeft;
    TextView manualDataRight;

    Integer pitchSend, rollSend, yawSend, throttleSend;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.manual_control);

        backButton = (Button) findViewById(R.id.backButtonMan);

        // write pitch, roll, yaw and throttle
        manualDataLeft = (TextView) findViewById(R.id.manualDataLeft);
        manualDataRight = (TextView) findViewById(R.id.manualDataRight);

        backButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Intent intent = new Intent(ManualControl.this,MainActivity.class);
                startActivity(intent);

            }
        });
/*
        mBTDevices.clear();
        if (bAdapter == null) {
            Toast.makeText(getApplicationContext(), "Bluetooth Not Supported", Toast.LENGTH_SHORT).show();
        } else {
            Set<BluetoothDevice> pairedDevices = bAdapter.getBondedDevices();
            if (pairedDevices.size() > 0) {
                for (BluetoothDevice device : pairedDevices) {
                    mBTDevices.add(device);
                }
            }
        }

        int pos = getIntent().getExtras().getInt("pos");

        mBTDevice = mBTDevices.get(pos);


        Log.d(TAG, "import text" + mBTDevice.getName());
        Toast.makeText(getApplicationContext(), mBTDevice.getName(), Toast.LENGTH_SHORT).show();

        mBTDevice.createBond();

        mBluetoothConnection = new BluetoothConnectionService(ControlDrone.this);

        mBluetoothConnection.startClient(mBTDevice,MY_UUID_INSECURE);

*/

        JoystickView joystickLeft = (JoystickView) findViewById(R.id.joystickLeft);
        joystickLeft.setOnMoveListener(new JoystickView.OnMoveListener() {
            @Override
            public void onMove(int roll, int pitch) {
                // do whatever you want

                rollSend = roll/8;
                if(rollSend > 99){
                    rollSend = 99;
                }

                pitchSend = (-pitch+800)/8;

                if(pitchSend > 99){
                    pitchSend = 99;
                }
                roll = (roll - 400)/16;
                pitch = (-pitch + 400)/16;
                manualDataLeft.setText("Roll " + roll + " Pitch " + pitch);
            }
        });

        JoystickView joystickRight = (JoystickView) findViewById(R.id.joystickRight);
        joystickRight.setOnMoveListener(new JoystickView.OnMoveListener() {
            @Override
            public void onMove(int yaw, int throttle) {
                // do whatever you want
                if(throttle > 400){
                    throttle = 400;
                }

                throttleSend = -throttle/4 +100;

                if(throttleSend > 99){
                    throttleSend = 99;
                }

                yawSend = yaw/8;
                if(yawSend > 99){
                    yawSend = 99;
                }
                yaw = (yaw - 400)/16;
                throttle = (int) ((throttle-400)*(-2.5)+1000);
                String data = "<P" + pitchSend + "R" + rollSend + "T" + throttleSend + "Y" + yawSend + ">";

                manualDataRight.setText("Throttle " + throttle + " Yaw " + yaw);


                //byte[] bytes = data.getBytes();
                //mBluetoothConnection.write(bytes);

            }
        });


    }
}