package com.example.myapplication;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.nfc.Tag;
import android.os.Build;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.Set;
import java.util.UUID;

public class MainActivity extends AppCompatActivity implements AdapterView.OnItemClickListener{

    private static final String TAG = "MainActivity";

    Button  btnStartConnection;

    private ListView lstvw;
    private BluetoothAdapter bAdapter = BluetoothAdapter.getDefaultAdapter();
    public DeviceListAdapter mDeviceListAdapter;
    public ArrayList<BluetoothDevice> mBTDevices = new ArrayList<>();
    BluetoothDevice mBTDevice;
    BluetoothConnectionService mBluetoothConnection;

    private static final UUID MY_UUID_INSECURE =
            UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Image Arduino
        ImageView imageArduino = (ImageView) findViewById(R.id.imageArduino);
        int imageResource = getResources().getIdentifier("@drawable/dron",null, this.getPackageName());
        imageArduino.setImageResource(imageResource);

        // Using textView
        TextView myTextView = (TextView) findViewById(R.id.textWiewMainActiv);
        myTextView.setText("Connection");
        String stringfromTextView = myTextView.getText().toString();

        Log.d(TAG, "import text" + stringfromTextView);

        //Buttons
        Button blueButton = (Button) findViewById(R.id.bluebutton);
        btnStartConnection = (Button) findViewById(R.id. btnStartConnection);

        //Bluetooth buttons do something
        blueButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view){
                Log.d(TAG, "Bluetooth selected");
                Toast.makeText(MainActivity.this, "Bluetooth selected", Toast.LENGTH_SHORT).show();

                Intent intent = new Intent(MainActivity.this, BlueActivity.class);

                startActivity(intent);
            }
        });

        showPairedBTDevices();
        lstvw.setOnItemClickListener(MainActivity.this);
/*
        btnStartConnection.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                startConnection();
                Toast.makeText(MainActivity.this,"Connected with" + mBTDevice.getName(), Toast.LENGTH_SHORT).show();

            }
        });
        */
    }

    public  void showPairedBTDevices(){
        mBTDevices.clear();
        if(bAdapter==null){
            Toast.makeText(getApplicationContext(),"Bluetooth Not Supported",Toast.LENGTH_SHORT).show();
        }
        else{
            Set<BluetoothDevice> pairedDevices = bAdapter.getBondedDevices();
            if(pairedDevices.size()>0){
                for(BluetoothDevice device: pairedDevices){
                    mBTDevices.add(device);
                }
                lstvw = (ListView) findViewById(R.id.bondedDevices);

                mDeviceListAdapter = new DeviceListAdapter(getApplicationContext(), R.layout.device_adapter_view, mBTDevices);
                lstvw.setAdapter(mDeviceListAdapter);
            }
        }
    }

    @Override
    public void onItemClick(AdapterView<?> adapterView, View view, int i, long l) {
        //first cancel discovery because its very memory intensive.
        bAdapter.cancelDiscovery();

        Log.d(TAG, "onItemClick: You Clicked on a device.");
        String deviceName = mBTDevices.get(i).getName();
        String deviceAddress = mBTDevices.get(i).getAddress();

        Log.d(TAG, "onItemClick: deviceName = " + deviceName);
        Log.d(TAG, "onItemClick: deviceAddress = " + deviceAddress);

        //create the bond.
        //NOTE: Requires API 17+? I think this is JellyBean
        if(Build.VERSION.SDK_INT > Build.VERSION_CODES.JELLY_BEAN_MR2){
            /*
            Log.d(TAG, "Trying to pair with " + deviceName);
            mBTDevices.get(i).createBond();

            mBTDevice = mBTDevices.get(i);
            mBluetoothConnection = new BluetoothConnectionService(MainActivity.this);

            mBluetoothConnection.startClient(mBTDevice,MY_UUID_INSECURE);
*/
            Intent intent = new Intent(MainActivity.this,ControlDrone.class);
            intent.putExtra("pos",i);
            startActivity(intent);
            finish();
        }
    }


}