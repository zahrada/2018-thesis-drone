package com.example.myapplication;

import android.Manifest;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.location.Location;
import android.os.Bundle;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;

import com.google.android.gms.common.ConnectionResult;
import com.google.android.gms.common.GoogleApiAvailability;
import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.MapView;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.MarkerOptions;

import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.Set;
import java.util.UUID;

public class AutonomControl extends AppCompatActivity implements OnMapReadyCallback{
    private static final String TAG = "AutonomControl";

    //Google maps
    private MapView mMapView;
    private static final String MAPVIEW_BUNDLE_KEY = "MapViewBundleKey";
    private static final int LOCATION_PERMISSION_REQUEST_CODE = 1;

    Button sendButton,homeButton, backButton;

    //EditText dataLog = (EditText) findViewById(R.id.dataLog);
    //EditText dataLat = (EditText) findViewById(R.id.dataLat);

    double lalitude = 50.0;
    double longitude = 14.0;


/*
    BluetoothDevice mBTDevice;
    private BluetoothAdapter bAdapter = BluetoothAdapter.getDefaultAdapter();
    public ArrayList<BluetoothDevice> mBTDevices = new ArrayList<>();
    BluetoothConnectionService mBluetoothConnection;

    private static final UUID MY_UUID_INSECURE =
            UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
*/


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.autonom_control);

        sendButton = (Button) findViewById(R.id.sendButton);
        homeButton = (Button) findViewById(R.id.homeButton);
        backButton = (Button) findViewById(R.id.backButtonAut);
/*

        sendButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                String lat = dataLat.getText().toString();
                String log = dataLog.getText().toString();

                longitude = Double.parseDouble(log);
                lalitude = Double.parseDouble(lat);

                String data = "<L" + lat.toString() + "G" + log.toString() + ">";

                dataLog.setText("Type Longitude");
                dataLat.setText("Type Latitude");

                //mBluetoothConnection.write(bytes);

            }
        });

*/

        backButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Intent intent = new Intent(AutonomControl.this,MainActivity.class);
                startActivity(intent);

            }
        });

        Bundle mapViewBundle = null;
        if (savedInstanceState != null) {
            mapViewBundle = savedInstanceState.getBundle(MAPVIEW_BUNDLE_KEY);
        }
        mMapView = (MapView) findViewById(R.id.mapAuto);
        mMapView.onCreate(mapViewBundle);
        mMapView.getMapAsync(this);

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


    }

    @Override
    public void onSaveInstanceState(Bundle outState) {
        super.onSaveInstanceState(outState);

        Bundle mapViewBundle = outState.getBundle(MAPVIEW_BUNDLE_KEY);
        if (mapViewBundle == null) {
            mapViewBundle = new Bundle();
            outState.putBundle(MAPVIEW_BUNDLE_KEY, mapViewBundle);
        }

        mMapView.onSaveInstanceState(mapViewBundle);
    }

    @Override
    protected void onResume() {
        super.onResume();
        mMapView.onResume();
    }

    @Override
    protected void onStart() {
        super.onStart();
        mMapView.onStart();
    }

    @Override
    protected void onStop() {
        super.onStop();
        mMapView.onStop();
    }

    @Override
    public void onMapReady(GoogleMap map) {
        map.addMarker(new MarkerOptions().position(new LatLng(lalitude, longitude)).title("Marker"));


        Location location;

        if (ContextCompat.checkSelfPermission(this,
                Manifest.permission.ACCESS_FINE_LOCATION)
                != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this,
                    new String[]{Manifest.permission.ACCESS_FINE_LOCATION,
                            Manifest.permission.ACCESS_FINE_LOCATION},
                    LOCATION_PERMISSION_REQUEST_CODE);
        } else if (map != null) {
            map.setMyLocationEnabled(true);
        }
        map.moveCamera(CameraUpdateFactory.newLatLngZoom(new LatLng(lalitude,longitude), 10000));
    }

    @Override
    protected void onPause() {
        mMapView.onPause();
        super.onPause();
    }

    @Override
    protected void onDestroy() {
        mMapView.onDestroy();
        super.onDestroy();
    }

    @Override
    public void onLowMemory() {
        super.onLowMemory();
        mMapView.onLowMemory();
    }
}
