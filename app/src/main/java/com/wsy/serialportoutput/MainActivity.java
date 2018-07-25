package com.wsy.serialportoutput;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.widget.TextView;

import java.io.File;

public class MainActivity extends AppCompatActivity {
    private static final String TAG = "MainActivity";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        TextView textView = findViewById(R.id.sample_text);

        File f = new File("/dev/ttyS0");
        if (f.exists()) {
            int firstSendLength = SerialPortUtil.sendBuffer(new byte[]{0x11, 0x22, 0x33}, f.getAbsolutePath(), 19200);
            Log.i(TAG, "onCreate: " + firstSendLength);
            int secondSendLength = SerialPortUtil.sendBuffer(new byte[]{0x44, 0x55, 0x66, 0x77}, f.getAbsolutePath(), 19200);
            Log.i(TAG, "onCreate: " + secondSendLength);
            textView.setText("" + firstSendLength + "   " + secondSendLength);
        } else {
            Log.i(TAG, "onCreate: " + f.getAbsolutePath() + " not exists!");
            textView.setText("serialport addr '" + f.getAbsolutePath() + "' not exists!");
        }

    }

}
