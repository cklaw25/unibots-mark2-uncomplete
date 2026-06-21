// Tencent is pleased to support the open source community by making ncnn available.
//
// Copyright (C) 2025 THL A29 Limited, a Tencent company. All rights reserved.
//
// Licensed under the BSD 3-Clause License (the "License"); you may not use this file except
// in compliance with the License. You may obtain a copy of the License at
//
// https://opensource.org/licenses/BSD-3-Clause
//
// Unless required by applicable law or agreed to in writing, software distributed
// under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

package com.tencent.yolo11ncnn;

import android.Manifest;
import android.app.Activity;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.graphics.PixelFormat;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.WindowManager;
import android.widget.AdapterView;
import android.widget.Button;
import android.widget.Spinner;
import android.widget.TextView;

import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;

public class MainActivity extends Activity implements SurfaceHolder.Callback
{
    public static final int REQUEST_CAMERA = 100;

    // Task 5 = ball detection. Board A handles Mode 1/2 switching autonomously.
    // The phone stays on ball detection the entire match.
    private static final int BALL_DETECT_TASK = 5;

    private YOLO11Ncnn yolo11ncnn = new YOLO11Ncnn();
    private int facing = 0;

    private Spinner spinnerTask;
    private Spinner spinnerModel;
    private Spinner spinnerCPUGPU;
    private int current_task   = 0;
    private int current_model  = 0;
    private int current_cpugpu = 0;

    private SurfaceView cameraView;
    private Button      buttonUsb;
    private TextView    textStatusUsb;
    private boolean     isStreaming = false;

    // Polls USB connection state every 2 seconds and updates the status label
    private final Handler  statusHandler = new Handler();
    private final Runnable statusPoller  = new Runnable() {
        @Override
        public void run() {
            updateUsbStatus();
            statusHandler.postDelayed(this, 2000);
        }
    };

    // ── Status display ─────────────────────────────────────────────────────────

    private void updateUsbStatus() {
        boolean connected = UsbSerialHelper.getInstance().isConnected();
        if (textStatusUsb == null) return;
        if (connected) {
            textStatusUsb.setText("ESP32: Connected");
            textStatusUsb.setTextColor(Color.rgb(0, 160, 0));
        } else {
            textStatusUsb.setText("ESP32: Not connected");
            textStatusUsb.setTextColor(Color.rgb(180, 0, 0));
        }
    }

    // ── Activity lifecycle ──────────────────────────────────────────────────────

    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        cameraView    = (SurfaceView) findViewById(R.id.cameraview);
        textStatusUsb = (TextView)    findViewById(R.id.textStatusUsb);

        cameraView.getHolder().setFormat(PixelFormat.RGBA_8888);
        cameraView.getHolder().addCallback(this);

        Button buttonSwitchCamera = (Button) findViewById(R.id.buttonSwitchCamera);
        buttonSwitchCamera.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View arg0) {
                int new_facing = 1 - facing;
                yolo11ncnn.closeCamera();
                yolo11ncnn.openCamera(new_facing);
                facing = new_facing;
            }
        });

        final Button buttonStream = (Button) findViewById(R.id.buttonStream);
        buttonStream.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View arg0) {
                if (isStreaming) {
                    yolo11ncnn.stopUdp();
                    isStreaming = false;
                    buttonStream.setText("Stream");
                } else {
                    boolean ok = yolo11ncnn.startUdp("192.168.4.1", 4210);
                    if (ok) {
                        isStreaming = true;
                        buttonStream.setText("Stop");
                    }
                }
            }
        });

        buttonUsb = (Button) findViewById(R.id.buttonUsb);
        buttonUsb.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View arg0) {
                if (UsbSerialHelper.getInstance().isConnected()) {
                    UsbSerialHelper.getInstance().disconnect();
                    buttonUsb.setText("USB");
                } else {
                    boolean ok = UsbSerialHelper.getInstance().connect(getApplicationContext());
                    buttonUsb.setText(ok ? "USB: ON" : "USB: FAIL");
                }
                updateUsbStatus();
            }
        });

        spinnerTask = (Spinner) findViewById(R.id.spinnerTask);
        spinnerTask.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> arg0, View arg1, int position, long id) {
                if (position != current_task) {
                    current_task = position;
                    reload();
                }
            }
            @Override
            public void onNothingSelected(AdapterView<?> arg0) {}
        });

        spinnerModel = (Spinner) findViewById(R.id.spinnerModel);
        spinnerModel.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> arg0, View arg1, int position, long id) {
                if (position != current_model) {
                    current_model = position;
                    reload();
                }
            }
            @Override
            public void onNothingSelected(AdapterView<?> arg0) {}
        });

        spinnerCPUGPU = (Spinner) findViewById(R.id.spinnerCPUGPU);
        spinnerCPUGPU.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> arg0, View arg1, int position, long id) {
                if (position != current_cpugpu) {
                    current_cpugpu = position;
                    reload();
                }
            }
            @Override
            public void onNothingSelected(AdapterView<?> arg0) {}
        });

        // Auto-select ball detection task — teammates don't need to touch this
        spinnerTask.setSelection(BALL_DETECT_TASK);

        reload();
    }

    private void reload() {
        boolean ok = yolo11ncnn.loadModel(getAssets(), current_task, current_model, current_cpugpu);
        if (!ok) Log.e("MainActivity", "loadModel failed");
    }

    @Override
    public void onResume()
    {
        super.onResume();

        if (ContextCompat.checkSelfPermission(getApplicationContext(), Manifest.permission.CAMERA)
                == PackageManager.PERMISSION_DENIED) {
            ActivityCompat.requestPermissions(this,
                    new String[]{ Manifest.permission.CAMERA }, REQUEST_CAMERA);
        }

        yolo11ncnn.openCamera(facing);

        // Auto-connect USB on resume. If it fails first time (permission not yet granted),
        // user presses the USB button — the OS permission dialog will appear, approve it,
        // then press USB button once more (or replug cable).
        if (!UsbSerialHelper.getInstance().isConnected()) {
            boolean ok = UsbSerialHelper.getInstance().connect(getApplicationContext());
            if (ok) {
                Log.i("MainActivity", "USB serial auto-connected.");
                buttonUsb.setText("USB: ON");
            }
        }

        updateUsbStatus();
        statusHandler.postDelayed(statusPoller, 2000);   // start polling
    }

    @Override
    public void onPause()
    {
        super.onPause();

        statusHandler.removeCallbacks(statusPoller);   // stop polling

        if (isStreaming) {
            yolo11ncnn.stopUdp();
            isStreaming = false;
        }

        UsbSerialHelper.getInstance().disconnect();
        yolo11ncnn.closeCamera();
    }

    // ── SurfaceHolder callbacks ─────────────────────────────────────────────────

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
        yolo11ncnn.setOutputWindow(holder.getSurface());
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {}

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {}
}
