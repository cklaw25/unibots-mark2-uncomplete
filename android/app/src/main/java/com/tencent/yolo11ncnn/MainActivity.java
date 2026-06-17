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

import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;

public class MainActivity extends Activity implements SurfaceHolder.Callback
{
    public static final int REQUEST_CAMERA = 100;

    private YOLO11Ncnn yolo11ncnn = new YOLO11Ncnn();
    private int facing = 0;

    private Spinner spinnerTask;
    private Spinner spinnerModel;
    private Spinner spinnerCPUGPU;
    private int current_task = 0;
    private int current_model = 0;
    private int current_cpugpu = 0;

    private SurfaceView cameraView;
    private boolean isStreaming = false;

    private static final long PART1_DURATION_MS = 90000; // Task 5 runs 90s then switches to Task 6
    private static final long PART2_DURATION_MS = 90000; // Task 6 runs 90s then switches back to Task 5
    private Handler autoSwitchHandler = new Handler();

    private Runnable switchToPart2 = new Runnable() {
        @Override
        public void run() {
            if (current_task == 5) {
                Log.i("AutoSwitch", "Part 1 90s done — switching to Part 2 (Task 6)");
                spinnerTask.setSelection(6);
            }
        }
    };

    private Runnable switchToPart1 = new Runnable() {
        @Override
        public void run() {
            if (current_task == 6) {
                Log.i("AutoSwitch", "Part 2 90s done — switching back to Part 1 (Task 5)");
                spinnerTask.setSelection(5);
            }
        }
    };

    private Runnable pollDeliveryComplete = new Runnable() {
        @Override
        public void run() {
            if (current_task == 6) {
                if (yolo11ncnn.checkAndClearPart1Switch()) {
                    Log.i("AutoSwitch", "Delivery complete — switching back to Part 1 immediately");
                    spinnerTask.setSelection(5);
                } else {
                    autoSwitchHandler.postDelayed(this, 500);
                }
            }
        }
    };

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        cameraView = (SurfaceView) findViewById(R.id.cameraview);

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
                if (isStreaming)
                {
                    yolo11ncnn.stopUdp();
                    isStreaming = false;
                    buttonStream.setText("Stream");
                }
                else
                {
                    boolean ok = yolo11ncnn.startUdp("192.168.4.1", 4210);
                    if (ok)
                    {
                        isStreaming = true;
                        buttonStream.setText("Stop");
                    }
                }
            }
        });

        final Button buttonUsb = (Button) findViewById(R.id.buttonUsb);
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
            }
        });

        spinnerTask = (Spinner) findViewById(R.id.spinnerTask);
        spinnerTask.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> arg0, View arg1, int position, long id)
            {
                if (position != current_task)
                {
                    current_task = position;
                    reload();
                }
            }

            @Override
            public void onNothingSelected(AdapterView<?> arg0)
            {
            }
        });

        spinnerModel = (Spinner) findViewById(R.id.spinnerModel);
        spinnerModel.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> arg0, View arg1, int position, long id)
            {
                if (position != current_model)
                {
                    current_model = position;
                    reload();
                }
            }

            @Override
            public void onNothingSelected(AdapterView<?> arg0)
            {
            }
        });

        spinnerCPUGPU = (Spinner) findViewById(R.id.spinnerCPUGPU);
        spinnerCPUGPU.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> arg0, View arg1, int position, long id)
            {
                if (position != current_cpugpu)
                {
                    current_cpugpu = position;
                    reload();
                }
            }

            @Override
            public void onNothingSelected(AdapterView<?> arg0)
            {
            }
        });

        reload();
    }

    private void reload()
    {
        // Cancel any pending auto-switches
        autoSwitchHandler.removeCallbacks(switchToPart2);
        autoSwitchHandler.removeCallbacks(switchToPart1);
        autoSwitchHandler.removeCallbacks(pollDeliveryComplete);

        if (current_task == 5) {
            autoSwitchHandler.postDelayed(switchToPart2, PART1_DURATION_MS);
            Log.i("AutoSwitch", "Part 1 started — switching to Part 2 in 90s");
        } else if (current_task == 6) {
            autoSwitchHandler.postDelayed(switchToPart1, PART2_DURATION_MS);
            autoSwitchHandler.postDelayed(pollDeliveryComplete, 500);
            Log.i("AutoSwitch", "Part 2 started — switching back to Part 1 in 90s (or on delivery complete)");
        }

        boolean ret_init = yolo11ncnn.loadModel(getAssets(), current_task, current_model, current_cpugpu);
        if (!ret_init)
        {
            Log.e("MainActivity", "yolo11ncnn loadModel failed");
        }
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height)
    {
        yolo11ncnn.setOutputWindow(holder.getSurface());
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder)
    {
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder)
    {
    }

    @Override
    public void onResume()
    {
        super.onResume();

        if (ContextCompat.checkSelfPermission(getApplicationContext(), Manifest.permission.CAMERA) == PackageManager.PERMISSION_DENIED)
        {
            ActivityCompat.requestPermissions(this, new String[] {Manifest.permission.CAMERA}, REQUEST_CAMERA);
        }

        yolo11ncnn.openCamera(facing);
    }

    @Override
    public void onPause()
    {
        super.onPause();

        autoSwitchHandler.removeCallbacks(switchToPart2);
        autoSwitchHandler.removeCallbacks(switchToPart1);
        autoSwitchHandler.removeCallbacks(pollDeliveryComplete);

        if (isStreaming)
        {
            yolo11ncnn.stopUdp();
            isStreaming = false;
        }

        UsbSerialHelper.getInstance().disconnect();

        yolo11ncnn.closeCamera();
    }
}
