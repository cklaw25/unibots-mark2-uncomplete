package com.tencent.yolo11ncnn;

import android.content.Context;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.util.Log;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.List;

public class UsbSerialHelper {
    private static final String TAG = "UsbSerialHelper";
    private static UsbSerialHelper instance;
    private UsbSerialPort port;
    private boolean connected = false;

    private UsbSerialHelper() {}

    public static UsbSerialHelper getInstance() {
        if (instance == null) instance = new UsbSerialHelper();
        return instance;
    }

    public boolean connect(Context context) {
        disconnect();
        UsbManager manager = (UsbManager) context.getSystemService(Context.USB_SERVICE);
        List<UsbSerialDriver> drivers = UsbSerialProber.getDefaultProber().findAllDrivers(manager);
        if (drivers.isEmpty()) {
            Log.e(TAG, "No USB serial device found — is the OTG cable plugged in?");
            return false;
        }
        UsbSerialDriver driver = drivers.get(0);
        UsbDeviceConnection connection = manager.openDevice(driver.getDevice());
        if (connection == null) {
            Log.e(TAG, "USB permission denied — unplug and replug to get the permission dialog");
            return false;
        }
        port = driver.getPorts().get(0);
        try {
            port.open(connection);
            port.setParameters(115200, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
            connected = true;
            Log.i(TAG, "USB serial connected at 115200 baud");
            return true;
        } catch (IOException e) {
            Log.e(TAG, "Failed to open port: " + e.getMessage());
            port = null;
            connected = false;
            return false;
        }
    }

    public void disconnect() {
        if (port != null) {
            try { port.close(); } catch (IOException ignored) {}
            port = null;
        }
        connected = false;
    }

    public boolean isConnected() {
        return connected && port != null;
    }

    // Called from native (C++) code via JNI on the camera thread
    public void send(String pkt) {
        if (!isConnected()) return;
        try {
            port.write(pkt.getBytes(StandardCharsets.UTF_8), 100);
        } catch (IOException e) {
            Log.e(TAG, "USB write failed: " + e.getMessage());
            connected = false;
        }
    }
}
