package com.wsy.serialportoutput;

public class SerialPortUtil {
    static {
        System.loadLibrary("native-lib");
    }
    public static native int sendBuffer(byte[] buffer,String addr,int baudRate);
}
