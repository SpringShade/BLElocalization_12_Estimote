package com.example.iotlab.blelocalization_04_navbox;

import android.app.Activity;
import android.content.Context;
import android.util.Log;
import android.widget.Toast;

import com.estimote.sdk.EstimoteSDK;
import com.estimote.sdk.SystemRequirementsChecker;
import com.estimote.sdk.connection.scanner.ConfigurableDevicesScanner;

import java.util.List;

/**
 * Created by Kyuchang on 2017-12-20.
 */

public class estimoteManager {
    //Member of Estimote (Native function from the estimote SDK library)
    private ConfigurableDevicesScanner devicesScanner;

    //Activity interfaces
    private Context mContext;
    private Activity mActivity;

    //DB for pairing
    private boolean isDetectionInitialized = false;
    private List<ConfigurableDevicesScanner.ScanResultItem> beaconItem;
    private ConfigurableDevicesScanner.ScanResultItem estimoteReferenceSpot;



    //Main
    public estimoteManager(Context context, Activity activity){
        //Conducting activity interfaces
        this.mContext = context;
        this.mActivity = activity;
        //Toast.makeText(mContext,"Start Estimote Manager", Toast.LENGTH_SHORT).show(); //Debug

        //Estimote APK permission from library (This script is mandatory to use the estimote beacon.)
        EstimoteSDK.initialize(mContext, "myconfigure00-hik", "94c69f77f6b3f78054d886873cab0293");
        EstimoteSDK.enableDebugLogging(false);

        //Estimote scanner initialization
        devicesScanner = new ConfigurableDevicesScanner(this.mContext);
        devicesScanner.setOwnDevicesFiltering(true);

        //Estimote scanner threading
        if (SystemRequirementsChecker.checkWithDefaultDialogs(this.mActivity)) {
            devicesScanner.scanForDevices(new ConfigurableDevicesScanner.ScannerCallback() {
                @Override
                public void onDevicesFound(List<ConfigurableDevicesScanner.ScanResultItem> _list) {
                    //list.size() //= The number of beacons   // debug
                    //Connection start
                    if (!_list.isEmpty()) {
                        isDetectionInitialized=true;
                        //Updating beacon DB
                        beaconItem = _list;     //This part might be heavy for the calculating
//                        for(ConfigurableDevicesScanner.ScanResultItem item : list){
//                            if(item.device.macAddress.toHexString() == _MAC){
//                                estimoteReferenceSpot =  item;
//                            }
//                        }
                        //For debug
                        //Toast.makeText(getApplicationContext(),"MAC : "+item.device.macAddress+" RSS," + item.rssi+" dBm", Toast.LENGTH_SHORT).show();
                    }else{
                        isDetectionInitialized=false;
                    }
                }   //end of onDevicesFound
            });
        }//end of if System

    }

    public ConfigurableDevicesScanner.ScanResultItem getSpotDevice(){
        return estimoteReferenceSpot;
    }

    public String getAnyMAC(){
        return beaconItem.get(0).device.macAddress+"";
    }


    public boolean isReferenceSpotInDB(String targetID){
        boolean result = false;
        for(ConfigurableDevicesScanner.ScanResultItem i : beaconItem){
            if((""+i.device.deviceId).equals("["+targetID+"]")){
                result = true;
                estimoteReferenceSpot = i;
            }
        }
        return result;
    }

    public boolean checkingID(String targetID){
        boolean result = false;
        for(ConfigurableDevicesScanner.ScanResultItem i : beaconItem){
            if((""+i.device.deviceId).equals("["+targetID+"]")) result = true;
        }
        return result;
    }

    //for debug with checkingID
    public void LoggingID(String targetID){
        boolean result = false;
        for(ConfigurableDevicesScanner.ScanResultItem i : beaconItem){
            Log.i("DB comparing logging",""+i.device.deviceId+" vs "+"["+targetID+"]");
            if((""+i.device.deviceId).equals("["+targetID+"]")){
                Log.i("DB comparing logging","Correct matching");
                result = true;
            }else{
                Log.i("DB comparing logging","Wrong matching");
            }
        }

        if(result == true){
            Log.i("DB comparing logging","Matching result success"+"\n"+"\n");
        }else{
            Log.i("DB comparing logging","Matching result failed"+"\n"+"\n");
        }
    }

    public List<ConfigurableDevicesScanner.ScanResultItem> getDB(){
        return beaconItem;
    }

    public String getCurrentSpotMAC(){
        if (estimoteReferenceSpot.equals(null)){
            return "None with null";
        }
        return estimoteReferenceSpot.device.macAddress.toHexString();
    }

    public int getEstimoteBeacons(){
        if(isDetectionInitialized){
            return beaconItem.size();
        }else{
            return 0;
        }
    }

    private List<ConfigurableDevicesScanner.ScanResultItem> getDatabase(){
        return beaconItem;
    }

    public ConfigurableDevicesScanner.ScanResultItem findReferenceSpot(){
        boolean _ready = false;
        if(!beaconItem.isEmpty()){
            for(ConfigurableDevicesScanner.ScanResultItem item : beaconItem) {
                //Current_list found?
                if((""+estimoteReferenceSpot.device.macAddress).equals(item.device.macAddress.toHexString())){
                    _ready =true;
                    return item;
                }
            }
        }else{
            _ready =false;
            return estimoteReferenceSpot;
        }
        return estimoteReferenceSpot;
    }

}
