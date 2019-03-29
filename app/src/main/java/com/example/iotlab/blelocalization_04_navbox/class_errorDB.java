package com.example.iotlab.blelocalization_04_navbox;

import java.util.ArrayList;

/**
 * Created by IoTLAB on 2017-02-10.
 */

public class class_errorDB {
    ArrayList<Location2D> targetArray = new ArrayList<>();
    ArrayList<Location2D> estimationArray = new ArrayList<>();
    Location2D marker = new Location2D(0,0);
    Location2D localizedMarker = new Location2D(0,0);
    private float avgResult = 0;

    public class_errorDB(Location2D _marker){
        this.avgResult = 0;
        this.marker=_marker;
    }

    public void addErrorDB(Location2D _estimatedLocation){
            targetArray.add(marker);
            estimationArray.add(_estimatedLocation);
            calculateAverageError();

    }

    public void setMarker(Location2D _newMarkerLocation){
        this.marker = _newMarkerLocation;
    }

    public void setLocalized(Location2D _newLocalizedMarker){
        this.localizedMarker= _newLocalizedMarker;
    }

    public void undoErrorDB(){
        targetArray.remove(targetArray.size()-1);
        estimationArray.remove(estimationArray.size()-1);
        calculateAverageError();
    }

    public void clearDB(){
        targetArray.clear();
        estimationArray.clear();
        this.avgResult=0;
    }

    public float getAvgErrorResult(){
        return avgResult;
    }

    public float getLastErrorfromDB(){
        return getDistance(targetArray.get(targetArray.size()-1),estimationArray.get(estimationArray.size()-1));
    }

    public float getLastErrorfromMarker(){
        return getDistance(marker,localizedMarker);
    }

    public float getErrorfromIndex(int i){
        if(i>0 && targetArray.size()>i){
            return getDistance(targetArray.get(i),estimationArray.get(i));
        }else{
            return -1;
        }
    }

    public float calculateAverageError(){
        float result = 0;
        for (int i = 0 ;  i < targetArray.size();i++){
            result = result+getErrorfromIndex(i);
        }
        this.avgResult = result/targetArray.size();
        return result/targetArray.size();
    }

    private float getDistance(Location2D loc1, Location2D loc2){
        return (float) Math.sqrt((loc1.getX() - loc2.getX())*(loc1.getX() - loc2.getX()) + (loc1.getY() - loc2.getY())*(loc1.getY() - loc2.getY()));
    }

}
