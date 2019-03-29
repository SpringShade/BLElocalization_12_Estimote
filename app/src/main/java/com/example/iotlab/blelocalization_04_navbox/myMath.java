package com.example.iotlab.blelocalization_04_navbox;

import android.app.Activity;
import android.content.Context;

/**
 * Created by Kyuchang on 2017-12-22.
 */

public class myMath {
    private Context mContext;
    private android.app.Activity mActivity;

    public myMath(Context context, Activity activity){
        this.mContext = context;
        this.mActivity = activity;
    }

    //좌표사이 중점 반환
    private Location2D getCenterLocation(Location2D a, Location2D b){
        return new Location2D((a.getX() + b.getX())/2,(a.getY() + b.getY())/2);
    }

    //삼중점 반환
    private Location2D getCenterLocation(Location2D a, Location2D b, Location2D c){
        return new Location2D((a.getX() + b.getX()+ c.getX())/3,(a.getY() + b.getY() + c.getY())/3);
    }

}
