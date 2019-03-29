package com.example.iotlab.blelocalization_04_navbox;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * Created by Spring-shade on 2017-01-01.
 */

public class BleDatabase extends Location2D {
    //PHY
    private String ADRESS;
    private float lastRX_time = 0;
    private float before_time = 0;
    private float period_time = 0;
    //Parameter
    private int P0;
    private float MU;
    private final int MAX_WINDOWSIZE = 4;
    private final int SPIKE = 100;
    private float node_rmse = 0;
    private float rmse = 0;
    private boolean isNear = false;
    private boolean NLOSbeacon = false ;
    //for RSS
    private ArrayList<Integer> rss_table = new ArrayList<>();               //RSS data list
    private int avg_rss = 0;
    private int raw_rss = 0;
    private int pre_rss = 0;
    //for KF-RSS
    private ArrayList<Integer> rss_table_kalman = new ArrayList<>();        //Filtered RSS data list
    private double      prcs_init =     10;                                      //Process Noise Initial value   10
    private double      prcs_var  =   0.2;                                       //Process Noise Vari ance         0.6
    private double      meas_var  =   22.00;          //Measurement Noise Variance    6.0
    private double      gain_init =   1.00;          //Kalman gain init value        1.00
    private double      kalman_init = -70.00;        //Init                         -10.00
    private boolean     isKalman_init = true;
    private double      prcs = prcs_init;
    private double      kalman_data = kalman_init;
    //for Body shading azimuth(Not in use)
    private float       anchorAzimuth = 0;
    private boolean     bodyLOS = true;
    private int myAncNum = 0;
    //for RSS Rank
    private int myRank = -1;
    //forEstimoteMU table
    private String identifier = "";
    private HashMap<Integer, Double> spatiotemporalMU;   //DB for anchor nodes (Key,Address Code)

    //Initializing
    public BleDatabase(double x, double y, int P0, float MU, String ADD,int _myAncNum){
        super(x, y);
        this.P0 = P0;
        this.MU = MU;
        this.ADRESS=ADD;
        this.isNear=true;
        this.myAncNum = _myAncNum;
        this.spatiotemporalMU = new HashMap <>();
    }
    public BleDatabase(float x, float y, int P0, float MU, String ADD,int _myAncNum){
        super(x, y);
        this.P0 = P0;
        this.MU = MU;
        this.ADRESS=ADD;
        this.isNear=true;
        this.myAncNum = _myAncNum;
        this.spatiotemporalMU = new HashMap <>();
    }
    public BleDatabase(double x, double y, int P0, float MU, String ADD,int _myAncNum,String _ident){
        super(x, y);
        this.P0 = P0;
        this.MU = MU;
        this.ADRESS=ADD;
        this.isNear=true;
        this.myAncNum = _myAncNum;
        this.spatiotemporalMU = new HashMap <>();
        this.identifier = _ident;
    }
    public BleDatabase(float x, float y, int P0, float MU, String ADD,int _myAncNum, String _ident){
        super(x, y);
        this.P0 = P0;
        this.MU = MU;
        this.ADRESS=ADD;
        this.isNear=true;
        this.myAncNum = _myAncNum;
        this.spatiotemporalMU = new HashMap <>();
        this.identifier = _ident;
    }

    //Parameter information
    public int      getP0(){
        return P0;
    }              //Return current P0 value
    public float    getMU(){
        return MU;
    }            //Return current Mu value
    public String   getAdress() {
        return ADRESS;
    }
    public String   getID() {
        return identifier;
    }
    public boolean  isNLOSbeacon() {return NLOSbeacon; }
    public void  setNLOSbeacon(boolean _result) {NLOSbeacon = _result;}

    //RSS-information
    public void addRSS(int r) {     //getInfo (스택에 쌓기)
        pre_rss = raw_rss;
        raw_rss = r;

        if((rss_table.size() >= MAX_WINDOWSIZE) && (r>pre_rss+SPIKE || r<pre_rss-SPIKE)){
            //스파이크 자름.
            return;
        }

        rss_table.add(raw_rss);
        rss_table_kalman.add(KF(raw_rss));

        if (rss_table.size() > MAX_WINDOWSIZE) {
            rss_table.remove(0);
            rss_table_kalman.remove(0);
        }
    }               //Add RSS to Arraylist and KF-Arraylist

    public int getRSS() {       // (평균을 낸 스택값호출 - )
        avg_rss = 0;

        for (int i : rss_table) {
            avg_rss += i;
        }

        avg_rss /= rss_table.size();
        //avg_rss -= (max + min);
        //avg_rss /= (rss_table.size() - 2);

        return avg_rss;
    }                     //Return last avg RSS for information

    public int getRawRSS() {
        return raw_rss;
    }    //Return last Raw RSS for information

    public int get_aptRSS(int _buffer, boolean _bKF) {       // (평균을 낸 스택값호출 - )
        avg_rss = 0;

        if(_bKF){
            if(_buffer>rss_table_kalman.size()){
                //요구된 참조 buffer가 채워진 rss_table보다 많아 요구량보다 적게 Averaging해야 할 때
                for (int i : rss_table_kalman) avg_rss += i;
                avg_rss /= rss_table_kalman.size();
            }
            else if(_buffer >= MAX_WINDOWSIZE){
                //요구된 참조 buffer가 코딩상 정해진 MAX_Window보다 많아서 적게 Averaging해야 할 때
                for (int i : rss_table_kalman) avg_rss += i;
                avg_rss /= rss_table_kalman.size();
            }else{
                //일반적인 상황일때
                for (int i = rss_table_kalman.size()-_buffer ; i < rss_table_kalman.size() ;i++ ){
                    avg_rss += rss_table_kalman.get(i);
                }
                avg_rss /= _buffer;
            }
        }else{
            if(_buffer>rss_table.size()){
                //요구된 참조 buffer가 채워진 rss_table보다 많아 요구량보다 적게 Averaging해야 할 때
                for (int i : rss_table) avg_rss += i;
                avg_rss /= rss_table.size();
            }
            else if(_buffer >= MAX_WINDOWSIZE){
                //요구된 참조 buffer가 코딩상 정해진 MAX_Window보다 많아서 적게 Averaging해야 할 때
                for (int i : rss_table) avg_rss += i;
                avg_rss /= rss_table.size();
            }else{
                //일반적인 상황일때
                for (int i = rss_table.size()-_buffer ; i < rss_table.size() ;i++ ){
                    avg_rss += rss_table.get(i);
                }
                avg_rss /= _buffer;
            }
        }

        return avg_rss;
    }   //Return RSS or KF-RSS for MLE

    public int get_KF_RSS() {       // (평균을 낸 스택값호출 - )
        return rss_table_kalman.get(rss_table.size()-1);
    }                  //return last single KF-RSS

    public int KF(int _rss){

        if(isKalman_init){
            prcs =  _rss + prcs_var;
            isKalman_init = !isKalman_init;
        }else{
            prcs = prcs + prcs_var;
        }


        double gain = prcs/(prcs+meas_var);
        double kalman = (gain*_rss)+(1-gain)*kalman_data;
        prcs = (1-gain)*prcs;

        kalman_data = kalman;

        return (int)kalman;  //Filtered value of RSS
    }                   //Execute KF

    //Time information
    public void updateRXtime(float _time){
        this.before_time = lastRX_time;
        this.lastRX_time = _time;
        this.period_time = lastRX_time -before_time;
    }

    public float getRXperiod(){
        return period_time;
    }

    public float getDelay(float _currentTime){
        return _currentTime-lastRX_time;
    }

    //forAnchorAzimuth
    public void setAnchorAzimuth(Location2D _targetNodeLocation, float _Mapazimuth, float _threshold){
        float _result = 0;
        float radian = 0;
        this.getX();
        this.getY();


        if(getDistance(_targetNodeLocation,this)!=0){
            radian = (float)Math.acos((float)((this.getX()-_targetNodeLocation.getX())/(getDistance(_targetNodeLocation,this))));  //0~180 Radian callbac
            //_result = (float) ((radian/180)*Math.PI);
            _result = (float)radian;

            if(this.getY() > _targetNodeLocation.getY()) _result = _result*(-1);      //upperside = 1, down = -1 for android device


            this.anchorAzimuth = _result;
        }else{
            _result = 0;
            this.anchorAzimuth = _result;
        }

        if(Math.abs(_Mapazimuth-_result)>_threshold){    //음수일시 왼쪽 양수일시 오른쪽
            this.bodyLOS=false;     //body NLOS
        }else{
            this.bodyLOS=true;
        }

    }

    public boolean getBodyLOS(){
        return this.bodyLOS;
    }

    public float getAnchorAzimuthResult(){
        return this.anchorAzimuth;
    }

    //ETC   ----------------------------------------------------------------------------------------
    public void setrmse(float _rmse){
        this.rmse = _rmse;
    }

    public boolean isCalibrated() {
        if (rss_table.size() >= MAX_WINDOWSIZE) {
            return true;
        }
        return false;
    }

    public boolean getNear() {
        return isNear;
    }

    public float getNodeRMSE() {
        return node_rmse;
    }

    public void addNodeRMSE(float vl){
        node_rmse = vl;
    }

    private float getDistance(Location2D loc1, Location2D loc2){
        return (float) Math.sqrt((loc1.getX() - loc2.getX())*(loc1.getX() - loc2.getX()) + (loc1.getY() - loc2.getY())*(loc1.getY() - loc2.getY()));
    }

    public int sum(ArrayList<Integer> list) {
        int sum = 0;
        for(int i=0; i<list.size(); i++ ){
            sum = sum + list.get(i) ;
        }
        return sum;
    }

    public float variance(ArrayList<Integer> list, int _average) {
        // write code here
        float sumMinusAverage = sum(list) - _average;
        return sumMinusAverage * sumMinusAverage / (list.size()-1);
    }

    public float getRawVariance(){
        return variance(rss_table,avg_rss);
    }

    public float getKFVariance(){
        return variance(rss_table,avg_rss);
    }

    public int getMyNodeNumber(){
        return myAncNum;
    }


    public void setRank(int i){
        this.myRank = i;
    }

    public int getRank(){
        return this.myRank;
    }

    public HashMap<Integer, Double> getSpatiotemporalMU(){
        return spatiotemporalMU;
    }

    public void setSpatiotemporalMU(HashMap<Integer, Double> _newDB){
        this.spatiotemporalMU = _newDB;
    }

}


