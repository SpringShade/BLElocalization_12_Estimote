package com.example.iotlab.blelocalization_04_navbox;

/**
 * Created by IoTLAB on 2017-02-05.
 */

public class class_muDB {
    //RX is Prime node's number
    //TX is Ancrho node's number

    private float [][] DB;  //[RX], [TX]
    private float def           = (float) 2.1;

    private float lowRate       = (float) 2.0;
    private float normalRate    = (float) 2.1;
    private float highRate      = (float) 2.2;


    //Initialization
    public class_muDB(int _anchor_size, float _defalut){
        this.DB = new float[_anchor_size][_anchor_size];
        this.def = _defalut;

        for(int i=0 ; i<_anchor_size ; i++){
            for(int j=0 ; j<_anchor_size ; j++){
                DB[i][j] = _defalut;
            }
        }

    }   //End of Initialization

    //Manual DB setting
    public void setManualDB_1F5node(){

        //HardCoding (Needs Database)
        int A1 = 7, A2 = 11, A3 = 12, A4 = 13, A5 = 15, A6 = 16, A7 = 19, A8 = 22, A9 = 9;
        //A1 = A1-1; A2 = A2-1; A3 = A3-1; A4 = A4-1; A5 = A5-1; A6 = A6-1; A7 = A7-1; A8 = A8-1;     //2D Array starts from zero (0=1)

        //중앙

        DB[A9][A3]=(float)2.0;          DB[A9][A5]=(float)2.1;      //위왼쪽오른쪽

        DB[A9][A7] =(float) 2.0;        DB[A9][A1]=(float)2.1;      //아래왼쪽 오른쪽오른쪽


    }

    public void setManualDB(){      //for 8 Node

        //HardCoding (Needs Database)
        int A1 = 7, A2 = 11, A3 = 12, A4 = 13, A5 = 15, A6 = 16, A7 = 19, A8 = 22;
        //A1 = A1-1; A2 = A2-1; A3 = A3-1; A4 = A4-1; A5 = A5-1; A6 = A6-1; A7 = A7-1; A8 = A8-1;     //2D Array starts from zero (0=1)

        DB[A1][A1]=(float) def; DB[A1][A2]=(float) 1.0; DB[A1][A3]=(float) 1.5; DB[A1][A4]=(float) 2.2; DB[A1][A5]=(float) 1.3; DB[A1][A6]=(float) 2.2; DB[A1][A7]=(float) 0.9; DB[A1][A8]=(float) 1.0;
        DB[A2][A1]=(float) 1.6; DB[A2][A2]=(float) def; DB[A2][A3]=(float) 2.0; DB[A2][A4]=(float) 1.0; DB[A2][A5]=(float) 1.0; DB[A2][A6]=(float) 1.8; DB[A2][A7]=(float) 2.3; DB[A2][A8]=(float) 1.5;
        DB[A3][A1]=(float) 1.7; DB[A3][A2]=(float) 1.1; DB[A3][A3]=(float) def; DB[A3][A4]=(float) 1.6; DB[A3][A5]=(float) 2.2; DB[A3][A6]=(float) 1.7; DB[A3][A7]=(float) 1.3; DB[A3][A8]=(float) 0.9;
        DB[A4][A1]=(float) 2.1; DB[A4][A2]=(float) 1.3; DB[A4][A3]=(float) 1.5; DB[A4][A4]=(float) def; DB[A4][A5]=(float) 1.2; DB[A4][A6]=(float) 1.1; DB[A4][A7]=(float) 1.1; DB[A4][A8]=(float) 0.5;
        DB[A5][A1]=(float) 1.9; DB[A5][A2]=(float) 1.3; DB[A5][A3]=(float) 1.7; DB[A5][A4]=(float) 2.1; DB[A5][A5]=(float) def; DB[A5][A6]=(float) 1.2; DB[A5][A7]=(float) 1.4; DB[A5][A8]=(float) 1.6;
        DB[A6][A1]=(float) 1.8; DB[A6][A2]=(float) 1.5; DB[A6][A3]=(float) 1.6; DB[A6][A4]=(float) 0.9; DB[A6][A5]=(float) 1.0; DB[A6][A6]=(float) def; DB[A6][A7]=(float) 0.4; DB[A6][A8]=(float) 1.2;
        DB[A7][A1]=(float) 1.6; DB[A7][A2]=(float) 2.3; DB[A7][A3]=(float) 1.4; DB[A7][A4]=(float) 1.6; DB[A7][A5]=(float) 1.1; DB[A7][A6]=(float) 1.8; DB[A7][A7]=(float) def; DB[A7][A8]=(float) 1.4;
        DB[A8][A1]=(float) 2.2; DB[A8][A2]=(float) 1.0; DB[A8][A3]=(float) 1.8; DB[A8][A4]=(float) 1.7; DB[A8][A5]=(float) 0.9; DB[A8][A6]=(float) 0.9; DB[A8][A7]=(float) 1.7; DB[A8][A8]=(float) def;

        ratingDB();

    }

    public void ratingDB(){
        for(int i=0 ; i<DB.length ; i++){
            for(int j=0 ; j<DB.length ; j++){
                if(DB[i][j]>highRate){
                    DB[i][j]= highRate;
                }else if(DB[i][j]>normalRate){
                    DB[i][j]= normalRate;
                }else{
                    DB[i][j]= lowRate;
                }

            }
        }
    }

    public int getEstimatedRSSfromDB(int _P0,float d, int primeTxNum, int ancRxNum){
        return (int) Math.round(_P0 - 10 * (DB[ancRxNum][primeTxNum]) * Math.log10(d));
    }

}

