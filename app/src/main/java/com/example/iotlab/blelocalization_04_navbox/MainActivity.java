package com.example.iotlab.blelocalization_04_navbox;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.text.DecimalFormat;
import java.util.UUID;

import android.Manifest;
import android.app.ProgressDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothManager;
import android.bluetooth.le.BluetoothLeScanner;

import android.content.DialogInterface;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import android.os.Bundle;
import android.support.design.widget.FloatingActionButton;
import android.support.design.widget.Snackbar;
import android.support.v4.app.ActivityCompat;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.support.design.widget.NavigationView;
import android.support.v4.view.GravityCompat;
import android.support.v4.widget.DrawerLayout;
import android.support.v7.app.ActionBarDrawerToggle;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.Toolbar;
import android.view.Menu;
import android.view.MenuItem;

import android.app.AlertDialog;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.PorterDuff;
import android.graphics.Typeface;
import android.os.Handler;
import android.text.method.ScrollingMovementMethod;
import android.util.DisplayMetrics;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;
import android.widget.ToggleButton;

import com.estimote.sdk.connection.DeviceConnection;
import com.estimote.sdk.connection.DeviceConnectionCallback;
import com.estimote.sdk.connection.DeviceConnectionProvider;
import com.estimote.sdk.connection.exceptions.DeviceConnectionException;
import com.estimote.sdk.connection.scanner.ConfigurableDevice;
import com.estimote.sdk.connection.settings.SettingCallback;
import com.estimote.sdk.connection.settings.SettingsEditor;
import com.gun0912.tedpermission.TedPermission;
import com.gun0912.tedpermission.PermissionListener;

import uk.co.senab.photoview.PhotoViewAttacher;

public class MainActivity extends AppCompatActivity
        implements NavigationView.OnNavigationItemSelectedListener {


    private boolean debug1 = true;
    private boolean Debug_NodeFinder = false;   //노드개수 변동에 따른 토스트 알림 출력

    private boolean onErrTouch = false;         //true = Enable error calculation touch
                                                //false =

    //Text writing for Experiment
    private FileOutputStream fos = null;
    String dirPath = "/sdcard/RSS/";
    File file;
    private boolean firstTxtWrite = true;
    private boolean onText = false;
    private String txtName = "";
    private float txtStartTime = 0;

    //Time set (Warning : Depend on device performance)
    private static final long   SCAN_PERIOD     = 900000000;            //BLE scan enable time
    private static final float  LossCheckPeriod = (float) 3.0;       //BLE signal loss checking
    private static final float  PLOTPeriod      = (float) 1.00;       //UI plot period
    private static final float  MLEPeriod       = (float) 2.0;      //MLE period

    //MAP UI constant
    private float mapScale = 65;        //Defalut map scale for eng 11 floor
    private float mapWidth_x = 1, mapWidth_y = 1, mapWidth;


    //MAP UI Member
    private ImageView imgv_map;
    private Bitmap bitmap_map;
    private Bitmap bitmap_eng11; private final int bitmap_eng11_scale = 65;  //도트당1:65사이즈 지도
    private Canvas canvas_map;
    private PhotoViewAttacher mAttacher;
    private int screenWidth_pixel;
    DecimalFormat format = new DecimalFormat(".##");                                    //Decimal

    //Layout member
    private TextView tv_info;
    private ToggleButton tbtn_save,tbtn_En_scale;

    private Button btn_error_undo, btn_error_clearAndSave, btn_error_shot,btn_error_shot2, btn_MUupdate;

    //Location Tracking(BLElocalization08ver - 망함)
    private boolean onTrajectory = false;
    private ArrayList<Location2D> trajectory = new ArrayList<>();
    private Location2D trackingTemp = new Location2D(0,0);

    //ErrorEstimation (BLElocalization09ver)
    class_errorDB errorDBs = new class_errorDB(new Location2D(0,0));

    //Location member (위치좌표 데이터 변수 선언)
    private Location2D PDRLocation2D = new Location2D(0,0);         //pdr로 판단한 로케이션 좌표
    private Location2D RSSLocation2D = new Location2D(0,0);         //RSS로 판단한 로케이션 좌표
    private Location2D currentLocation2D = new Location2D(0,0);     //현재 로케이션 좌표
    private Location2D currentLocation2D_imag = new Location2D(0,0);     //현재 로케이션 좌표
    private Location2D priorRSSLocation2D = new Location2D(0,0);    //이전 로케이션 좌표
    private Location2D ProposedLocation2D = new Location2D(0,0);         //랜드마크영향을 받은 현재 로케이션 좌표
    //private Location2D BScurrentLocation2D = new Location2D(0,0);

    /*---------------------PART.1 - PDR  -----------------------------------*/

    //Control setting
    private boolean runPDR = false;

    //PDR constant 1
    private static final int   SENSOR_INIT_TIME = 100;               // Sensor Warm-up time in ms
    private static final float MIN_MAX_INTERVAL = (float) 0.05;     // Minimum Intervals in Step Detection
    private static final float STEP_INTERVAL = (float) 0.1;
    private static final float MAX_PEAK = (float) 2;                // Acc. Norm Threshold in Step Detection
    private static final float MIN_PEAK = (float) 1.8;
    private static float       THETA_ABS_DEG = 0;                         // The angle between Mag North and x-axis (CCW)
    private static final float THETA_ABS_RAD = (float) (THETA_ABS_DEG * (Math.PI / 180));
    private static final float W_PREV = 2, W_MAG = 1, W_GYRO = 2;   // Heading Estimation Parameters
    private static final float TH_COR = (float) (5 * Math.PI / 180);
    private static final float TH_MAG = (float) (2 * Math.PI / 180); // in radian
    private static final float STEP_CONSTANT = (float) 0.45;

    //PDR constant 2
    private float angle_prev, angle_mag, angle_mag_prev, angle_gyro, angle_cur;
    private float delta_cor, delta_mag;

    private float acc_p;            // Prior ACC
    private float t_max_peak;       // previous step time
    private float t_step_s;

    private boolean isStep;

    private float azimuth;          // Azimuth
    private float pitch;
    private float roll;

    private float acc_max, acc_min;
    private float steplength;

    private int n_step;             // Step count
    private int n_phase ;
    private int cycle_phase=0;
    private static float n_compensation = 0;

    //PDR member declaration
    private SensorManager mSensorManager;
    private SensorEventListener mSensorEventListener = new SensorEventListener() {
        @Override
        public void onAccuracyChanged(Sensor sensor, int accuracy) {
        }

        @Override
        public void onSensorChanged(SensorEvent event) {

            switch (event.sensor.getType()) {
                case Sensor.TYPE_ROTATION_VECTOR:
                    if (System.currentTimeMillis() - t_begin > SENSOR_INIT_TIME) {
                        rot_vector = event.values.clone();
                        getOrientation();
                    }
                    break;
                case Sensor.TYPE_LINEAR_ACCELERATION:
                    data_acc = event.values.clone();
                    updateLocation();   //가속센서가 작동함과 함께 위치정보 업데이트 실시
                    break;
                case Sensor.TYPE_GYROSCOPE:
                    data_gyro = event.values.clone();
                    break;
            }
        }
    };



    private Sensor mSensorLinAcc, mSensorGyro, mSensorRot; // IMU

    private boolean isInitialized, isAngleCalibrated;
    private boolean isRun = false;
    private boolean isLOS = false;

    private long t_begin;
    private float t_current_s, dt, t_lastplot;
    private float t_lastMLE;
    private float t_prior_s;

    private float[] data_acc = null;
    private float acc_norm;
    private float[] data_gyro = null;
    private float[] rot_vector = null;


    //PDR Initial orientation setting
    private void getOrientation() {
        /* rot_vector 사용하는 경우 */
        float[] rotMat = new float[9];
        float[] orientation = new float[3];

        SensorManager.getRotationMatrixFromVector(rotMat, rot_vector);
        SensorManager.getOrientation(rotMat, orientation);

        if(!isAngleCalibrated){
            THETA_ABS_DEG = orientation[0];
            isAngleCalibrated = true;
        }

        azimuth = (-1) * orientation[0]; // CW에서 CCW로 변환
        pitch = orientation[1];
        roll = orientation[2];
        // in radian
        // target orientation angle in PDR coordinate system.

        t_prior_s = (System.currentTimeMillis() - t_begin) / 1000;
        angle_cur = (azimuth + THETA_ABS_RAD);
        angle_prev = angle_cur;
        angle_mag = (azimuth + THETA_ABS_RAD);
        angle_mag_prev = angle_mag;

        // target orientation in absolute coordinate system
    }
    //End of folding for part 1 PDR

    /*---------------------PART2. Beacon Hardware parameters ---------------*/
    //private int[] availableAnchorset = {19,7,11,13,12,15,16,22,0};
    //private int[] availableAnchorset = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43}; //for 8Node
    //private int[] availableAnchorset = {24,25,27,33,30,1}; //for 8Node
    //private int[] availableAnchorset = {44,45,46,47,48,49,50,51,52,53,25,27,33}; //for 8Node
    //private int[] availableAnchorset = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43}; //for 8Node
    private int[] availableAnchorset = {54,55,56,57,58};


    private static int P0 = -75;      //final is constant
    private static float MU = (float) 1.7;

    private static BleDatabase[] BLE_Set = {                        //default is final
            new BleDatabase(0, 0, P0, MU, "IOTLAB",0),     //Not in use
            new BleDatabase(36, 64, P0, MU, "FF:A0:AC:A1:3F:DC",1),     //50121855    4c444947    0dbm  0.3msec   1
            new BleDatabase(42, 64, P0, MU, "C4:53:23:F1:DA:CD",2),     //50121856    7d7300a1    0dbm  0.3msec   2
            new BleDatabase(42, 58, P0, MU, "C4:4F:C0:F4:E5:67",3),     //50121857    e01f57ff    0dbm  0.3msec   3
            new BleDatabase(36, 58, P0, MU, "FC:85:E3:9A:36:4A",4),     //50113612    a3c7458c    0dbm  0.3msec   4
            new BleDatabase(39, 61, P0, MU, "ED:47:EE:0C:25:8A",5),     //50113613    83568079    0dbm  0.3msec   5
            new BleDatabase(14,18.5, P0, MU, "F5:60:13:02:CB:7E",6),      //50113608  353eff33    0dbm  0.3msec   6   //고장 > 19

            new BleDatabase(20, 18.5, P0, MU, "EC:72:C6:26:98:44",7),   //50113611  f25ae9ce    4dbm  0.1msec   7
            new BleDatabase(4, 13, P0, MU, "C9:26:A5:99:C6:5F",8),     //50113605  a282ecb0    0dbm  0.3msec   8
            new BleDatabase(16, 16, P0, MU, "DA:BF:DB:34:40:5E",9),     //50113610  d516dd21    0dbm  0.1msec   9
            new BleDatabase(21,16, P0, MU, "CC:81:EE:73:2D:76",10),        //50113609  f7366f83    0dbm  0.3msec   10  //고장
            new BleDatabase(11.5,12.5, P0, MU, "EA:7D:62:29:3B:EF",11),    //50113607  479d292d    0dbm  0.3msec   11
            new BleDatabase(11.5, 6, P0, MU, "DA:E1:F9:48:E3:24",12),        //50113604  be95a94a    4dbm  0.1msec   12

            new BleDatabase(20.5, 12.5, P0, MU, "FC:E9:E6:04:0E:B0",13),   //50113606  585e47c1    0dbm  0.3msec   13
            new BleDatabase(7, 11, P0, MU, "FD:B0:AC:E6:F2:E6",14),    //50112961  46d802bd    0dbm  0.3msec   14
            new BleDatabase(20.5, 6, P0, MU, "F6:CE:67:71:34:6F",15),        //50112969  98ae96c6    4// dbm  0.1msec   15
            new BleDatabase(16, 16, P0, MU, "C1:02:6C:D7:58:FA",16),       //50112962  9d98ec53    0dbm  0.3msec   16
            new BleDatabase(10.5,9, P0, MU, "F0:73:A2:62:BD:B5",17),      //50112968  2cffc41d    0dbm  0.3msec   17
            new BleDatabase(16, 8, P0, MU, "F2:39:B0:FA:58:9F",18),        //50112970  08734880    0dbm  0.3msec   18  //고장

            new BleDatabase(11.5, 18.5, P0, MU, "E6:15:22:54:EE:75",19),     //50112963  8dce3bc5    4dbm  0.1msec   19
            new BleDatabase(16, 8, P0, MU, "E5:33:3D:38:73:C2",20),     //50112967  bdde121d    0dbm  0.3msec   20     //고장인데 왜 여기껀 안들어오지? >> 22
            new BleDatabase(21,16, P0, MU, "EF:B0:1F:D0:2E:04",21),     //50112966  f191cfca    0dbm  0.3msec   21
            new BleDatabase(16, 9, P0, MU, "C1:C3:64:2B:3C:D0",22),     //50112964  66fd2285    0dbm  0.3msec   22
            new BleDatabase(4, 9, P0, MU, "C4:2D:F6:39:FA:58",23),     //50112965  17fab35c    0dbm  0.3msec   23
            new BleDatabase(24, 24, P0, MU, "DB:69:EE:72:18:EC",24),     //50109455	  e833d1e6 24
            new BleDatabase(37.5, 65.5, P0, MU, "F0:76:FE:A4:7C:50",25),     //50109454    6f082278
            new BleDatabase(25, 25, P0, MU, "F0:76:FE:A4:7C:50",26),     //50109453	6ef794f0
            new BleDatabase(40.5,65.5, P0, MU, "C3:44:05:B7:5E:23",27),     //50109452	e0278c60
            new BleDatabase(25, 25, P0, MU, "CE:52:8C:D8:05:85",28),     //50109451 b60bf8c7
            new BleDatabase(25, 25, P0, MU, "EC:33:97:8E:AD:26",29),     // 50109456	 1e1e4860
            new BleDatabase(25, 25, P0, MU, "C0:D7:40:E2:D8:94",30),     // 50109457	a0765dac
            new BleDatabase(25, 25, P0, MU, "DE:D6:94:39:29:8C",31),     // 50109458  8f4412ac  //
            new BleDatabase(25, 25, P0, MU, "E6:FB:E1:97:A3:48",32),     // 50109459  6fc13851
            new BleDatabase(40.5, 68.5, P0, MU, "D1:9D:B3:EE:BC:38",33),     // 50109460  441bc639
            new BleDatabase(25, 25, P0, MU, "CA:E4:F4:22:C0:45",34),     // 50109461  c7a9b493
            new BleDatabase(25, 25, P0, MU, "F1:33:93:FC:6F:5A",35),      // 50109462  0c4788cf
            new BleDatabase(25, 25, P0, MU, "FB:B5:86:EA:E0:64",36),    // 50109464  47e06851	  허재기준 37번
            new BleDatabase(25, 25, P0, MU, "E7:E8:8C:3D:54:E0",37),    //  50109463 18785a76  허재기준 36번
            new BleDatabase(25, 25, P0, MU, "CB:78:59:7B:5A:91",38),     //  50109465  d56ff0db
            new BleDatabase(25, 25, P0, MU, "CE:D5:3F:CA:5E:15",39),     // 50109466  014cfd56
            new BleDatabase(25, 25, P0, MU, "F0:85:6E:D8:C2:9B",40),     // 50109467  bf32fd5d
            new BleDatabase(25, 25, P0, MU, "F5:D9:AC:F4:AD:AF",41),     //50109468	  c4bed79e
            new BleDatabase(25, 25, P0, MU, "F1:D2:70:27:33:24",42),     //50109469  c9e6b60e
            new BleDatabase(25, 25, P0, MU, "C7:8F:D9:1D:89:4B",43),     //50109470  aad63521
            new BleDatabase(36, 70, P0, MU, "FC:2B:65:D0:50:01",44),     //50112234 c4d1e584 4 dbm 100ms
            new BleDatabase(39, 70, P0, MU, "FB:EA:65:66:35:F4",45),     //50112233 84524787 4 dbm 100ms
            new BleDatabase(42, 70, P0, MU, "DE:D1:34:07:17:53",46),     //50112235  19099254 4 dbm 100ms
            new BleDatabase(36, 67, P0, MU, "F2:A2:E8:25:05:B6",47),     //50112236  f36f6dd5 4 dbm 100ms
            new BleDatabase(39, 67, P0, MU, "EE:17:7E:F4:2A:F5",48),     //50112237  e9fec773 4 dbm 100ms
            new BleDatabase(42, 67, P0, MU, "D3:72:95:78:89:EE",49),     //50112238  363d97ff 4 dbm 100ms
            new BleDatabase(36, 64, P0, MU, "E3:7F:5C:2A:58:AF",50),     //50112239  4a3adcce  4 dbm 100ms
            new BleDatabase(39, 64, P0, MU, "CC:F9:A3:6F:0A:0F",51),     //50112240  75e2b21b  4 dbm 100ms
            new BleDatabase(42, 64, P0, MU, "F4:DD:1C:C0:3C:51",52),     //50112241 2cdf203d 4 dbm 100ms
            new BleDatabase(37.5, 68.5, P0, MU, "ED:5E:75:E1:4E:AA",53),     //50112242  6a4bb561  4 dbm 100ms

            //Estimote devices
            new BleDatabase(36, 64, P0, MU, "DC:C1:FA:5E:83:9D",54,"6303ef469d8043427bb493ea615f5e0a"),     // Beetroot A (DE:C3:FC:60:85:9F)
            new BleDatabase(42, 64, P0, MU, "D6:38:97:14:31:B3",55,"e9bbcd90fd490084294b02281ad69338"),     // Beetroot B (D8:BA:90:58:EE:BA)
            new BleDatabase(42, 58, P0, MU, "FB:7F:0D:0C:B3:15",56,"45f22f3997f59918eefb9efe573b421a"),     // Lemon C
            new BleDatabase(36, 58, P0, MU, "CB:36:0F:6E:7D:24",57,"6c0f7747687a0e970766b71a27550e12"),     // Candy E
            new BleDatabase(39, 61, P0, MU, "D5:B2:8C:28:22:2B",58,"abbad822c263d421f7a2feb17cb99b35")      // Candy G         //

    };

    // BLE members
    private BluetoothAdapter mBluetoothAdapter;
    private Handler mHandler;
    private BluetoothLeScanner mBLEScanner; //for above API 21

    //Data Base
    private HashMap<Integer, BleDatabase> anchors;   //DB for anchor nodes (Key,Address Code)
    private class_muDB muDBs;                              //DB for MU

    //Loss signal checks
    private float t_lastLossCheck = 0;


    /*------------------------ PART3-1. MLE --------------------------*/
    // 로케이션 좌표는 위에 나와있음 아래는 참고만 할것
    //private Location2D RSSLocation2D = new Location2D(0,0);           //RSS로 판단한 로케이션 좌표
    //private Location2D currentLocation2D = new Location2D(0,0);       //현재 로케이션 좌표
    //private Location2D priorRSSLocation2D = new Location2D(0,0);      //이전 로케이션 좌표

    private boolean isKF = false;        //KF사용 유무 설정
    private float RESOLUTION = (float) 1.0;
    private int RSSbuffer = 7;          //Buffersize설정

    private boolean MLE_success_flag = false;
    private boolean runMLE = false;

    //아래 변수는 ChangeMap에 의해 변화될 수 있음.
    private float map_min_x = 0,map_min_y = 0, map_max_x = 65, map_max_y = 65;

    private float min_x = 30 ;
    private float max_x = 50 ;
    private float min_y = 50 ;
    private float max_y = 65 ;

    //private

    //계산을 위한 변수
    private float rmse;

    /*------------------------ PART3-2. NLOS --------------------------*/
    private boolean onNLOS = false;
    private boolean onLM_NLOS_BLOCKING = true;
    private boolean isNLOS = false;
    private final float NLOSErrorDistanceThreshold = (float) 1.5;  //No NLOS calculate = 1, 1st Iteration
    ArrayList<Location2D> firstSubLocation = new ArrayList<>();
    Location2D correctedLocation = new Location2D(0,0);

    private int iterationDepth = 0;                                 //
    private final int iterationDepthThreshold = 1;                  //Warning : Iteration Threshold
    ArrayList<Integer> eliminatedNodeArray = new ArrayList<>();

    /*------------------------ PART3-3. Landmarks --------------------------*/      //지금 와서는 랜드마크보다는 현재 실험점으로서 가치가 더 높음.
    //private Location2D LMcurrentLocation2D = new Location2D(0,0);     //랜드마크영향을 받은 현재 로케이션 좌표

    //for common path loss location number experiment
    HashMap<Integer, BleDatabase> redRankHash = new HashMap<>();
    HashMap<Integer, BleDatabase> blueRankHash= new HashMap<>();
    private Location2D redRankLocation   = new Location2D(0,0);      //랜드마크에 쓰일 로케이션 좌표
    private Location2D blueRankLocation  = new Location2D(0,0);     //랜드마크에 쓰일 로케이션좌표
    private int redRankNum               = 3;
    private int blueRankNum              = 5;
    private boolean isred                = false;
    private boolean isblue                = false;

    ArrayList<BleDatabase> rankedRefData= new ArrayList<>();

    private boolean     onSpot = true;              //Current anchor node number (From Hashmap)
    private boolean     isSpot = false;
    private boolean     onRanking = false;          //plot the Ranked numbers above the node. (Red and Blue)

    private final float SpotRANGE = (float) 3;        //Spot RANGE
    private final int   SpotRSSTHRESHOLD = -90;         //Shade Threshold
    private int         Spotnode          = 0;          //Current anchor node number (From Hashmap)
    //private int         LM_KF_size      = 4;            //Only for experiment. RSS_KF size for LM detection (High-Stable but reconition is low)


    /*------------------------ PART3-4. Body Shading --------------------------*/   //Not in use
    private boolean onBodyShadingNLOS = false;                          //Not in use (DO NOT USE IT)

    //private Location2D BScurrentLocation2D = new Location2D(0,0)
    private boolean isBodyShadingMLESuccessFlag = false;
    private final float bodyShadingDistanceThreshold = (float) 3.0;     //(BodyShading disThreshold)
    private final float bodyShadingAngleThreshold = (float) 2.1;         //(BodyShading absolute angle threshold)

    /*------------------------ PART4. ESTIMOTE Pairing-----------------------*/

    //Master Setting
    private final boolean onMUupdate = true;
    private final boolean onlyMucalculation = true;

    //Library
    private estimoteManager UlpEstimoteManager;

    //Pairing member
    private ConfigurableDevice configurableDevice;
    private DeviceConnection connection;
    private DeviceConnectionProvider connectionProvider;
    private ProgressDialog progressDialog;

    //MU noise guard //실제로 MU측정에도 노이즈가 끼기 때문에 한계치 설정해주기 위함
    private boolean muNoiseGuard = true;         //노이즈를 고려여부
    private double muNoiseGuardValue = 0.2;      //노이즈 고려 한계치

    //Current path loss exponent
    private HashMap<Integer, Double> currentMuTable = new HashMap<>();
    private HashMap<Integer, HashMap<Integer, Double>> totalTable = new HashMap<>();


    /*----------------------------------------------------------------------*/
    /*----------------------------------------------------------------------*/
    /*----------------------------------------------------------------------*/

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        Toolbar toolbar = (Toolbar) findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);

        //Permission
        ActivityCompat.requestPermissions(this,new String[]{Manifest.permission.BLUETOOTH},1);

        new TedPermission(this)
                .setPermissionListener(permissionlistener)
                .setDeniedMessage("If you reject permission,you can not use this service\n\nPlease turn on permissions at [Setting] > [Permission]")
                //.setPermissions(Manifest.permission.READ_EXTERNAL_STORAGE, Manifest.permission.WRITE_EXTERNAL_STORAGE, Manifest.permission.ACCESS_COARSE_LOCATION)
                .setPermissions(Manifest.permission.ACCESS_COARSE_LOCATION)
                .check();


        FloatingActionButton fab = (FloatingActionButton) findViewById(R.id.fab);
        fab.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Snackbar.make(view, "Replace with your own action", Snackbar.LENGTH_LONG)
                        .setAction("Action", null).show();
            }
        });

        DrawerLayout drawer = (DrawerLayout) findViewById(R.id.drawer_layout);
        ActionBarDrawerToggle toggle = new ActionBarDrawerToggle(
                this, drawer, toolbar, R.string.navigation_drawer_open, R.string.navigation_drawer_close);
        drawer.setDrawerListener(toggle);
        toggle.syncState();

        NavigationView navigationView = (NavigationView) findViewById(R.id.nav_view);
        navigationView.setNavigationItemSelectedListener(this);

        //Estimote beacon library
        UlpEstimoteManager = new estimoteManager(this,this);


        //UI layout decalration
        tv_info = (TextView) findViewById(R.id.tv_info);

        tv_info.setMovementMethod(new ScrollingMovementMethod()); //Enable Scrolling

        //Buttons and Toggle buttons
        tbtn_save = (ToggleButton) findViewById(R.id.tbtn_save);
        tbtn_save.setEnabled(false);
        tbtn_save.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if (tbtn_save.isChecked()) {
                    onText = true;
                } else {
                    textWriteEnd();
                    onText = false;
                }
            }
        });

        tbtn_En_scale = (ToggleButton) findViewById(R.id.tbtn_En_scale);
        tbtn_En_scale.setEnabled(false);
        tbtn_En_scale.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if (tbtn_En_scale.isChecked()) {
                    onErrTouch=true;
                    enableOnTouch();
                } else {
                    onErrTouch=false;
                    disableOnTouch();
                }
            }
        });



        btn_error_shot = (Button) findViewById(R.id.btn_error_shot);
        btn_error_shot.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View arg0) {
                //Operator
                final Location2D temp_for_click = new Location2D(currentLocation2D_imag.getX(),currentLocation2D_imag.getY());

                errorDBs.addErrorDB(temp_for_click);

                //Debug
                Toast.makeText(getApplicationContext(),"Shot : "+errorDBs.targetArray.size()+" DB," + format.format(errorDBs.getLastErrorfromDB())+"m Error", Toast.LENGTH_SHORT).show();

                //Delay
                btn_error_shot.setEnabled(false);
                new Handler().postDelayed(new Runnable() {
                    @Override
                    public void run() {
                        btn_error_shot.setEnabled(true);
                    }
                },1000);
            }
        });

        btn_error_shot2 = (Button) findViewById(R.id.btn_error_shot2);
        btn_error_shot2.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View arg0) {
                //Operator
                final Location2D temp_for_click2 = new Location2D(ProposedLocation2D.getX(), ProposedLocation2D.getY());

                if(isSpot){
                    errorDBs.addErrorDB(temp_for_click2);
                    //Debug
                    Toast.makeText(getApplicationContext(),"Shot2 : "+errorDBs.targetArray.size()+" DB," + format.format(errorDBs.getLastErrorfromDB())+"m Error", Toast.LENGTH_SHORT).show();
                }else{
                    Toast.makeText(getApplicationContext(),"Not LM state", Toast.LENGTH_SHORT).show();
                }

                //Delay
                btn_error_shot.setEnabled(false);
                new Handler().postDelayed(new Runnable() {
                    @Override
                    public void run() {
                        btn_error_shot.setEnabled(true);
                    }
                },300);
            }
        });


        btn_error_clearAndSave = (Button) findViewById(R.id.btn_error_clearAndSave);
        btn_error_clearAndSave.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View arg0) {
                //Operator
                String _errTxtName = "Esterr"+System.currentTimeMillis();

                textSaveErrorDB("Esterr"+_errTxtName);
                errorDBs.clearDB();

                //Debug
                Toast.makeText(getApplicationContext(),"Clear & Save : _errTxtName", Toast.LENGTH_SHORT).show();

                //Delay
                btn_error_shot.setEnabled(false);
                new Handler().postDelayed(new Runnable() {
                    @Override
                    public void run() {
                        btn_error_shot.setEnabled(true);
                    }
                },1000);
            }
        });

        btn_error_undo = (Button) findViewById(R.id.btn_error_undo);
        btn_error_undo.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View arg0) {
                //Operator

                errorDBs.undoErrorDB();

                //Debug
                Toast.makeText(getApplicationContext(),"Undo : "+ errorDBs.targetArray.size()+" DB availiable", Toast.LENGTH_SHORT).show();

                //Delay
                btn_error_undo.setEnabled(false);
                new Handler().postDelayed(new Runnable() {
                    @Override
                    public void run() {
                        btn_error_undo.setEnabled(true);
                    }
                },1000);
            }
        });

        //강제로 Reference spot에서 MU를 측정하는 버튼 (디버깅용도)
        btn_MUupdate = (Button) findViewById(R.id.btn_MUupdate);
        btn_MUupdate.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View arg0) {
                //Operator
                if(isSpot){   //Curernt state is in LM
                    if(Spotnode != 0){    //LM node found
                        Toast.makeText(getApplicationContext(), "Calling the  "+anchors.get(Spotnode).getAdress(), Toast.LENGTH_SHORT).show();
                        if(UlpEstimoteManager.isReferenceSpotInDB(anchors.get(Spotnode).getID())){
                            Toast.makeText(getApplicationContext(), "Start to connection  "+anchors.get(Spotnode).getAdress(), Toast.LENGTH_SHORT).show();
                            muUpdate(UlpEstimoteManager.getSpotDevice().device);
                            //
                            //Toast.makeText(getApplicationContext(), "MAC is "+UlpEstimoteManager.getAnyMAC(), Toast.LENGTH_SHORT).show();
                            //Toast.makeText(getApplicationContext(), "Target is "+UlpEstimoteManager.getCurrentSpotMAC(), Toast.LENGTH_SHORT).show();
                            //muUpdate(UlpEstimoteManager.findReferenceSpot().device);

                        }else{
                            Toast.makeText(getApplicationContext(), "Wrong LMnode number", Toast.LENGTH_SHORT).show();
                            //String tempString = UlpEstimoteManager.getAnyID();
                            //Toast.makeText(getApplicationContext(), "MAC is "+UlpEstimoteManager.getAnyMAC(), Toast.LENGTH_SHORT).show();

                            //for(BleDatabase i : BLE_Set){
//                                UlpEstimoteManager.checkingID(i.getID());
//                            }
                        }
                    }else{

                    }
                }else{  //Current state is not in LM
                    Toast.makeText(getApplicationContext(), "No reference spot nearby", Toast.LENGTH_SHORT).show();
                }

                //Delay
                btn_error_undo.setEnabled(false);
                new Handler().postDelayed(new Runnable() {
                    @Override
                    public void run() {
                        btn_error_undo.setEnabled(true);
                    }
                },1000);
            }
        });



        //Screen calculate -- for layout length setting
        DisplayMetrics displaymetrics = new DisplayMetrics();
        getWindowManager().getDefaultDisplay().getMetrics(displaymetrics);
        screenWidth_pixel = displaymetrics.widthPixels;

        //Image setting
        imgv_map = (ImageView) findViewById(R.id.map);
        bitmap_map = Bitmap.createBitmap(screenWidth_pixel, screenWidth_pixel, Bitmap.Config.ARGB_8888);

        BitmapFactory.Options options = new BitmapFactory.Options();
        options.inSampleSize = 2;
        bitmap_eng11 = BitmapFactory.decodeResource(getResources(), R.drawable.newengineer11fall65scale, options);
        //bitmap_eng11 = BitmapFactory.decodeResource(getResources(), R.drawable.newengineer11fall65scale, options);

        //bitmap_map = BitmapFactory.decodeResource(this.getApplicationContext().getResources(), R.drawable.map).copy(Bitmap.Config.ARGB_8888, true);
        canvas_map = new Canvas(bitmap_map);
        imgv_map.setImageBitmap(bitmap_map);

        mAttacher = new PhotoViewAttacher(imgv_map);    //ADD ZOOM
        mAttacher.setMaximumScale(10);                  //ZOOM scale

        //PDR manager declaration
        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        mSensorLinAcc = mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        mSensorGyro = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        mSensorRot = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);



        //BLE setting
        anchors = new HashMap<>();  //turn on DB set

        //BLE mu DBs
        muDBs= new class_muDB(BLE_Set.length,MU);
        muDBs.setManualDB();
        muDBs.ratingDB();

        //BLE Scanner Thread
        mHandler = new Handler();

        //(Essential)
        //Get BlueTooth adapter.    (21이상에선 Final 지움)
        BluetoothManager bluetoothManager =
                (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
        mBluetoothAdapter = bluetoothManager.getAdapter();
        //Bluetooth auto setting for permission
        if (mBluetoothAdapter == null || !mBluetoothAdapter.isEnabled()) {
            Toast.makeText(getApplicationContext(), "ERROR : BlueTooth disabled.", Toast.LENGTH_SHORT).show();
        }else{
            Toast.makeText(getApplicationContext(), "BlueTooth enabled.", Toast.LENGTH_SHORT).show();
        }

        scanLeDevice(); //The handler obtains RSS values

        changeMap_1FloorLoby();
        //Map setting
        switch(11){
            case 1 :{
                changeMap_1FloorLoby();
            }

            case 11 :{
                changeMap_11FloorAll();
            }
        }
        resume();
    }

    //Permission
    PermissionListener permissionlistener = new PermissionListener() {
        @Override
        public void onPermissionGranted() {
            Toast.makeText(MainActivity.this, "Permission Granted", Toast.LENGTH_SHORT).show();
        }

        @Override
        public void onPermissionDenied(ArrayList<String> deniedPermissions) {
            Toast.makeText(MainActivity.this, "Permission Denied\n" + deniedPermissions.toString(), Toast.LENGTH_SHORT).show();
        }

    };

    //Enable Touch on
    private void enableOnTouch(){
        if(onErrTouch) {
            imgv_map.setOnTouchListener(new View.OnTouchListener() {

                @Override
                public boolean onTouch(View v, MotionEvent event) {

                    switch (event.getAction()) {
                        case MotionEvent.ACTION_DOWN:
                        case MotionEvent.ACTION_MOVE:
                        case MotionEvent.ACTION_UP:
                            // event.getX();    event.getY();
                            errorDBs.setMarker(new Location2D(reverseConvert(event.getX()),
                                    reverseConvert(event.getY()))
                            );
                    }
                    return true;
                }
            });
        }

    }

    private void disableOnTouch(){
        imgv_map.setOnTouchListener(new View.OnTouchListener() {

            @Override
            public boolean onTouch(View v, MotionEvent event) {

                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                    case MotionEvent.ACTION_MOVE:
                    case MotionEvent.ACTION_UP:
                        // event.getX();    event.getY();
                        errorDBs.setMarker(new Location2D(reverseConvert(event.getX()),
                                reverseConvert(event.getY()))
                        );
                }
                return false;
            }
        });

    }

    private void pause(){
        runMLE = false;
        runPDR = false;
        mSensorManager.unregisterListener(mSensorEventListener);
        isRun = false;
        tbtn_save.setEnabled(false);
        tbtn_En_scale.setEnabled(false);

        // 20000us (20ms) sampling time.


        //clearMap();
    }

    private void resume(){
        runMLE = true;
        tbtn_save.setChecked(false);
        tbtn_save.setEnabled(true);

        tbtn_En_scale.setChecked(false);
        tbtn_En_scale.setEnabled(true);

        mSensorManager.registerListener(mSensorEventListener, mSensorGyro, 20000);
        mSensorManager.registerListener(mSensorEventListener, mSensorLinAcc, 20000);
        mSensorManager.registerListener(mSensorEventListener, mSensorRot, 20000);
        init();

    }

    //Main process for update location
    //This part Attached
    private void updateLocation() {
        //reset map
        tv_info.setText("Phase : " + cycle_phase + "\n");
        cycle_phase++;

        t_current_s = (float) (System.currentTimeMillis() - t_begin) / (float) 1000;

        // 1. PDR
        try {//정상적으로 작동하였을 경우
            if(runPDR){
                if (data_acc != null && data_gyro != null) {
                    updateLocation_PDR();
                }
            }
        } catch (Exception e){
            tv_info.append("ERROR in updateLocation : "+ e.toString());
        }

        //2. BLE-RSS localization with NLOS mitigation (if NLOSon == true)
        if((t_current_s- t_lastMLE)>= MLEPeriod){
            if(runMLE){
                isLOS = updateLocation_RSS();       //Caclulate current location using MLE
            }
            t_lastMLE = t_current_s;
        }else{
            isLOS = false;
        }

        currentLocation2D.setLocation(RSSLocation2D);
        currentLocation2D.setRMSE(RSSLocation2D.getRMSE());


        //3. plot
        if((t_current_s- t_lastplot)>= PLOTPeriod) {
            clearMap();
            plotCurrentLocation();
            t_lastplot = t_current_s;
        }

        //4. TV view
        //tv_info.append("Average Error : "+format.format(errorDBs.getAvgErrorResult())+", Depth : "+iterationDepth+"  ");
        //tv_info.append("RMSE:"+(int)currentLocation2D.getRMSE()+"\n");
        tv_info.append("Spot : "+Spotnode+" "+"("+ isSpot +")"+" Availiable"+UlpEstimoteManager.getEstimoteBeacons()+"\n");
        tv_info.append("NoiseGuard("+ muNoiseGuard +")"+getMuTableResult(currentMuTable)+"\n");

        for (int i : anchors.keySet()) {

            String _delay = format.format(anchors.get(i).getRXperiod());
            float ancazimuth = anchors.get(i).getAnchorAzimuthResult();
            //tv_info.append("Beacon: " + i + ", KF-RSS: " + anchors.get(i).get_KF_RSS() +"Delay"+_delay+"\n");
            tv_info.append("Beacon: " + i + ", KF-RSS: " + anchors.get(i).get_KF_RSS() +"ancAz"+ancazimuth+"\n");
        }

        //5. BLE signal Loss checks
        if((t_current_s- t_lastLossCheck)>= LossCheckPeriod){
            checkBeaconAvailable(LossCheckPeriod, t_current_s);
            t_lastLossCheck = t_current_s;
        }
    }

    //제곧내 - 이거 잘못빼면
    private void checkBeaconAvailable(float _LossCheckPeriod, float _t_current_s){

        HashMap<Integer, BleDatabase> anchorsClone = (HashMap<Integer, BleDatabase>) anchors.clone();
        //여기서 Clone을 하는 이유는 위의 _anchors를 그대로 사용할 경우 충돌때문에 고장나기 때문.

        for (int i : anchorsClone.keySet()) {
            if(anchorsClone.get(i).getDelay(_t_current_s) >= _LossCheckPeriod){
                anchors.remove(i);
                //Toast.makeText(getApplicationContext(),""+ i + " Beacon Lost, Current : "+ anchors.size() +" DB rest", Toast.LENGTH_SHORT).show();
            }
        }
    }


    private boolean updateLocation_RSS() {
        //Initialization
        iterationDepth = 0;
        eliminatedNodeArray = new ArrayList<>();

        //Flag checks
        if (n_compensation >= 3) return false;
        if (n_compensation == 0) priorRSSLocation2D.setLocation(currentLocation2D);
        if (anchors.size() < 4) return false; // not enough anchor nodes;
        MLE_success_flag = false;

        //2-1.Proposed algorithm
        isred = false;
        isblue = false;
        if(onSpot){

            isSpot = ProposedLSEstimation(ProposedLocation2D, anchors, SpotRANGE);
            //isred = LSEstimation(redRankLocation, redRankHash, false);    //Only for experiment.
            //isblue = LSEstimation(blueRankLocation, blueRankHash, false) ;//Only for experiment.
        }else{
            isSpot = false;}

        //2-#. Origianal Estimation
        return LSEstimation(RSSLocation2D, anchors,true);
    }

    //UI
    private void clearMap(){
        canvas_map.drawColor(Color.TRANSPARENT, PorterDuff.Mode.CLEAR);
        imgv_map.invalidate();
    }

    //PDR
    private void init() {
        PDRLocation2D.resetLocation();
        currentLocation2D.setLocation((float) 1.5, (float) 1.5);

        angle_prev = 0;
        angle_mag = 0;
        angle_gyro = 0;
        angle_cur = 0;
        angle_mag_prev = 0;

        mapWidth_x = 50;
        mapWidth_y = 50;

        n_step = -1;
        acc_p = -10;
        t_prior_s = 0;
        t_lastMLE = 0;
        t_max_peak = 0;
        t_step_s = 0;

        isInitialized = false;
        isAngleCalibrated = false;
        isStep = false;
        acc_max = 0;
        acc_min = 0;
        steplength = 0;

        t_begin = System.currentTimeMillis();
    }

    //PDR 작동부 상세 내용
    private void updateLocation_PDR() {
        /*----- Heading Estimation -----*/
        dt = t_current_s - t_prior_s;
        t_prior_s = t_current_s;

        angle_gyro = (angle_prev + (float) (dt * Math.round(data_gyro[2] * 100) / 100));
        // noise reduced by rounding up
        angle_mag_prev = angle_mag;
        angle_mag = (THETA_ABS_RAD + azimuth);
        // all angles in radian

        delta_mag = (float) (Math.PI - (Math.abs((angle_mag - angle_mag_prev) - Math.PI)));
        delta_cor = (float) (Math.PI - (Math.abs((angle_mag - angle_gyro) - Math.PI)));

        if (delta_cor <= TH_COR && delta_mag <= TH_MAG)
            angle_cur = ((W_PREV * angle_prev) + (W_MAG * angle_mag) + (W_GYRO * angle_mag)) / (W_PREV + W_MAG + W_GYRO);
        else if (delta_cor <= TH_COR && delta_mag > TH_MAG)
            angle_cur = ((W_MAG * angle_mag) + (W_GYRO * angle_mag)) / (W_MAG + W_GYRO);
        else if (delta_cor > TH_COR && delta_mag <= TH_MAG)
            angle_cur = angle_prev;
        else if (delta_cor > TH_COR && delta_mag > TH_MAG)
            angle_cur = ((W_PREV * angle_prev) + (W_GYRO * angle_gyro)) / (W_PREV + W_GYRO);

        angle_prev = angle_cur;
            /*----- Step Detection, Step Length Estimation, and PDR Location Update -----*/

        acc_norm = (float) Math.sqrt(data_acc[0] * data_acc[0] + data_acc[1] * data_acc[1] + data_acc[2] * data_acc[2]);
        steplength = 0;

        if (acc_norm >= MAX_PEAK && acc_norm < acc_p && (t_current_s - t_step_s) > STEP_INTERVAL && isStep == false) {
            // On Max Peak
            isStep = true;
            acc_max = acc_p;
            t_max_peak = t_prior_s;
        } else if (acc_norm <= MIN_PEAK && acc_norm > acc_p && (t_prior_s - t_max_peak) > MIN_MAX_INTERVAL && isStep == true) {
            // On Min Peak
            isStep = false;
            n_step++; // 걸음수 추가
            n_compensation = 0;
            t_step_s = t_prior_s;

            acc_min = acc_p;

            steplength = (float) (Math.sqrt(Math.sqrt(acc_max - acc_min)));
            steplength = STEP_CONSTANT * steplength;

            //여기에 최종적인 X,
            PDRLocation2D.setLocation(PDRLocation2D.getX() + (float) (steplength * Math.cos(angle_cur)), PDRLocation2D.getY() + (float) (steplength * Math.sin(angle_cur)));

        }
        acc_p = acc_norm;
    }

    //BLE scanning Part
    private void scanLeDevice() {
        if (true) {
            // Stops scanning after a pre-defined scan period.
            mHandler.postDelayed(new Runnable() {
                @Override
                public void run() {
                    mBluetoothAdapter.stopLeScan(mLeScanCallback);
                    invalidateOptionsMenu();
                    Toast.makeText(getApplicationContext(), "Scan Stoped", Toast.LENGTH_SHORT).show();
                }
            }, SCAN_PERIOD);


            //Pre-define
            mBluetoothAdapter.startLeScan(mLeScanCallback);
            //Toast.makeText(getApplicationContext(), "Start : Start Scanning", Toast.LENGTH_SHORT).show();

        } else {
            mBluetoothAdapter.stopLeScan(mLeScanCallback);
        }
        invalidateOptionsMenu();
    }


    // BLE Device scan callback.//getInfo
    private BluetoothAdapter.LeScanCallback mLeScanCallback = new BluetoothAdapter.LeScanCallback() {

        @Override
        public void onLeScan(final BluetoothDevice device, final int rssi, byte[] scanRecord) {
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    //데이터 목록
                    //device.getName()  : RECO
                    //device.getAddress() :
                    //rssi  :RSS (INT)
                    //scanRecord : Data bit

                    device.getBondState();
                    //The part for RECO
                    //if("RECO".equals(device.getName())){
                        getBleRSS(device.getAddress(),rssi);
//                    }
                }
            });
        }
    };

    //BLE : get RSS
    private void getBleRSS(String address, int rssi){
        for(int i = 1 ; i < BLE_Set.length ;i++) {
            if ( BLE_Set[i].getAdress().equals(address)) {  //HARD CODING
                if(availableAnchor(availableAnchorset,i)){  //HARD CODING
                    if (!anchors.containsKey(i)) {
                        anchors.put(i,BLE_Set[i]);
                        if(Debug_NodeFinder){
                            Toast.makeText(getApplicationContext(), "New "+i+" Found (Current : " +anchors.size() +" Nodes", Toast.LENGTH_SHORT).show();
                        }

                    }
                    anchors.get(i).addRSS(rssi);
                    anchors.get(i).updateRXtime(t_current_s);


                    //Text Write start
                    if(onText){
                        //anchorNumber, RSS,
                        textWrite(i,                                            //AnchorNumber
                                rssi,                                           //RSS
                                anchors.get(i).get_aptRSS(RSSbuffer,false),     //Average RSS
                                (t_current_s-txtStartTime)/1000,                                    //Timestamp
                                acc_norm,                                       //ACC
                                anchors.get(i).getRawVariance(),                //Average's Variance
                                obtainPLE(rssi,getDistance(new Location2D(36+3,64+3),anchors.get(i)))                                               // 0 = Non-filtered
                                );
                    }

                }
            }
        }
    }//END of isContained

    private float obtainPLE(int _rss, float _dist){
        if(_dist==0){
            return 0;
        }
        if(P0==_rss){
            return 0;
        }else{
            return (P0-_rss)/(10*(float)Math.log(_dist));
        }
    }

    //This class provides
    private boolean availableAnchor(int[] _set, int _targetAnchor){
        boolean result = false;
        for(int i = 0 ; i < _set.length ; i++){
            if(_set[i]==_targetAnchor){
                result=true;
            }
        }
        return result;
    }
    //-------------------------------------------------------------------------------------------------//



    public boolean LSEstimation(Location2D _RSSLocation2D, HashMap<Integer, BleDatabase> _anchors, boolean _isIteration){

        HashMap<Integer, BleDatabase> anchors = (HashMap<Integer, BleDatabase>) _anchors.clone();
        //여기서 Clone을 하는 이유는 위의 _anchors를 그대로 사용할 경우 충돌때문에 고장나기 때문.

        int n = anchors.size();
        //if(MLE_success_flag) return true;             //
        if(n < 1) return false;                         // first iteration termination criterion

        float resolution = (float) RESOLUTION;         //
        float min_rmse = 999999;                        //
        float est_rss;                                  //
        Location2D MLELocation2D = new Location2D(0,0);
        boolean result = false;

        for(float x=min_x; x<=max_x; x+=resolution) {
            for (float y = min_y; y <= max_y; y += resolution) {
                rmse = 0;

                for(BleDatabase anc: anchors.values()){
                    est_rss = getEstimatedRSS(getDistance(new Location2D(x, y), anc));
                    rmse += (anc.get_aptRSS(RSSbuffer,isKF) - est_rss)*(anc.get_aptRSS(RSSbuffer,isKF) - est_rss);
                }

                rmse = (float) Math.sqrt(rmse);

                if(min_rmse > rmse){
                    min_rmse = rmse;
                    MLELocation2D.setLocation(x, y);
                }
            }
        }

        _RSSLocation2D.setLocation(MLELocation2D);   //Non NLOS
        _RSSLocation2D.setRMSE(min_rmse);   //Non NLOS

        if(onNLOS && _isIteration){
            //Initialization
            ArrayList<Location2D> subLocation = new ArrayList<>();
            isNLOS = false;
            int eliminated_AnchorNode = 0;

            //get sub-Location using MLE
            if(anchors.size()>3){
                int temp2 = 0;
                for(int i : anchors.keySet()){

                    HashMap<Integer, BleDatabase> subset = (HashMap<Integer, BleDatabase>) anchors.clone();
                    subset.remove(i);
                    //Toast.makeText(getApplicationContext(),"Error occured from NLOS subset creation : Error A Type", Toast.LENGTH_SHORT).show();
                    subLocation.add(new Location2D(0,0));
                    subLocation.get(temp2).writeEliminatedAnchor(i);
                    LSEstimation(subLocation.get(temp2), subset,false);
                    temp2++;
                }

                //calculate NLOS error distance
                float NLOSErrorDistance=0;
                float tempMinRmse = 9999;
                for(Location2D i : subLocation){
                    //Find MLE location from subset location
                    if(tempMinRmse>i.getRMSE()){
                        tempMinRmse = i.getRMSE();
                        correctedLocation.setLocation(i);
                        correctedLocation.setRMSE(i.getRMSE());
                        correctedLocation.writeEliminatedAnchor(i.getEliminatedAnchor());
                    }
                    //find NLOS error distance
                    for(Location2D j : subLocation){
                        float temp = getDistance(i,j);
                        if(NLOSErrorDistance < temp) NLOSErrorDistance = temp;
                    }
                }
                firstSubLocation = (ArrayList<Location2D>)subLocation.clone();


                //NLOS detection
                if(NLOSErrorDistance > NLOSErrorDistanceThreshold){
                    isNLOS = true;

                }

                //Start Iteration
                if(isNLOS){
                    if(iterationDepth < iterationDepthThreshold){
                        iterationDepth++;
                        HashMap<Integer, BleDatabase> iterationsset = (HashMap<Integer, BleDatabase>) anchors.clone();
                        iterationsset.remove(correctedLocation.getEliminatedAnchor());
                        eliminatedNodeArray.add(correctedLocation.getEliminatedAnchor());
                        LSEstimation(currentLocation2D, iterationsset, true);
                    }
                }

            }

        }

        return true;
    }



    //Purpose of this part is experiment for Prime node solution + Local Landmark solution
    //if this method returns false -> is LM ==false
    public boolean ProposedLSEstimation(Location2D _RSSLocation2D, HashMap<Integer, BleDatabase> anchors, float _range){

        //
        HashMap<Integer, BleDatabase> rankTemp = (HashMap<Integer, BleDatabase>) anchors.clone();
        HashMap<Integer, BleDatabase> redTemp = new HashMap<>();    //For RSS checking. Not related in proposed algorithm.
        HashMap<Integer, BleDatabase> blueTemp= new HashMap<>();    //For RSS checking. Not related in proposed algorithm.

        int tempRefNum = 0;                 //  Ranked node Number for calculation for reference spot detection
        int tempRankNum = 1;                //  Rank number
        int tempReferenceSpot = 0;          //  Reference Spot
        int tempSpotRSS=-99999;             //  For the detecting_calculation
        int tempReferenceSpotRSS=-99999;    //
        boolean result = false;             //  For
        boolean firstForLM = false;         //  The reference spot number is determined at first bubble

        int debugBreaker = 0;

        //0. Calculate RSS power Rank as BUBBLE SORT algorithm
        while(rankTemp.size()>0){   //Bubble loop break
            tempSpotRSS=-99999;
            for (int i : rankTemp.keySet()) {
                if(tempSpotRSS <= rankTemp.get(i).get_aptRSS(RSSbuffer,isKF)  ){
                    tempRefNum = i;
                    tempSpotRSS = rankTemp.get(i).get_aptRSS(RSSbuffer,isKF);
                }
            }   //end of for

            //Get a ranked information
            anchors.get(tempRefNum).setRank(tempRankNum);
            tempRankNum++;

            //Find reference spot at first bubble
            if(!firstForLM){
                Spotnode = tempRefNum;                //for global parameter(and MLE)
                tempReferenceSpotRSS = tempSpotRSS;
                firstForLM = true;
            }
            if(tempRankNum<=redRankNum){ redTemp.put(tempRefNum,anchors.get(tempRefNum));
                //Toast.makeText(getApplicationContext(),"Getting red rank.", Toast.LENGTH_SHORT).show();
            }
            if(tempRankNum<=blueRankNum) blueTemp.put(tempRefNum,anchors.get(tempRefNum));
            rankTemp.remove(tempRefNum);                        //for next calculation

        }

        redRankHash = redTemp;
        blueRankHash = blueTemp;


        //1-1. Landmark Range setting
        //float min_x_LM = anchors.get(Spotnode).getX()-_range;     //
        //float min_y_LM = anchors.get(Spotnode).getY()-_range;     //
        //float max_x_LM = anchors.get(Spotnode).getX()+_range;     //
        //float max_y_LM = anchors.get(Spotnode).getY()+_range;     //

        //2. Estimate max RSS state
        if(tempReferenceSpotRSS <= SpotRSSTHRESHOLD)  {return result;}
        else                            {result = true;}

        //3.LS for Prime node solution
        float resolution = (float) RESOLUTION;
        float min_rmse = 999999;
        float est_rss;
        Location2D tempLMLLELocation2D = new Location2D(0,0);

        for(float x=min_x; x<=max_x; x+=resolution) {
        //for(float x=min_x_LM; x<=max_x_LM; x+=resolution) {
            for (float y = min_y; y <= max_y; y += resolution) {
            //for (float y = min_y_LM; y <= max_y_LM; y += resolution) {
                rmse = 0;

                for(BleDatabase anc: anchors.values()){     //anc is Location2D class

                    //대조군 : 일반계산
                    //est_rss = getEstimatedRSS(getDistance(new Location2D(x, y), anc));

                    //실험군 : MU계산

                    if(totalTable.containsKey(Spotnode)){
                        if(totalTable.get(Spotnode).containsKey(anc.getMyNodeNumber())){
                            //MU가 테이블에 있을때
                            est_rss = getSpatioTemoralRSS(  P0,
                                    getDistance(new Location2D(x, y), anc),
                                    totalTable.get(Spotnode).get(anc.getMyNodeNumber()));
                        }else{
                            //MU가 테이블에 없을때
                            est_rss = getSpatioTemoralRSS(  P0,
                                    getDistance(new Location2D(x, y), anc),
                                    MU);
                        }
                    }else{
                        //MU가 테이블에 없을때
                        est_rss = getSpatioTemoralRSS(  P0,
                                getDistance(new Location2D(x, y), anc),
                                MU);
                    }

                    rmse += (anc.get_aptRSS(RSSbuffer,isKF) - est_rss)*(anc.get_aptRSS(RSSbuffer,isKF) - est_rss);

                }

                rmse = (float) Math.sqrt(rmse);

                if(min_rmse > rmse){
                    min_rmse = rmse;
                    tempLMLLELocation2D.setLocation(x, y);
                }
            }
        }

        _RSSLocation2D.setLocation(tempLMLLELocation2D);   //Non NLOSx = 3;
        _RSSLocation2D.setRMSE(min_rmse);   //Non NLOS

        return result;
    }

    public int getSpatioTemoralRSS(int _P0,float _d, double _MU){
        return (int) Math.round(_P0 - 10 * (_MU) * Math.log10(_d));
    }

    //Purpose of this part is experiment for checking body shading
    //This part requires PDR sensor.
    //I do not recommend use this part someone who wants to configurate RSS based localization
    public boolean MLE_BodyShading(Location2D _resultLocation,
                                   Location2D _currentLocation,
                                   HashMap<Integer, BleDatabase> anchors,
                                   float _azimuth){

        //여기서 Clone을 하는 이유는 위의 _anchors를 그대로 사용할 경우 충돌때문에 고장나기 때문.
        int landmarkAnchor = 0;
        int maxLMRSS=-99999;
        boolean result = false;
        float resolution = (float) RESOLUTION;
        float est_rss;
        float min_rmse = 999999;

        HashMap<Integer, BleDatabase> bodyLOSanchors = (HashMap<Integer, BleDatabase>) anchors.clone();

        //1. Determine AnchorAzimuth
        for (int i : anchors.keySet()) {
            //anchors.get(i).setAnchorAzimuth(currentLocation2D,androidMapOrientation(_azimuth),bodyShadingAngleThreshold);
            anchors.get(i).setAnchorAzimuth(currentLocation2D,_azimuth,bodyShadingAngleThreshold);
                if(!anchors.get(i).getBodyLOS()){
                    if(i!=Spotnode){
                        bodyLOSanchors.remove(i);
                    }
                }
        }



        for(float x=min_x; x<=max_x; x+=resolution) {
            for (float y = min_y; y <= max_y; y += resolution) {
                rmse = 0;

                for(BleDatabase anc: bodyLOSanchors.values()){
                    est_rss = getEstimatedRSS(getDistance(new Location2D(x, y), anc));
                    rmse += (anc.get_aptRSS(RSSbuffer,isKF) - est_rss)*(anc.get_aptRSS(RSSbuffer,isKF) - est_rss);
                }

                rmse = (float) Math.sqrt(rmse);

                if(min_rmse > rmse){
                    min_rmse = rmse;
                    _resultLocation.setLocation(x, y);
                }
            }
        }

        if(getDistance(_resultLocation,_currentLocation)>bodyShadingDistanceThreshold) {return true;}   //BodyNLOS
        else{return false;}     //BodyLOS
    }


    public float getEstimatedDistance(int _rss, int _P0, float _MU){
        return (float) Math.pow(10,(_P0-_rss/10*_MU));

    }

    public int getEstimatedRSS(float d){
        int rss = (int) Math.round(P0 - 10 * MU * Math.log10(d));
        return rss;
    }

    private float getDistance(Location2D loc1, Location2D loc2){
        return (float) Math.sqrt((loc1.getX() - loc2.getX())*(loc1.getX() - loc2.getX()) + (loc1.getY() - loc2.getY())*(loc1.getY() - loc2.getY()));
    }

    //----------------------------------Map Changing----------------------------------------//
    private void changeMap_11FloorAll() {

        //Map image - new ENG 11 floor - 65 : 1 scale ratio
        BitmapFactory.Options options = new BitmapFactory.Options();
        options.inSampleSize = 2;
        bitmap_eng11 = BitmapFactory.decodeResource(getResources(), R.drawable.newengineer11fall65scale, options);
        mapScale = 65;

        map_min_x = 0;
        map_min_y = 0;
        map_max_x = 65;
        map_max_y = 65;

        //MLE range     -   experimental range for IoT& wireless LAB office
        min_x = 30 ;    max_x = 50 ;
        min_y = 50 ;    max_y = 65 ;

        mAttacher.setMaximumScale(10);                  //ZOOM scale

    }

    private void changeMap_2FloorLoby() {

        //Map image - new ENG 2 floor - 16.7 : 1 scale ratio
        BitmapFactory.Options options = new BitmapFactory.Options();
        options.inSampleSize = 2;
        bitmap_eng11 = BitmapFactory.decodeResource(getResources(), R.drawable.newengineer2floby, options);
        mapScale = (float)16.7;

        //MLE range     -   experimental range for IoT& wireless room
        min_x = 0 ;    max_x = 10 ;
        min_y = 0 ;    max_y = 10 ;

    }

    private void changeMap_1FloorLoby() {

        //Map image - new ENG 1 floor - 19.6 : 1 scale ratio
        BitmapFactory.Options options = new BitmapFactory.Options();
        options.inSampleSize = 2;
        bitmap_eng11 = BitmapFactory.decodeResource(getResources(), R.drawable.newengineer1floby, options);
        mapScale = (float)19.6;

        //MLE range     -   experimental range for IoT& wireless room
        min_x = 0 ;    max_x = 22 ;
        min_y = 0 ;    max_y = 22 ;

        map_min_x = min_x;
        map_min_y = min_y;
        map_max_x = max_x;
        map_max_y = max_y;

    }


    //----------------------------------Plotting--------------------------------------------//
    //imgv_map.setImageResource(R.drawable.newengfloor11);//이건 다 덮어뿐다 이기야
    //갤2기준 360,552가 converted됨, screenwidth_pixel은 720나옴 (갤노트2는 1280*720픽셀스펙)

    //Main plotting part
    private void plotCurrentLocation() {


        final float SIZE_MARKER = convert(1)/3; //Marker size

        float tempx = currentLocation2D.getX();
        float tempy = currentLocation2D.getY();

        currentLocation2D_imag.setLocation(tempx,tempy);    //For differ of thread

        float x_converted = convert(tempx);
        float y_converted = convert(tempy);


        //Insert map image
        Paint mappaint = new Paint();
        mappaint.setAntiAlias(true);
        canvas_map.drawBitmap(bitmap_eng11,0,0,mappaint);

        //Draw landmark shade
        //plotLMShade(Spotnode, SpotRANGE); //잠깐 멈추자.


        //Draw map grid
        plotGrid((int)map_min_x,(int)map_min_y,(int)map_max_x,(int)map_max_y,5);
        plotGridDot((int)map_min_x,(int)map_min_y,(int)map_max_x,(int)map_max_y,1);
        plotGridScaleNumber((int)map_min_x,(int)map_min_y,(int)map_max_x,(int)map_max_y,5);

        //Draw Anchor node
        plotAnchors(SIZE_MARKER, onRanking);

        plotAverageError(SIZE_MARKER);

        plotTouchMarker(SIZE_MARKER,errorDBs.marker);

        //Draw subset location
        plotFirstSubLocation(firstSubLocation,SIZE_MARKER);

        //Obsoledate //Draw Landmark result location
        //plotLMresult(ProposedLocation2D,Spotnode,SIZE_MARKER);

        //Obsoledate //Draw Estiamted bodyshading location
        //plotBSDetectingArrow(BScurrentLocation2D,SIZE_MARKER);


        //Only for RSS checking
        //if(isred)   plotArrowConvert(  redRankLocation, 0xFFFF0000,SIZE_MARKER/(float)1.7);
        //if(isblue)   plotArrowConvert( blueRankLocation,0xFF0000FF,SIZE_MARKER/(float)1.7);

        //Main Arrow
        //plotArrowConvert(x_converted,y_converted,0xFF444444,SIZE_MARKER/(float)1.2);

        //plotArrowConvert(currentLocation2D,0x22222222,SIZE_MARKER/(float)1.9);

        if(isSpot){
            plotArrowConvert(ProposedLocation2D,0xFFFF0000,SIZE_MARKER/(float)1.9);
        }


        imgv_map.invalidate();
    }

    private Location2D getCenterLocation(Location2D a, Location2D b){
        return new Location2D((a.getX() + b.getX())/2,(a.getY() + b.getY())/2);
    }

    private void plotTouchMarker(float _size, Location2D _Marker){
        Paint touchMarker = new Paint();
        touchMarker.setColor(Color.RED);
        touchMarker.setAntiAlias(true);

        Paint touchMarkerText = new Paint();
        touchMarkerText.setColor(Color.BLACK);
        touchMarkerText.setTextSize(2 * _size);
        touchMarkerText.setTypeface(Typeface.create(Typeface.DEFAULT, Typeface.BOLD));
        touchMarkerText.setAntiAlias(true);

        canvas_map.drawRect(
                (float)(convert(_Marker.getX()) - 1.5*_size)    , (float)(convert(_Marker.getY()) - 1.5*_size),
                (float)(convert(_Marker.getX()) + 1.5*_size)    , (float)(convert(_Marker.getY()) + 1.5*_size), touchMarker);

        canvas_map.drawText("("+format.format(_Marker.getX())+","+format.format(_Marker.getY())+")", convert((float)(_Marker.getX()+0.5))-(_size/2), convert(_Marker.getY())+(_size-2), touchMarkerText);
        imgv_map.invalidate();

    }

    private void plotAverageError(float _size){
        Paint errorCircle = new Paint();
        errorCircle.setAntiAlias(true);

        Paint errorText = new Paint();
        errorText.setColor(Color.BLACK);
        errorText.setTextSize(1 * _size);       //e.f : Ancrho Node number size is 2
        errorText.setTypeface(Typeface.create(Typeface.DEFAULT, Typeface.BOLD));
        errorText.setAntiAlias(true);

        Location2D _temp = new Location2D(0,0);
        boolean _tempfirst = true;

        for(int i =0; i<errorDBs.targetArray.size() ; i++){
            try{
                Location2D targets = errorDBs.targetArray.get(i);
                Location2D estimations = errorDBs.estimationArray.get(i);

                //DrawEstimation ErrorLine
                errorCircle.setColor(Color.RED);
                canvas_map.drawLine(
                        convert(targets.getX()),     convert(targets.getY()),
                        convert(estimations.getX()), convert(estimations.getY()),   errorCircle );

                //DrawTrajectory
                errorCircle.setColor(Color.BLUE);
                if(_tempfirst){
                    _tempfirst=false;
                }else{
                    canvas_map.drawLine(
                            convert(targets.getX()),     convert(targets.getY()),
                            convert(_temp.getX()), convert(_temp.getY()),   errorCircle );
                }
                _temp = targets;

                //Draw targetNode
                errorCircle.setColor(Color.BLUE);
                canvas_map.drawCircle(convert(targets.getX()),convert(targets.getY()),_size/4,errorCircle);

                //Draw targetNode Step Number
                errorText.setColor(Color.BLACK);
                canvas_map.drawText(""+i+" step",
                        convert((float)(targets.getX()+0.5))-(_size/2),
                        convert(targets.getY())+(_size-2),                  errorText);

                //Draw Estimated Location
                errorCircle.setColor(Color.RED);
                canvas_map.drawCircle(convert(estimations.getX()),convert(estimations.getY()),_size/4,errorCircle);

                //Draw Estimated Location Error distance
                errorText.setColor(Color.RED);
                canvas_map.drawText(""+format.format(errorDBs.getErrorfromIndex(i))+" m",
                        convert((float)(estimations.getX()+0.5))-(_size/2),
                        convert(estimations.getY())+(_size-2),                  errorText);

            }catch(Exception e){
                Toast.makeText(getApplicationContext(),"Error occured from drawing Error DB :" +e.toString(), Toast.LENGTH_SHORT).show();
            }
        }


    }

    //X,YConverter to adapt map     // mapscale example : 650dot / 10dot for 11F
    //convert axis to monitorscale
    private float convert(float _target){return _target*(imgv_map.getWidth()/mapScale);}
    //convert monitorscale to axis
    private float reverseConvert(float _monitorscale){return _monitorscale*(mapScale/imgv_map.getWidth());}

    //Not in Use(Heading Estimation required)
    private void plotBSDetectingArrow(Location2D _BSlocation, float _markersize){

        if(isBodyShadingMLESuccessFlag){

            float _Orientation_result = androidMapOrientation(azimuth);

            //Color setting
            Paint Targetnode = new Paint();
            Targetnode.setColor(Color.WHITE);
            Targetnode.setStyle(Paint.Style.STROKE);
            Targetnode.setStrokeWidth(1);
            Targetnode.setAntiAlias(true);

            Paint ARROW = new Paint(Paint.ANTI_ALIAS_FLAG);

            ARROW.setStrokeWidth(2);
            ARROW.setColor(Color.GREEN);
            ARROW.setAlpha(80);
            ARROW.setStyle(Paint.Style.FILL_AND_STROKE);
            ARROW.setAntiAlias(true);

            //DRAW
            Path path = new Path();
            path.setFillType(Path.FillType.EVEN_ODD);
            path.moveTo((float) (_BSlocation.getX() + 4*_markersize * Math.cos(_Orientation_result)), (float) (_BSlocation.getY() + 4*_markersize * Math.sin(_Orientation_result)));
            path.lineTo((float) (_BSlocation.getX() - 2*_markersize * Math.sin(_Orientation_result)), (float) (_BSlocation.getY() + 2*_markersize * Math.cos(_Orientation_result)));
            path.lineTo((float) (_BSlocation.getX() - 2*_markersize * Math.cos(_Orientation_result)), (float) (_BSlocation.getY() - 2*_markersize * Math.sin(_Orientation_result)));
            path.lineTo((float) (_BSlocation.getX() + 2 * _markersize * Math.sin(_Orientation_result)), (float) (_BSlocation.getY() - 2 * _markersize * Math.cos(_Orientation_result)));
            path.close();

            canvas_map.drawPath(path, ARROW);

            canvas_map.drawCircle(_BSlocation.getX(), _BSlocation.getY(), _markersize/2, Targetnode);

        }

    }

    //Not in Use(Heading Estimation required)
    private float androidMapOrientation(float _azimuth){
        float _defalut_orientation = (float) 0;  //도단위  반시계방향 : +, 시계방향 : -

        return((float) -(azimuth+((_defalut_orientation/180)*Math.PI)));
    }

    //Main Arrow (Auto Converted)
    //private void plotArrow(float _x, float _y , int _arrowColor, float _arrowSize){
    private void plotArrowConvert(Location2D _loc, int _arrowColor, float _arrowSize){
        float _x = convert((float)_loc.getX());
        float _y = convert((float)_loc.getY());
        float _Orientation_result = androidMapOrientation(azimuth);

        //Color setting
        Paint Targetnode = new Paint();
        Targetnode.setColor(Color.WHITE);
        Targetnode.setStyle(Paint.Style.STROKE);
        Targetnode.setStrokeWidth(1);
        Targetnode.setAntiAlias(true);

        Paint ARROW = new Paint(Paint.ANTI_ALIAS_FLAG), TEXT = new Paint();

        ARROW.setStrokeWidth(2);
        ARROW.setColor(_arrowColor);
        ARROW.setAlpha(100);
        ARROW.setStyle(Paint.Style.FILL_AND_STROKE);
        ARROW.setAntiAlias(true);

        //DRAW
        Path path = new Path();
        path.setFillType(Path.FillType.EVEN_ODD);
        path.moveTo((float) (_x + 4*_arrowSize * Math.cos(_Orientation_result)), (float) (_y + 4*_arrowSize * Math.sin(_Orientation_result)));
        path.lineTo((float) (_x - 2*_arrowSize * Math.sin(_Orientation_result)), (float) (_y + 2*_arrowSize * Math.cos(_Orientation_result)));
        path.lineTo((float) (_x - 2*_arrowSize * Math.cos(_Orientation_result)), (float) (_y - 2*_arrowSize * Math.sin(_Orientation_result)));
        path.lineTo((float) (_x + 2 * _arrowSize * Math.sin(_Orientation_result)), (float) (_y - 2 * _arrowSize * Math.cos(_Orientation_result)));
        path.close();

        canvas_map.drawPath(path, ARROW);

        canvas_map.drawCircle(_x, _y, _arrowSize/2, Targetnode);
    }

    //Draw Landmark shade area
    private void plotLMShade(int _LMnode, float _range){
        if (anchors.containsKey(_LMnode) && isSpot) {             //Null check
            Paint LMRangeShade = new Paint();
            LMRangeShade.setColor(Color.LTGRAY);
            LMRangeShade.setAntiAlias(true);

            canvas_map.drawRect(
                    convert(anchors.get(_LMnode).getX()-_range), convert(anchors.get(_LMnode).getY()-_range),
                    convert(anchors.get(_LMnode).getX()+_range), convert(anchors.get(_LMnode).getY()+_range), LMRangeShade);
        }
    }

    //Draw Landmark point (+ Adaptive MU)
    private void plotLMresult(Location2D _estLocation, int _LMnode, float circlesize){

        if (anchors.containsKey(_LMnode) && isSpot) {

            Paint LMResultPoint = new Paint();
            LMResultPoint.setColor(Color.BLACK);
            LMResultPoint.setTextSize(circlesize*2);
            LMResultPoint.setTypeface(Typeface.create(Typeface.DEFAULT, Typeface.BOLD));
            LMResultPoint.setAntiAlias(true);

            canvas_map.drawCircle(convert(_estLocation.getX()), convert(_estLocation.getY()), circlesize/3, LMResultPoint);

        }
    }


    //Plot anchors and its rank from LM results
    private void plotAnchors(float _size, boolean _onRanking) {

        Paint ancRECT = new Paint();
        ancRECT.setColor(Color.BLACK);
        ancRECT.setAntiAlias(true);

        Paint ancText = new Paint();
        ancText.setColor(Color.WHITE);
        ancText.setTextSize(2 * _size);
        ancText.setTypeface(Typeface.create(Typeface.DEFAULT, Typeface.BOLD));
        ancText.setAntiAlias(true);

        Paint ancRankRECT = new Paint();
        ancRankRECT.setColor(Color.BLUE);
        ancRankRECT.setAlpha(0);
        ancRankRECT.setAntiAlias(true);

        Paint ancRankText = new Paint();
        ancRankText.setTextSize(2 * _size);
        ancRankText.setTypeface(Typeface.create(Typeface.DEFAULT, Typeface.BOLD));
        ancRankText.setAntiAlias(true);

        for(int source:anchors.keySet()){
            //Initialization

            //PreDraw Anchor
            float x_converted = convert(anchors.get(source).getX());
            float y_converted = convert(anchors.get(source).getY());


            if(!eliminatedNodeArray.contains(source)){
                //LOS anchor
                ancRECT.setStyle(Paint.Style.FILL);
                ancRECT.setColor(Color.BLACK);
            }else{
                //NLOS anchor
                ancRECT.setStyle(Paint.Style.FILL);
                ancRECT.setColor(Color.YELLOW);
            }
            canvas_map.drawRect(
                    (float)(x_converted - 1.5*_size)    , (float)(y_converted - 1.5*_size),
                    (float)(x_converted + 1.5*_size), (float)(y_converted + 1.5*_size), ancRECT);

            canvas_map.drawText(source + "", x_converted-(_size/(float)1.5), y_converted+(_size/2), ancText);
            imgv_map.invalidate();

            if(_onRanking){
            //PreDraw Rank
            ancRankRECT.setStyle(Paint.Style.STROKE);
            ancRankText.setColor(Color.WHITE);

            if(anchors.get(source).getRank()<4){
                ancRankRECT.setColor(Color.RED);
                ancRankRECT.setStrokeWidth(_size/4);
                canvas_map.drawRect(
                        (float)(x_converted - 1.6*_size)    , (float)(y_converted - 1.6*_size),
                        (float)(x_converted + 1.6*_size), (float)(y_converted + 1.6*_size), ancRankRECT);

                ancRankText.setColor(Color.RED);
                canvas_map.drawText(anchors.get(source).getRank() + "", x_converted+(_size/2), y_converted-(_size+15), ancRankText);
            }else if(anchors.get(source).getRank()<6){
                ancRankRECT.setColor(Color.BLUE);
                ancRankRECT.setStrokeWidth(_size/4);
                canvas_map.drawRect(
                        (float)(x_converted - 1.6*_size)    , (float)(y_converted - 1.6*_size),
                        (float)(x_converted + 1.6*_size), (float)(y_converted + 1.6*_size), ancRankRECT);

                ancRankText.setColor(Color.BLUE);
                canvas_map.drawText(anchors.get(source).getRank() + "", x_converted+(_size/2), y_converted-(_size+15), ancRankText);
            }
            }
            //Plot all
            imgv_map.invalidate();
        }
        imgv_map.invalidate();
    }

    private void drawAnchorCircle(float _markersize){
        Paint kalmanCircle = new Paint();
        kalmanCircle.setStyle(Paint.Style.STROKE);
        kalmanCircle.setStrokeWidth(_markersize*(float)0.4);
        kalmanCircle.setAntiAlias(true);

        for(int source:anchors.keySet()){
            float x_converted = convert(anchors.get(source).getX());
            float y_converted = convert(anchors.get(source).getY());

            kalmanCircle.setColor(Color.RED);
            canvas_map.drawCircle(
                    convert(trackingTemp.getX()),
                    convert(trackingTemp.getY()),
                    convert(getEstimatedDistance(anchors.get(source).get_aptRSS(RSSbuffer,isKF),P0,MU)),
                    kalmanCircle);

            imgv_map.invalidate();
        }



    }


    private void plotFirstSubLocation(ArrayList<Location2D> subloc, float circlesize){  //Csize = 사이즈마커희망
        Paint SubLoc = new Paint();
        SubLoc.setColor(Color.CYAN);
        SubLoc.setStyle(Paint.Style.STROKE);
        SubLoc.setStrokeWidth(3);
        SubLoc.setAntiAlias(true);

        Paint MLSubLoc = new Paint();
        MLSubLoc.setColor(Color.RED);
        MLSubLoc.setStyle(Paint.Style.STROKE);
        MLSubLoc.setStrokeWidth(3);
        MLSubLoc.setAntiAlias(true);

        Paint subText = new Paint();
        subText.setColor(Color.BLACK);
        subText.setTextSize(circlesize*2);
        subText.setTypeface(Typeface.create(Typeface.DEFAULT, Typeface.BOLD));
        subText.setAntiAlias(true);

        for(Location2D i : subloc){
            float tempX = convert(i.getX());
            float tempY = convert(i.getY());
            String temprmse = format.format(i.getRMSE());
            if(i.getX()==correctedLocation.getX() && i.getY()==correctedLocation.getY()){
                canvas_map.drawCircle(tempX, tempY, circlesize/3, MLSubLoc);
            }else{
                canvas_map.drawCircle(tempX, tempY, circlesize/3, SubLoc);
            }
            canvas_map.drawText(""+temprmse, tempX-(circlesize/2), tempY+(circlesize-2), subText);
        }
        canvas_map.drawCircle(convert(correctedLocation.getX()), convert(correctedLocation.getY()), circlesize/3, MLSubLoc);
    }

    private void plotGrid(int minx, int miny, int maxx, int maxy, int resolution){
        Paint _paint = new Paint();
        _paint.setColor(Color.BLACK);
        _paint.setAntiAlias(true);

        //Draw x line
        for(int i = miny ; i<maxy+1 ;i=i+resolution){
            canvas_map.drawLine(convert(0),convert(i),convert(maxx),convert(i),_paint);
        }

        //Draw y line
        for(int i = minx ; i<maxx+1 ;i=i+resolution){
            canvas_map.drawLine(convert(i),convert(0),convert(i),convert(maxy),_paint);
        }
    }

    private void plotGridDot(int minx, int miny, int maxx, int maxy, int resolution){
        Paint _paint = new Paint();
        _paint.setColor(Color.GRAY);
        _paint.setAntiAlias(true);

        //Draw x dot
        for(int i = miny ; i<maxy+1 ;i=i+resolution){
            for(int j = minx ; j<maxx+1 ;j=j+resolution){
                canvas_map.drawCircle(convert(i), convert(j), 1, _paint);
            }
        }
    }

    private void plotGridScaleNumber(int minx, int miny, int maxx, int maxy, int resolution){
        Paint _paint_text = new Paint();
        final int textsize = 10; //Marker size

        _paint_text.setColor(Color.LTGRAY);
        _paint_text.setTextSize(textsize);
        _paint_text.setAntiAlias(true);

        //Draw x line
        for(int i = miny ; i<maxy+1 ;i=i+resolution){
            for(int j = minx ; j<maxx+1 ;j=j+resolution){
                canvas_map.drawText("" + i+","+j, convert(i)+convert(-1) ,convert(j)+convert(0) , _paint_text);
            }
        }

        //Draw y line

    }


    @Override
    public void onBackPressed() {
        DrawerLayout drawer = (DrawerLayout) findViewById(R.id.drawer_layout);
        if (drawer.isDrawerOpen(GravityCompat.START)) {
            drawer.closeDrawer(GravityCompat.START);
        } else {
            super.onBackPressed();
        }
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }

    @SuppressWarnings("StatementWithEmptyBody")
    @Override
    public boolean onNavigationItemSelected(MenuItem item) {
        // Handle navigation view item clicks here.
        int id = item.getItemId();

        if (id == R.id.nav_mu) {
            //------------------
            final float _min = (float) 1.0; final float _step = (float) 0.05;
            final String _scale = " mu";
            final float _default = MU;
            // 아래꺼도 있어요..P0..
            //------------------
            LinearLayout linearLayout = new LinearLayout(MainActivity.this);
            linearLayout.setOrientation(LinearLayout.VERTICAL);
            linearLayout.setPadding(50, 10, 30, 10);
            final TextView textView_value = new TextView(MainActivity.this);
            textView_value.setText("Reference value setting");
            LinearLayout buttonLayout = new LinearLayout(MainActivity.this);
            buttonLayout.setOrientation(LinearLayout.HORIZONTAL);
            buttonLayout.setLayoutParams(new LinearLayout.LayoutParams(LinearLayout.LayoutParams.FILL_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT));
            buttonLayout.setPadding(0, 5, 0, 0);
            Button buttonRollback = new Button(MainActivity.this);
            buttonRollback.setText("Reset:("+_default+")");
            Button buttonConfirm = new Button(MainActivity.this);
            buttonConfirm.setText("Confirm");
            SeekBar seekBar = new SeekBar(MainActivity.this);
            seekBar.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {
                float min = _min;
                float _progressresult=0;
                public void onStopTrackingTouch(SeekBar seekBar) {
                    MU = (float)_progressresult;
                }
                public void onStartTrackingTouch(SeekBar seekBar) { }
                public void onProgressChanged(SeekBar seekBar,int progress, boolean fromUser) {
                    textView_value.setText(String.valueOf( min + progress*(_step)) + _scale);
                    _progressresult =  (float) ( min + progress*(_step));

                }
            });
            //버튼순서
            buttonLayout.addView(buttonRollback,new LinearLayout.LayoutParams(LinearLayout.LayoutParams.FILL_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT,1));
            buttonLayout.addView(buttonConfirm,new LinearLayout.LayoutParams(LinearLayout.LayoutParams.FILL_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT,1));
            linearLayout.addView(textView_value);
            linearLayout.addView(seekBar,new LinearLayout.LayoutParams(400, LinearLayout.LayoutParams.WRAP_CONTENT));
            linearLayout.addView(buttonLayout);

            AlertDialog.Builder builder = new AlertDialog.Builder(MainActivity.this);
            builder.setTitle("Resetting parameter value");
            builder.setView(linearLayout);
            final AlertDialog alert = builder.create();
            alert.show();
            buttonRollback.setOnClickListener(new View.OnClickListener() {
                public void onClick(View v) {
                    MU = (float) _default;
                    Toast.makeText(MainActivity.this,"Roll back: "+MU, Toast.LENGTH_SHORT).show();
                    alert.cancel();
                }
            });
            buttonConfirm.setOnClickListener(new View.OnClickListener() {
                public void onClick(View v) {
                    Toast.makeText(MainActivity.this,"Modifying value : "+MU, Toast.LENGTH_SHORT).show();
                    alert.cancel();
                }
            });
            //------------------





        } else if (id == R.id.nav_p0) {

            //------------------
            final int _min = -100; final float _step =1;
            final String _scale = " dBm";
            final float _default = P0;
            // 아래꺼도 있어요..P0..
            //------------------
            LinearLayout linearLayout = new LinearLayout(MainActivity.this);
            linearLayout.setOrientation(LinearLayout.VERTICAL);
            linearLayout.setPadding(50, 10, 30, 10);
            final TextView textView_value = new TextView(MainActivity.this);
            textView_value.setText("Reference value setting");
            LinearLayout buttonLayout = new LinearLayout(MainActivity.this);
            buttonLayout.setOrientation(LinearLayout.HORIZONTAL);
            buttonLayout.setLayoutParams(new LinearLayout.LayoutParams(LinearLayout.LayoutParams.FILL_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT));
            buttonLayout.setPadding(0, 5, 0, 0);
            Button buttonRollback = new Button(MainActivity.this);
            buttonRollback.setText("Reset:("+_default+")");
            Button buttonConfirm = new Button(MainActivity.this);
            buttonConfirm.setText("Confirm");
            SeekBar seekBar = new SeekBar(MainActivity.this);
            seekBar.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {
                int min = _min;
                float _progressresult=0;
                public void onStopTrackingTouch(SeekBar seekBar) {
                    P0 = (int)_progressresult;
                }
                public void onStartTrackingTouch(SeekBar seekBar) { }
                public void onProgressChanged(SeekBar seekBar,int progress, boolean fromUser) {
                    textView_value.setText(String.valueOf( min + progress*(_step)) + _scale);
                    _progressresult =  (float) ( min + progress*(_step));

                }
            });
            //버튼순서
            buttonLayout.addView(buttonRollback,new LinearLayout.LayoutParams(LinearLayout.LayoutParams.FILL_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT,1));
            buttonLayout.addView(buttonConfirm,new LinearLayout.LayoutParams(LinearLayout.LayoutParams.FILL_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT,1));
            linearLayout.addView(textView_value);
            linearLayout.addView(seekBar,new LinearLayout.LayoutParams(400, LinearLayout.LayoutParams.WRAP_CONTENT));
            linearLayout.addView(buttonLayout);

            AlertDialog.Builder builder = new AlertDialog.Builder(MainActivity.this);
            builder.setTitle("Resetting parameter value");
            builder.setView(linearLayout);
            final AlertDialog alert = builder.create();
            alert.show();
            buttonRollback.setOnClickListener(new View.OnClickListener() {
                public void onClick(View v) {
                    P0 = (int) _default;
                    Toast.makeText(MainActivity.this,"Roll back: "+P0, Toast.LENGTH_SHORT).show();
                    alert.cancel();
                }
            });
            buttonConfirm.setOnClickListener(new View.OnClickListener() {
                public void onClick(View v) {
                    Toast.makeText(MainActivity.this,"Modifying value : "+P0, Toast.LENGTH_SHORT).show();
                    alert.cancel();
                }
            });
            //------------------






        } else if (id == R.id.nav_rssbuffer) {

            //------------------
            final int _min = 1; final int _step =1;
            final String _scale = " RSS values";
            final float _default = RSSbuffer;
            // 아래꺼도 있어요..P0..
            //------------------
            LinearLayout linearLayout = new LinearLayout(MainActivity.this);
            linearLayout.setOrientation(LinearLayout.VERTICAL);
            linearLayout.setPadding(50, 10, 30, 10);
            final TextView textView_value = new TextView(MainActivity.this);
            textView_value.setText("Reference value setting");
            LinearLayout buttonLayout = new LinearLayout(MainActivity.this);
            buttonLayout.setOrientation(LinearLayout.HORIZONTAL);
            buttonLayout.setLayoutParams(new LinearLayout.LayoutParams(LinearLayout.LayoutParams.FILL_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT));
            buttonLayout.setPadding(0, 5, 0, 0);
            Button buttonRollback = new Button(MainActivity.this);
            buttonRollback.setText("Reset:("+_default+")");
            Button buttonConfirm = new Button(MainActivity.this);
            buttonConfirm.setText("Confirm");
            SeekBar seekBar = new SeekBar(MainActivity.this);
            seekBar.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {
                int min = _min;
                float _progressresult=0;
                public void onStopTrackingTouch(SeekBar seekBar) {
                    RSSbuffer = (int)_progressresult;
                }
                public void onStartTrackingTouch(SeekBar seekBar) { }
                public void onProgressChanged(SeekBar seekBar,int progress, boolean fromUser) {
                    textView_value.setText(String.valueOf( min + progress*(_step)) + _scale);
                    _progressresult =  (float) ( min + progress*(_step));

                }
            });
            //버튼순서
            buttonLayout.addView(buttonRollback,new LinearLayout.LayoutParams(LinearLayout.LayoutParams.FILL_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT,1));
            buttonLayout.addView(buttonConfirm,new LinearLayout.LayoutParams(LinearLayout.LayoutParams.FILL_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT,1));
            linearLayout.addView(textView_value);
            linearLayout.addView(seekBar,new LinearLayout.LayoutParams(400, LinearLayout.LayoutParams.WRAP_CONTENT));
            linearLayout.addView(buttonLayout);

            AlertDialog.Builder builder = new AlertDialog.Builder(MainActivity.this);
            builder.setTitle("Resetting parameter value");
            builder.setView(linearLayout);
            final AlertDialog alert = builder.create();
            alert.show();
            buttonRollback.setOnClickListener(new View.OnClickListener() {
                public void onClick(View v) {
                    RSSbuffer = (int) _default;
                    Toast.makeText(MainActivity.this,"Roll back: "+RSSbuffer, Toast.LENGTH_SHORT).show();
                    alert.cancel();
                }
            });
            buttonConfirm.setOnClickListener(new View.OnClickListener() {
                public void onClick(View v) {
                    Toast.makeText(MainActivity.this,"Modifying value : "+RSSbuffer, Toast.LENGTH_SHORT).show();
                    alert.cancel();
                }
            });





        } else if (id == R.id.nav_resolution) {
            //------------------
            final float _min = (float) 0.1; final float _step = (float) 0.1;
            final String _scale = "meter";
            final float _default = RESOLUTION;
            // 아래꺼도 있어요..P0..
            //------------------
            LinearLayout linearLayout = new LinearLayout(MainActivity.this);
            linearLayout.setOrientation(LinearLayout.VERTICAL);
            linearLayout.setPadding(50, 10, 30, 10);
            final TextView textView_value = new TextView(MainActivity.this);
            textView_value.setText("Reference value setting");
            LinearLayout buttonLayout = new LinearLayout(MainActivity.this);
            buttonLayout.setOrientation(LinearLayout.HORIZONTAL);
            buttonLayout.setLayoutParams(new LinearLayout.LayoutParams(LinearLayout.LayoutParams.FILL_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT));
            buttonLayout.setPadding(0, 5, 0, 0);
            Button buttonRollback = new Button(MainActivity.this);
            buttonRollback.setText("Reset:("+_default+")");
            Button buttonConfirm = new Button(MainActivity.this);
            buttonConfirm.setText("Confirm");
            SeekBar seekBar = new SeekBar(MainActivity.this);
            seekBar.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {
                float min = _min;
                float _progressresult=0;
                public void onStopTrackingTouch(SeekBar seekBar) {
                    RESOLUTION = (float)_progressresult;
                }
                public void onStartTrackingTouch(SeekBar seekBar) { }
                public void onProgressChanged(SeekBar seekBar,int progress, boolean fromUser) {
                    textView_value.setText(String.valueOf( min + progress*(_step)) + _scale);
                    _progressresult =  (float) ( min + progress*(_step));

                }
            });
            //버튼순서
            buttonLayout.addView(buttonRollback,new LinearLayout.LayoutParams(LinearLayout.LayoutParams.FILL_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT,1));
            buttonLayout.addView(buttonConfirm,new LinearLayout.LayoutParams(LinearLayout.LayoutParams.FILL_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT,1));
            linearLayout.addView(textView_value);
            linearLayout.addView(seekBar,new LinearLayout.LayoutParams(400, LinearLayout.LayoutParams.WRAP_CONTENT));
            linearLayout.addView(buttonLayout);

            AlertDialog.Builder builder = new AlertDialog.Builder(MainActivity.this);
            builder.setTitle("Resetting parameter value");
            builder.setView(linearLayout);
            final AlertDialog alert = builder.create();
            alert.show();
            buttonRollback.setOnClickListener(new View.OnClickListener() {
                public void onClick(View v) {
                    RESOLUTION = (float) _default;
                    Toast.makeText(MainActivity.this,"Roll back: "+RESOLUTION, Toast.LENGTH_SHORT).show();
                    alert.cancel();
                }
            });
            buttonConfirm.setOnClickListener(new View.OnClickListener() {
                public void onClick(View v) {
                    Toast.makeText(MainActivity.this,"Modifying value : "+RESOLUTION, Toast.LENGTH_SHORT).show();
                    alert.cancel();
                }
            });
            //------------------


        } else if (id == R.id.nav_share) {

        } else if (id == R.id.nav_send) {

        }

        DrawerLayout drawer = (DrawerLayout) findViewById(R.id.drawer_layout);
        drawer.closeDrawer(GravityCompat.START);
        return true;
    }

    //--------------------------for Experiment-------------------------------------//
    private void textWrite(int _anchorNumber, int _rss, int _avg_rss, float _timestamp, float _acc_norm, float _variance, float _PLE){

        if(firstTxtWrite){
            firstTxtWrite = false;
            txtName = "RSS"+System.currentTimeMillis();
            file = new File(dirPath + "/data"+ txtName +".txt");


            try{
                fos = new FileOutputStream(file);
                Toast.makeText (this, "Start Text Writing : ("+ txtName +")", Toast.LENGTH_LONG).show();
                firstTxtWrite = false;
                txtStartTime = _timestamp;
            } catch(FileNotFoundException e){
                e.printStackTrace();
                Toast.makeText (this, "Error in file output stream initialization error : "+e.toString(), Toast.LENGTH_LONG).show();
                firstTxtWrite = false;
                txtStartTime = 0;
            }

        }

        if(fos!=null) {
            try {
                String RSS_string = new String(""+_anchorNumber+","+_rss+","+_avg_rss+","+_variance+","+_timestamp+","+_acc_norm+","+_PLE+"\n");
                fos.write(RSS_string.getBytes());
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

    }

    private void textWriteEnd(){
        if(fos!=null) {
            try {
                //Ckear Resiyrces
                Toast.makeText (this, "End Saving : "+txtName, Toast.LENGTH_LONG).show();
                fos.close();
                firstTxtWrite = true;
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    private void textSaveErrorDB(String fileName){
        FileOutputStream fos2 = null;  String dirPath = "/sdcard/RSS/";
        File file2;
        file2 = new File(dirPath + "/data"+ fileName +".txt");

        //Initialization
        try{
            fos2 = new FileOutputStream(file2);
        } catch(FileNotFoundException e){
            Toast.makeText(getApplicationContext(),"errTXT Save Error 1 : Can't create fos", Toast.LENGTH_SHORT).show();
            e.printStackTrace();
        }

        //Writing using FileOutputStream
        if(fos2!=null) {
            try {

                for(int i =0; i<errorDBs.targetArray.size() ; i++){
                    String Txt_String = new String(""+errorDBs.targetArray.get(i).getX()+","+                   // Targetnode X
                            errorDBs.targetArray.get(i).getY()+","+                                             // Targetnode Y
                            errorDBs.estimationArray.get(i).getX()+","+                                         // Est X
                            errorDBs.estimationArray.get(i).getY()+","+                                         // Est Y
                            getDistance(errorDBs.estimationArray.get(i),errorDBs.targetArray.get(i))+"\n");     // Error distance
                    fos2.write(Txt_String.getBytes());
                }
            } catch (IOException e) {
                Toast.makeText(getApplicationContext(),"errTXT Save Error 2 : FOS is null", Toast.LENGTH_SHORT).show();
                e.printStackTrace();
            }
        }

        //Resource clear and end
        if(fos2!=null) {
            try {
                fos2.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

    }

    //--------------Spatiotemporal path loss exponent measurement and update----------------//

    private void muUpdate (ConfigurableDevice _confdevice){

         ;
        //This method requires both of the Estimote-library and SDK


        if (onMUupdate){    //Checking the programmer setting
            //pre-order for update
            currentMuTable = getSpatiotemporalPathlossExmponentValue();
            printMuTable(currentMuTable);
            totalTable.put(Spotnode,currentMuTable);

            if(!onlyMucalculation){
                //start update
                Toast.makeText (getApplicationContext(), "Start connection("+currentMuTable.size()+" data)", Toast.LENGTH_SHORT).show();
                configurableDevice = _confdevice;
                connectionProvider = new DeviceConnectionProvider(this);
                connectToDevice();
            }

        }
    }

    private void printMuTable(HashMap<Integer, Double> table){
        String out_result = "";

        for(Integer _nodeNum : table.keySet()){
            out_result = out_result+format.format(table.get(_nodeNum))+"("+_nodeNum+")";
        }
        if(table.size()==0){
            Toast.makeText (getApplicationContext(), "No tables"+out_result, Toast.LENGTH_SHORT).show();
        }else{
            Toast.makeText (getApplicationContext(), "Update Result : "+out_result, Toast.LENGTH_SHORT).show();
        }

    }

    private String getMuTableResult(HashMap<Integer, Double> table){
        String out_result = "";
        for(Integer _nodeNum : table.keySet()){
            out_result = out_result+format.format(table.get(_nodeNum))+"("+_nodeNum+")";
        }
        if(table.size()==0){
            out_result = "No table in current";
        }
        return out_result;

    }

    private HashMap<Integer, Double> getSpatiotemporalPathlossExmponentValue(){
        HashMap<Integer, Double> muTable = new HashMap<>();
        int n = anchors.size(); //get from static anchors.
        double tempMU = 0;

        if(onMUupdate){
            if(n < 1){
                //Exceptions
                Toast.makeText(getApplicationContext(), "Error41 : No anchors in current main HashMap", Toast.LENGTH_SHORT).show();
            }else{
                //
                String muResult_Info = "";
                for(BleDatabase anc: anchors.values()){
                    //obtain MU
                    if(getDistance(anc, ProposedLocation2D) == 0){
                        tempMU = MU;    // Zero distance will derive to error.
                    }else{
                        tempMU = (P0-anc.get_aptRSS(RSSbuffer,isKF))/(getDistance(anc, ProposedLocation2D));    //Estimated MU
                        muResult_Info = muResult_Info + (format.format(tempMU)) + " ";  //For debugging Toast
                    }

                    if(muNoiseGuard){
                        //reinforced MU
                        if(tempMU > MU+ muNoiseGuardValue){
                            //Too high MU
                            tempMU = MU+ muNoiseGuardValue;
                        }else if (tempMU < MU- muNoiseGuardValue){
                            //Too low MU
                            tempMU = MU- muNoiseGuardValue;
                        }else{
                            //tempMU is in Noise guard
                        }
                    }else{
                        //Non-reinfourcedMU
                    }
                    //record in Table
                    muTable.put(anc.getMyNodeNumber(),tempMU);
                }
                Toast.makeText(getApplicationContext(), "Calculation success", Toast.LENGTH_SHORT).show();
            }
        }


        return muTable;
    }

    private void connectToDevice() {
        Log.d("Gonai_System :", "connectToDevice started");
        if (connection == null || !connection.isConnected()) {
            //If not Conn, feed device information to connection.
            connectionProvider.connectToService(new DeviceConnectionProvider.ConnectionProviderCallback() {
                @Override
                public void onConnectedToService() {
                    //Excute connection
                    connection = connectionProvider.getConnection(configurableDevice);
                    connection.connect(new DeviceConnectionCallback() {
                        @Override
                        public void onConnected() {
                            //LOG
                            Toast.makeText(getApplicationContext(), "Beacon connected", Toast.LENGTH_SHORT).show();
                            Log.d("Gonai_Debug :", "connectToDevice : The device connected");
                            t_current_s = (float) (System.currentTimeMillis() - t_begin) / (float) 1000;
                            tv_info.append("Pairing establish time : " + t_current_s + "\n");

                            //Startwriting
                            progressDialog = ProgressDialog.show(MainActivity.this, ".", ".");     //Process shows
                            writeSettings();                                                                //Writing start
                        }

                        @Override
                        public void onDisconnected() {
                            Toast.makeText(getApplicationContext(), "Update successed and disconnected", Toast.LENGTH_SHORT).show();
                            Log.d("Gonai_Debug :", "connectToDevice : The device disconnected");
                        }

                        @Override
                        public void onConnectionFailed(DeviceConnectionException e) {
                            Log.d("Gonai_Debug :", "connectToDevice : Error in the device connection :" + e.toString());
                        }
                    });
                }
            });
        }
    }

    private void writeSettings() {
        Log.d("Gonai_Debug","writeSetting Started");
        SettingsEditor edit = connection.edit();        // Interface for changing multiple settings at once.

        //여기에 패스로스 계산정보 삽입
        Log.d("Gonai_Debug","writeSetting : 1 edit.set");
        edit.set(connection.settings.beacon.enable(), true);
        edit.set(connection.settings.beacon.proximityUUID(), UUID.fromString("B9407F30-F5F8-466E-AFF9-000000001000")); // You might want all your beacons to have a certain UUID.
        //Default UUID in estimote is.. B9407F30-F5F8-466E-AFF9-25556B57FE6D

        progressDialog.setTitle(R.string.writing_settings);
        progressDialog.setMessage(getString(R.string.please_wait));

        //Sets device setting to given value.
        Log.d("Gonai_Debug","writeSetting : 2 edit.commit");
        edit.commit(new SettingCallback() {
            @Override
            public void onSuccess(Object o) {
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        t_prior_s = t_current_s;
                        t_current_s = (float) (System.currentTimeMillis() - t_begin) / (float) 1000;
                        tv_info.append("UUID writing time : "+(t_current_s-t_prior_s)+"\n");
                        progressDialog.dismiss();
                        //displaySuccess();
                        closeConnection();
                        t_prior_s = t_current_s;
                        t_current_s = (float) (System.currentTimeMillis() - t_begin) / (float) 1000;
                        tv_info.append("Disconnection time : "+(t_current_s-t_prior_s)+"\n");

                    }
                });
            }

            @Override
            public void onFailure(DeviceConnectionException e) {
                final DeviceConnectionException eF = e;
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        progressDialog.dismiss();
                        displayError(eF);
                    }
                });
            }
        });
    }

    private void closeConnection(){
        if (connection != null && connection.isConnected())
            connection.close();
    }

    //Alert message box
    private void displaySuccess() {
        android.support.v7.app.AlertDialog.Builder builder = new android.support.v7.app.AlertDialog.Builder(this);
        builder.setMessage(R.string.configuration_succeeded);
        builder.setCancelable(true);
        builder.setPositiveButton(
                R.string.configure_next_beacon,
                new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int id) {
                        dialog.cancel();
                    }
                });
        android.support.v7.app.AlertDialog alert = builder.create();
        alert.show();

    }
    //Alert message box
    private void displayError(DeviceConnectionException e) {
        android.support.v7.app.AlertDialog.Builder builder = new android.support.v7.app.AlertDialog.Builder(this);
        builder.setMessage(e.getLocalizedMessage());
        builder.setCancelable(true);
        builder.setPositiveButton(
                R.string.alert_ok,
                new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int id) {
                        dialog.cancel();
                        finish();
                    }
                });
        android.support.v7.app.AlertDialog alert = builder.create();
        alert.show();
    }


}
