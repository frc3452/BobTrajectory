package org.usfirst.frc.team319.arcs;

import org.usfirst.frc.team319.models.SrxMotionProfile;
import org.usfirst.frc.team319.models.SrxTrajectory;

public class CenterToLeftSwitchPt5Arc extends SrxTrajectory{
	
	// WAYPOINTS:
	// (X,Y,degrees)
	// (2.03,13.08,0.00)
	// (8.40,17.08,0.00)
	
    public CenterToLeftSwitchPt5Arc() {
		super();
		this.highGear = true;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}

	
    public CenterToLeftSwitchPt5Arc(boolean flipped) {
		super();
		this.highGear = true;
		this.flipped = flipped;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}

	public boolean highGear = true;

	double[][] centerPoints = {
				{0.568,11.351,10.000,0.000},
				{2.838,34.054,10.000,0.000},
				{7.946,68.107,10.000,0.000},
				{17.027,113.512,10.000,0.000},
				{31.216,170.268,10.000,0.000},
				{51.648,238.376,10.000,0.000},
				{79.459,317.834,10.000,0.000},
				{115.783,408.644,10.000,0.000},
				{161.755,510.805,10.000,0.000},
				{218.511,624.318,10.000,0.000},
				{287.186,749.181,10.000,0.000},
				{368.915,885.396,10.000,0.000},
				{464.833,1032.962,10.000,0.010},
				{576.075,1191.879,10.000,0.010},
				{703.209,1350.797,10.000,0.010},
				{846.234,1509.714,10.000,0.020},
				{1005.152,1668.631,10.000,0.020},
				{1179.961,1827.548,10.000,0.030},
				{1370.661,1986.466,10.000,0.050},
				{1577.254,2145.383,10.000,0.060},
				{1799.738,2304.300,10.000,0.080},
				{2038.114,2463.217,10.000,0.100},
				{2292.381,2622.135,10.000,0.130},
				{2562.541,2781.052,10.000,0.160},
				{2848.592,2939.969,10.000,0.200},
				{3150.534,3098.886,10.000,0.240},
				{3468.369,3257.804,10.000,0.290},
				{3802.095,3416.721,10.000,0.350},
				{4151.713,3575.638,10.000,0.410},
				{4517.223,3734.555,10.000,0.490},
				{4898.624,3893.472,10.000,0.570},
				{5295.917,4052.390,10.000,0.660},
				{5709.102,4211.307,10.000,0.770},
				{6138.179,4370.224,10.000,0.890},
				{6583.147,4529.141,10.000,1.020},
				{7044.007,4688.059,10.000,1.160},
				{7520.759,4846.976,10.000,1.320},
				{8013.402,5005.893,10.000,1.490},
				{8521.937,5164.810,10.000,1.680},
				{9046.364,5323.728,10.000,1.890},
				{9586.683,5482.645,10.000,2.120},
				{10142.893,5641.562,10.000,2.370},
				{10714.995,5800.479,10.000,2.630},
				{11302.989,5959.397,10.000,2.920},
				{11906.875,6118.314,10.000,3.240},
				{12526.652,6277.231,10.000,3.580},
				{13162.321,6436.148,10.000,3.940},
				{13813.881,6595.066,10.000,4.330},
				{14481.334,6753.983,10.000,4.750},
				{15164.678,6912.900,10.000,5.200},
				{15863.914,7071.817,10.000,5.680},
				{16579.042,7230.735,10.000,6.200},
				{17310.061,7389.652,10.000,6.750},
				{18056.972,7548.569,10.000,7.330},
				{18819.775,7707.486,10.000,7.960},
				{19598.469,7866.404,10.000,8.620},
				{20393.055,8025.321,10.000,9.330},
				{21203.533,8184.238,10.000,10.070},
				{22029.903,8343.155,10.000,10.870},
				{22872.164,8502.073,10.000,11.700},
				{23730.318,8660.990,10.000,12.590},
				{24604.362,8819.907,10.000,13.520},
				{25494.299,8978.824,10.000,14.500},
				{26400.127,9137.742,10.000,15.530},
				{27321.847,9296.659,10.000,16.600},
				{28259.459,9455.576,10.000,17.730},
				{29212.962,9614.493,10.000,18.910},
				{30182.358,9773.411,10.000,20.140},
				{31167.645,9932.328,10.000,21.410},
				{32168.823,10091.245,10.000,22.720},
				{33185.894,10250.162,10.000,24.080},
				{34218.856,10409.080,10.000,25.480},
				{35267.709,10567.997,10.000,26.910},
				{36332.455,10726.914,10.000,28.380},
				{37413.092,10885.831,10.000,29.870},
				{38509.054,11033.397,10.000,31.370},
				{39619.204,11169.612,10.000,32.890},
				{40742.409,11294.476,10.000,34.410},
				{41877.532,11407.988,10.000,35.930},
				{43023.439,11510.149,10.000,37.430},
				{44178.994,11600.959,10.000,38.920},
				{45343.063,11680.417,10.000,40.370},
				{46514.510,11748.525,10.000,41.800},
				{47692.200,11805.281,10.000,43.180},
				{48874.999,11850.686,10.000,44.520},
				{50061.770,11884.740,10.000,45.810},
				{51251.379,11907.442,10.000,47.040},
				{52442.691,11918.793,10.000,48.220},
				{53634.570,11918.793,10.000,49.350},
				{54826.449,11918.793,10.000,50.410},
				{56018.329,11918.793,10.000,51.420},
				{57210.208,11918.793,10.000,52.370},
				{58402.087,11918.793,10.000,53.260},
				{59593.967,11918.793,10.000,54.100},
				{60785.846,11918.793,10.000,54.880},
				{61977.725,11918.793,10.000,55.610},
				{63169.605,11918.793,10.000,56.290},
				{64361.484,11918.793,10.000,56.910},
				{65553.363,11918.793,10.000,57.490},
				{66745.243,11918.793,10.000,58.020},
				{67937.122,11918.793,10.000,58.490},
				{69129.001,11918.793,10.000,58.930},
				{70320.881,11918.793,10.000,59.310},
				{71512.760,11918.793,10.000,59.660},
				{72704.639,11918.793,10.000,59.950},
				{73896.519,11918.793,10.000,60.210},
				{75088.398,11918.793,10.000,60.420},
				{76280.277,11918.793,10.000,60.600},
				{77472.157,11918.793,10.000,60.730},
				{78664.036,11918.793,10.000,60.810},
				{79855.915,11918.793,10.000,60.860},
				{81047.795,11918.793,10.000,60.870},
				{82239.674,11918.793,10.000,60.840},
				{83431.553,11918.793,10.000,60.770},
				{84623.433,11918.793,10.000,60.650},
				{85815.312,11918.793,10.000,60.500},
				{87007.191,11918.793,10.000,60.300},
				{88199.071,11918.793,10.000,60.070},
				{89390.950,11918.793,10.000,59.780},
				{90582.829,11918.793,10.000,59.460},
				{91774.709,11918.793,10.000,59.090},
				{92966.588,11918.793,10.000,58.680},
				{94158.467,11918.793,10.000,58.220},
				{95350.347,11918.793,10.000,57.710},
				{96542.226,11918.793,10.000,57.160},
				{97734.105,11918.793,10.000,56.550},
				{98925.985,11918.793,10.000,55.900},
				{100117.864,11918.793,10.000,55.190},
				{101309.743,11918.793,10.000,54.430},
				{102501.623,11918.793,10.000,53.620},
				{103693.502,11918.793,10.000,52.750},
				{104885.381,11918.793,10.000,51.820},
				{106077.261,11918.793,10.000,50.840},
				{107269.140,11918.793,10.000,49.800},
				{108461.019,11918.793,10.000,48.700},
				{109652.566,11912.133,10.000,47.540},
				{110842.878,11894.122,10.000,46.320},
				{112030.822,11864.759,10.000,45.060},
				{113215.263,11824.045,10.000,43.740},
				{114395.064,11771.980,10.000,42.370},
				{115569.091,11708.563,10.000,40.970},
				{116736.209,11633.795,10.000,39.520},
				{117895.283,11547.677,10.000,38.050},
				{119045.177,11450.206,10.000,36.550},
				{120184.756,11341.385,10.000,35.040},
				{121312.886,11221.212,10.000,33.520},
				{122428.431,11089.689,10.000,32.000},
				{123530.256,10946.813,10.000,30.490},
				{124617.226,10792.587,10.000,28.990},
				{125688.539,10633.670,10.000,27.520},
				{126743.960,10474.753,10.000,26.070},
				{127783.490,10315.835,10.000,24.660},
				{128807.127,10156.918,10.000,23.280},
				{129814.873,9998.001,10.000,21.950},
				{130806.728,9839.084,10.000,20.660},
				{131782.690,9680.166,10.000,19.410},
				{132742.761,9521.249,10.000,18.220},
				{133686.940,9362.332,10.000,17.070},
				{134615.227,9203.415,10.000,15.970},
				{135527.623,9044.497,10.000,14.920},
				{136424.127,8885.580,10.000,13.920},
				{137304.739,8726.663,10.000,12.970},
				{138169.459,8567.746,10.000,12.060},
				{139018.288,8408.828,10.000,11.210},
				{139851.225,8249.911,10.000,10.400},
				{140668.270,8090.994,10.000,9.630},
				{141469.424,7932.077,10.000,8.910},
				{142254.686,7773.160,10.000,8.230},
				{143024.056,7614.242,10.000,7.590},
				{143777.534,7455.325,10.000,6.990},
				{144515.121,7296.408,10.000,6.420},
				{145236.816,7137.491,10.000,5.890},
				{145942.619,6978.573,10.000,5.400},
				{146632.530,6819.656,10.000,4.930},
				{147306.550,6660.739,10.000,4.500},
				{147964.678,6501.822,10.000,4.100},
				{148606.914,6342.904,10.000,3.720},
				{149233.259,6183.987,10.000,3.370},
				{149843.712,6025.070,10.000,3.050},
				{150438.273,5866.153,10.000,2.750},
				{151016.942,5707.235,10.000,2.470},
				{151579.720,5548.318,10.000,2.220},
				{152126.606,5389.401,10.000,1.980},
				{152657.600,5230.484,10.000,1.770},
				{153172.703,5071.566,10.000,1.570},
				{153671.913,4912.649,10.000,1.390},
				{154155.232,4753.732,10.000,1.220},
				{154622.660,4594.815,10.000,1.070},
				{155074.195,4435.897,10.000,0.940},
				{155509.839,4276.980,10.000,0.820},
				{155929.591,4118.063,10.000,0.710},
				{156333.452,3959.146,10.000,0.610},
				{156721.420,3800.228,10.000,0.520},
				{157093.497,3641.311,10.000,0.440},
				{157449.683,3482.394,10.000,0.370},
				{157789.976,3323.477,10.000,0.310},
				{158114.378,3164.559,10.000,0.260},
				{158422.888,3005.642,10.000,0.210},
				{158715.506,2846.725,10.000,0.170},
				{158992.233,2687.808,10.000,0.140},
				{159253.068,2528.890,10.000,0.110},
				{159498.011,2369.973,10.000,0.090},
				{159727.063,2211.056,10.000,0.070},
				{159940.222,2052.139,10.000,0.050},
				{160137.490,1893.221,10.000,0.040},
				{160318.867,1734.304,10.000,0.030},
				{160484.351,1575.387,10.000,0.020},
				{160633.944,1416.470,10.000,0.010},
				{160767.645,1257.552,10.000,0.010},
				{160885.455,1098.635,10.000,0.010},
				{160987.705,946.378,10.000,0.000},
				{161075.298,805.473,10.000,0.000},
				{161149.367,675.918,10.000,0.000},
				{161211.049,557.715,10.000,0.000},
				{161261.478,450.863,10.000,0.000},
				{161301.789,355.362,10.000,0.000},
				{161333.118,271.212,10.000,0.000},
				{161356.599,198.414,10.000,0.000},
				{161373.368,136.967,10.000,0.000},
				{161384.560,86.871,10.000,0.000},
				{161391.310,48.127,10.000,0.000},
				{161394.753,20.733,10.000,0.000},
				{161396.024,4.691,10.000,0.000},
				{161396.259,0.000,10.000,0.000},
				{161396.259,0.000,10.000,0.000}		};

}