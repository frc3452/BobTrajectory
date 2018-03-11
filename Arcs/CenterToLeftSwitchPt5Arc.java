package org.usfirst.frc.team319.arcs;

import org.usfirst.frc.team319.models.SrxMotionProfile;
import org.usfirst.frc.team319.models.SrxTrajectory;

public class CenterToLeftSwitchPt5Arc extends SrxTrajectory{
	
	// WAYPOINTS:
	// (X,Y,degrees)
	// (4.03,13.08,0.00)
	// (10.40,17.08,0.00)
	
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
				{0.445,8.895,10.000,0.000},
				{2.224,17.788,10.000,0.000},
				{6.226,40.021,10.000,0.000},
				{13.340,71.140,10.000,0.000},
				{24.453,111.134,10.000,0.000},
				{40.452,159.984,10.000,0.000},
				{62.218,217.663,10.000,0.000},
				{90.631,284.133,10.000,0.000},
				{126.565,359.344,10.000,0.001},
				{170.889,443.233,10.000,0.001},
				{224.461,535.719,10.000,0.002},
				{288.131,636.705,10.000,0.004},
				{362.739,746.077,10.000,0.006},
				{449.109,863.700,10.000,0.009},
				{548.050,989.416,10.000,0.013},
				{660.355,1123.049,10.000,0.020},
				{786.795,1264.396,10.000,0.028},
				{928.118,1413.231,10.000,0.039},
				{1085.049,1569.305,10.000,0.053},
				{1258.282,1732.339,10.000,0.071},
				{1448.055,1897.727,10.000,0.095},
				{1654.149,2060.937,10.000,0.124},
				{1876.328,2221.793,10.000,0.160},
				{2114.341,2380.128,10.000,0.203},
				{2367.918,2535.773,10.000,0.256},
				{2636.775,2688.565,10.000,0.318},
				{2920.609,2838.341,10.000,0.391},
				{3219.103,2984.941,10.000,0.477},
				{3531.923,3128.204,10.000,0.576},
				{3858.720,3267.965,10.000,0.691},
				{4199.126,3404.061,10.000,0.821},
				{4552.758,3536.321,10.000,0.970},
				{4919.215,3664.572,10.000,1.139},
				{5298.078,3788.632,10.000,1.329},
				{5688.910,3908.314,10.000,1.541},
				{6091.252,4023.421,10.000,1.779},
				{6504.627,4133.748,10.000,2.043},
				{6928.535,4239.081,10.000,2.336},
				{7362.454,4339.194,10.000,2.659},
				{7805.840,4433.854,10.000,3.016},
				{8258.122,4522.820,10.000,3.407},
				{8718.706,4605.847,10.000,3.836},
				{9186.975,4682.685,10.000,4.305},
				{9662.284,4753.089,10.000,4.816},
				{10143.966,4816.823,10.000,5.372},
				{10631.333,4873.669,10.000,5.977},
				{11123.677,4923.441,10.000,6.631},
				{11620.276,4965.993,10.000,7.339},
				{12120.401,5001.245,10.000,8.104},
				{12623.321,5029.202,10.000,8.927},
				{13128.014,5046.933,10.000,9.812},
				{13633.212,5051.976,10.000,10.760},
				{14137.742,5045.299,10.000,11.772},
				{14640.550,5028.082,10.000,12.849},
				{15140.723,5001.736,10.000,13.990},
				{15637.514,4967.908,10.000,15.196},
				{16130.363,4928.483,10.000,16.465},
				{16618.919,4885.564,10.000,17.795},
				{17103.063,4841.445,10.000,19.182},
				{17582.918,4798.549,10.000,20.623},
				{18058.854,4759.362,10.000,22.113},
				{18531.488,4726.335,10.000,23.646},
				{19001.666,4701.782,10.000,25.215},
				{19470.442,4687.762,10.000,26.812},
				{19939.039,4685.965,10.000,28.430},
				{20408.801,4697.620,10.000,30.058},
				{20881.142,4723.410,10.000,31.689},
				{21357.485,4763.433,10.000,33.312},
				{21839.205,4817.197,10.000,34.920},
				{22327.570,4883.646,10.000,36.503},
				{22823.943,4963.737,10.000,38.054},
				{23329.771,5058.275,10.000,39.569},
				{23846.321,5165.505,10.000,41.040},
				{24374.679,5283.580,10.000,42.466},
				{24915.743,5410.637,10.000,43.841},
				{25470.229,5544.856,10.000,45.163},
				{26038.680,5684.515,10.000,46.430},
				{26621.483,5828.026,10.000,47.640},
				{27218.879,5973.964,10.000,48.793},
				{27830.987,6121.078,10.000,49.888},
				{28457.816,6268.298,10.000,50.924},
				{29099.289,6414.731,10.000,51.903},
				{29755.255,6559.653,10.000,52.825},
				{30425.504,6702.494,10.000,53.690},
				{31109.787,6842.825,10.000,54.500},
				{31807.821,6980.341,10.000,55.255},
				{32519.305,7114.845,10.000,55.958},
				{33243.928,7246.233,10.000,56.609},
				{33981.376,7374.476,10.000,57.209},
				{34731.337,7499.614,10.000,57.759},
				{35493.511,7621.737,10.000,58.262},
				{36267.609,7740.980,10.000,58.717},
				{37053.360,7857.511,10.000,59.126},
				{37850.513,7971.529,10.000,59.490},
				{38658.838,8083.251,10.000,59.811},
				{39478.129,8192.912,10.000,60.088},
				{40308.206,8300.761,10.000,60.322},
				{41148.911,8407.054,10.000,60.514},
				{42000.117,8512.056,10.000,60.665},
				{42861.720,8616.037,10.000,60.775},
				{43733.647,8719.269,10.000,60.844},
				{44615.850,8822.028,10.000,60.873},
				{45508.309,8924.590,10.000,60.861},
				{46411.032,9027.234,10.000,60.809},
				{47324.056,9130.236,10.000,60.716},
				{48247.443,9233.872,10.000,60.582},
				{49181.285,9338.416,10.000,60.407},
				{50125.698,9444.139,10.000,60.190},
				{51080.829,9551.309,10.000,59.930},
				{52046.848,9660.185,10.000,59.628},
				{53023.950,9771.020,10.000,59.282},
				{54012.355,9884.054,10.000,58.891},
				{55012.306,9999.513,10.000,58.455},
				{56024.067,10117.603,10.000,57.973},
				{57047.917,10238.506,10.000,57.442},
				{58084.155,10362.372,10.000,56.863},
				{59133.086,10489.311,10.000,56.233},
				{60195.024,10619.383,10.000,55.552},
				{61270.283,10752.590,10.000,54.819},
				{62359.169,10888.862,10.000,54.031},
				{63461.973,11028.040,10.000,53.189},
				{64578.960,11169.866,10.000,52.291},
				{65710.356,11313.965,10.000,51.336},
				{66856.339,11459.827,10.000,50.323},
				{68017.018,11606.793,10.000,49.253},
				{69192.422,11754.041,10.000,48.124},
				{70382.480,11900.576,10.000,46.937},
				{71587.002,12045.222,10.000,45.693},
				{72805.664,12186.624,10.000,44.394},
				{74037.990,12323.260,10.000,43.040},
				{75283.337,12453.461,10.000,41.635},
				{76540.881,12575.442,10.000,40.182},
				{77809.616,12687.350,10.000,38.685},
				{79088.348,12787.326,10.000,37.148},
				{80375.705,12873.568,10.000,35.577},
				{81669.765,12940.601,10.000,33.979},
				{82967.806,12980.408,10.000,32.361},
				{84266.713,12989.071,10.000,30.733},
				{85563.299,12965.858,10.000,29.102},
				{86854.361,12910.622,10.000,27.480},
				{88136.742,12823.809,10.000,25.873},
				{89407.385,12706.426,10.000,24.292},
				{90663.383,12559.982,10.000,22.743},
				{91902.022,12386.397,10.000,21.235},
				{93120.811,12187.889,10.000,19.773},
				{94317.498,11966.864,10.000,18.363},
				{95490.078,11725.805,10.000,17.009},
				{96636.795,11467.168,10.000,15.715},
				{97756.126,11193.307,10.000,14.483},
				{98846.766,10906.408,10.000,13.315},
				{99907.611,10608.447,10.000,12.212},
				{100937.728,10301.171,10.000,11.173},
				{101936.337,9986.087,10.000,10.199},
				{102902.784,9664.467,10.000,9.288},
				{103836.520,9337.365,10.000,8.439},
				{104737.417,9008.969,10.000,7.650},
				{105605.960,8685.431,10.000,6.919},
				{106442.880,8369.199,10.000,6.243},
				{107248.903,8060.226,10.000,5.618},
				{108024.742,7758.391,10.000,5.042},
				{108771.094,7463.525,10.000,4.513},
				{109488.636,7175.419,10.000,4.027},
				{110178.021,6893.846,10.000,3.581},
				{110839.877,6618.562,10.000,3.175},
				{111474.809,6349.320,10.000,2.804},
				{112083.396,6085.873,10.000,2.467},
				{112666.194,5827.976,10.000,2.162},
				{113223.733,5575.394,10.000,1.886},
				{113756.523,5327.898,10.000,1.637},
				{114265.050,5085.270,10.000,1.415},
				{114749.780,4847.300,10.000,1.216},
				{115211.159,4613.788,10.000,1.038},
				{115649.613,4384.543,10.000,0.882},
				{116065.551,4159.381,10.000,0.743},
				{116459.364,3938.126,10.000,0.622},
				{116831.425,3720.606,10.000,0.517},
				{117182.090,3506.655,10.000,0.425},
				{117511.701,3296.108,10.000,0.347},
				{117820.581,3088.805,10.000,0.280},
				{118109.040,2884.583,10.000,0.224},
				{118377.368,2683.282,10.000,0.177},
				{118625.842,2484.737,10.000,0.138},
				{118854.720,2288.784,10.000,0.106},
				{119064.246,2095.255,10.000,0.081},
				{119254.643,1903.976,10.000,0.060},
				{119426.387,1717.440,10.000,0.044},
				{119580.383,1539.962,10.000,0.032},
				{119717.698,1373.149,10.000,0.023},
				{119839.372,1216.737,10.000,0.016},
				{119946.421,1070.492,10.000,0.011},
				{120039.841,934.202,10.000,0.007},
				{120120.609,807.678,10.000,0.005},
				{120189.685,690.756,10.000,0.003},
				{120248.014,583.292,10.000,0.002},
				{120296.530,485.162,10.000,0.001},
				{120336.156,396.263,10.000,0.000},
				{120367.807,316.507,10.000,0.000},
				{120392.390,245.825,10.000,0.000},
				{120410.806,184.160,10.000,0.000},
				{120423.953,131.470,10.000,0.000},
				{120432.725,87.723,10.000,0.000},
				{120438.015,52.898,10.000,0.000},
				{120440.713,26.980,10.000,0.000},
				{120441.709,9.961,10.000,-0.000},
				{120441.893,1.838,10.000,-0.000},
				{120441.893,0.000,10.000,-0.000}		};

}