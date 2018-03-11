package org.usfirst.frc.team319.arcs;

import org.usfirst.frc.team319.models.SrxMotionProfile;
import org.usfirst.frc.team319.models.SrxTrajectory;

public class TestSTurnAutoArc extends SrxTrajectory{
	
	// WAYPOINTS:
	// (X,Y,degrees)
	// (0.00,0.00,0.00)
	// (8.38,5.50,0.00)
	
    public TestSTurnAutoArc() {
		super();
		this.highGear = true;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}

	
    public TestSTurnAutoArc(boolean flipped) {
		super();
		this.highGear = true;
		this.flipped = flipped;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}

	public boolean highGear = true;

	double[][] centerPoints = {
				{0.445,8.895,10.000,0.000},
				{2.224,17.789,10.000,0.000},
				{6.226,40.023,10.000,0.000},
				{13.341,71.147,10.000,0.000},
				{24.456,111.155,10.000,0.000},
				{40.460,160.035,10.000,0.000},
				{62.237,217.774,10.000,0.000},
				{90.672,284.349,10.000,0.000},
				{126.645,359.733,10.000,0.000},
				{171.034,443.889,10.000,0.001},
				{224.711,536.773,10.000,0.001},
				{288.544,638.330,10.000,0.002},
				{363.394,748.496,10.000,0.003},
				{450.113,867.193,10.000,0.005},
				{549.547,994.334,10.000,0.008},
				{662.528,1129.818,10.000,0.011},
				{789.881,1273.529,10.000,0.016},
				{932.415,1425.340,10.000,0.022},
				{1090.926,1585.106,10.000,0.030},
				{1266.193,1752.669,10.000,0.040},
				{1458.542,1923.488,10.000,0.053},
				{1667.848,2093.065,10.000,0.070},
				{1893.978,2261.297,10.000,0.090},
				{2136.786,2428.086,10.000,0.115},
				{2396.120,2593.332,10.000,0.144},
				{2671.814,2756.941,10.000,0.179},
				{2963.696,2918.819,10.000,0.221},
				{3271.583,3078.870,10.000,0.270},
				{3595.283,3237.004,10.000,0.326},
				{3934.596,3393.127,10.000,0.391},
				{4289.311,3547.148,10.000,0.465},
				{4659.208,3698.973,10.000,0.550},
				{5044.059,3848.508,10.000,0.646},
				{5443.624,3995.657,10.000,0.754},
				{5857.656,4140.320,10.000,0.875},
				{6285.896,4282.397,10.000,1.011},
				{6728.074,4421.782,10.000,1.161},
				{7183.911,4558.365,10.000,1.328},
				{7653.114,4692.032,10.000,1.512},
				{8135.380,4822.663,10.000,1.715},
				{8630.394,4950.132,10.000,1.938},
				{9137.824,5074.308,10.000,2.182},
				{9657.330,5195.052,10.000,2.448},
				{10188.552,5312.219,10.000,2.739},
				{10731.117,5425.659,10.000,3.055},
				{11284.639,5535.215,10.000,3.398},
				{11848.712,5640.725,10.000,3.769},
				{12422.914,5742.025,10.000,4.171},
				{13006.809,5838.948,10.000,4.605},
				{13599.941,5931.327,10.000,5.072},
				{14201.476,6015.349,10.000,5.575},
				{14810.217,6087.402,10.000,6.114},
				{15424.984,6147.671,10.000,6.691},
				{16044.622,6196.381,10.000,7.307},
				{16668.002,6233.805,10.000,7.962},
				{17294.029,6260.267,10.000,8.658},
				{17921.644,6276.152,10.000,9.395},
				{18549.835,6281.909,10.000,10.173},
				{19177.641,6278.057,10.000,10.993},
				{19804.160,6265.192,10.000,11.856},
				{20428.558,6243.984,10.000,12.760},
				{21050.076,6215.181,10.000,13.705},
				{21668.037,6179.608,10.000,14.691},
				{22281.853,6138.155,10.000,15.715},
				{22891.030,6091.775,10.000,16.778},
				{23495.177,6041.464,10.000,17.876},
				{24094.001,5988.246,10.000,19.007},
				{24687.316,5933.146,10.000,20.169},
				{25275.033,5877.171,10.000,21.358},
				{25857.160,5821.273,10.000,22.571},
				{26434.080,5769.196,10.000,23.805},
				{27006.527,5724.470,10.000,25.056},
				{27575.287,5687.607,10.000,26.322},
				{28141.192,5659.041,10.000,27.599},
				{28705.103,5639.115,10.000,28.884},
				{29267.910,5628.064,10.000,30.173},
				{29830.511,5626.015,10.000,31.463},
				{30393.808,5632.972,10.000,32.751},
				{30958.691,5648.824,10.000,34.032},
				{31526.025,5673.346,10.000,35.303},
				{32096.645,5706.203,10.000,36.562},
				{32671.342,5746.965,10.000,37.804},
				{33250.854,5795.120,10.000,39.027},
				{33835.863,5850.089,10.000,40.229},
				{34426.987,5911.243,10.000,41.407},
				{35024.779,5977.919,10.000,42.558},
				{35629.723,6049.439,10.000,43.681},
				{36242.235,6125.124,10.000,44.774},
				{36862.666,6204.307,10.000,45.836},
				{37491.301,6286.346,10.000,46.865},
				{38128.364,6370.631,10.000,47.861},
				{38774.023,6456.596,10.000,48.824},
				{39428.395,6543.720,10.000,49.752},
				{40091.548,6631.530,10.000,50.645},
				{40763.509,6719.605,10.000,51.503},
				{41444.266,6807.576,10.000,52.327},
				{42133.779,6895.122,10.000,53.116},
				{42831.976,6981.970,10.000,53.871},
				{43538.765,7067.892,10.000,54.592},
				{44254.035,7152.701,10.000,55.280},
				{44977.660,7236.250,10.000,55.934},
				{45709.502,7318.424,10.000,56.556},
				{46449.416,7399.140,10.000,57.146},
				{47197.251,7478.343,10.000,57.705},
				{47952.851,7556.002,10.000,58.234},
				{48716.062,7632.107,10.000,58.732},
				{49486.728,7706.664,10.000,59.201},
				{50264.698,7779.699,10.000,59.641},
				{51049.823,7851.247,10.000,60.053},
				{51841.958,7921.357,10.000,60.437},
				{52640.967,7990.087,10.000,60.794},
				{53446.717,8057.502,10.000,61.124},
				{54259.085,8123.674,10.000,61.429},
				{55077.953,8188.680,10.000,61.707},
				{55903.213,8252.601,10.000,61.961},
				{56734.765,8315.522,10.000,62.189},
				{57572.518,8377.532,10.000,62.393},
				{58416.390,8438.718,10.000,62.573},
				{59266.307,8499.174,10.000,62.729},
				{60122.206,8558.991,10.000,62.862},
				{60984.033,8618.263,10.000,62.971},
				{61851.741,8677.084,10.000,63.056},
				{62725.296,8735.549,10.000,63.119},
				{63604.671,8793.753,10.000,63.159},
				{64489.850,8851.791,10.000,63.176},
				{65380.826,8909.758,10.000,63.170},
				{66277.601,8967.751,10.000,63.141},
				{67180.188,9025.864,10.000,63.089},
				{68088.607,9084.192,10.000,63.014},
				{69002.890,9142.832,10.000,62.916},
				{69923.078,9201.877,10.000,62.795},
				{70849.220,9261.423,10.000,62.650},
				{71781.376,9321.563,10.000,62.481},
				{72719.615,9382.390,10.000,62.289},
				{73664.015,9443.996,10.000,62.072},
				{74614.662,9506.472,10.000,61.830},
				{75571.653,9569.906,10.000,61.564},
				{76535.091,9634.384,10.000,61.272},
				{77505.090,9699.990,10.000,60.954},
				{78481.771,9766.803,10.000,60.609},
				{79465.260,9834.897,10.000,60.238},
				{80455.694,9904.341,10.000,59.840},
				{81453.214,9975.197,10.000,59.413},
				{82457.966,10047.516,10.000,58.958},
				{83470.100,10121.342,10.000,58.474},
				{84489.770,10196.705,10.000,57.960},
				{85517.132,10273.620,10.000,57.415},
				{86552.341,10352.087,10.000,56.840},
				{87595.549,10432.083,10.000,56.233},
				{88646.906,10513.566,10.000,55.594},
				{89706.553,10596.466,10.000,54.922},
				{90774.621,10680.685,10.000,54.217},
				{91851.230,10766.089,10.000,53.478},
				{92936.481,10852.513,10.000,52.705},
				{94030.456,10939.747,10.000,51.898},
				{95133.210,11027.541,10.000,51.056},
				{96244.770,11115.596,10.000,50.179},
				{97365.126,11203.567,10.000,49.268},
				{98494.232,11291.057,10.000,48.321},
				{99631.994,11377.618,10.000,47.341},
				{100778.269,11462.752,10.000,46.327},
				{101932.860,11545.913,10.000,45.281},
				{103095.512,11626.512,10.000,44.202},
				{104265.904,11703.920,10.000,43.093},
				{105443.652,11777.483,10.000,41.955},
				{106628.304,11846.524,10.000,40.790},
				{107819.341,11910.364,10.000,39.599},
				{109016.174,11968.331,10.000,38.386},
				{110218.152,12019.779,10.000,37.152},
				{111424.562,12064.101,10.000,35.901},
				{112634.637,12100.754,10.000,34.635},
				{113847.564,12129.265,10.000,33.358},
				{115062.489,12149.254,10.000,32.074},
				{116278.534,12160.444,10.000,30.784},
				{117494.800,12162.666,10.000,29.494},
				{118710.387,12155.872,10.000,28.207},
				{119924.401,12140.131,10.000,26.926},
				{121135.963,12115.627,10.000,25.654},
				{122344.229,12082.653,10.000,24.395},
				{123548.389,12041.602,10.000,23.153},
				{124747.684,11992.949,10.000,21.930},
				{125941.095,11934.109,10.000,20.729},
				{127127.078,11859.829,10.000,19.554},
				{128303.895,11768.180,10.000,18.408},
				{129469.910,11660.145,10.000,17.294},
				{130623.584,11536.743,10.000,16.215},
				{131763.485,11399.002,10.000,15.172},
				{132888.278,11247.933,10.000,14.168},
				{133996.729,11084.510,10.000,13.203},
				{135087.694,10909.654,10.000,12.280},
				{136160.116,10724.222,10.000,11.398},
				{137213.016,10528.997,10.000,10.558},
				{138245.485,10324.687,10.000,9.759},
				{139256.677,10111.923,10.000,9.003},
				{140245.803,9891.258,10.000,8.288},
				{141212.120,9663.175,10.000,7.614},
				{142154.929,9428.089,10.000,6.979},
				{143073.564,9186.351,10.000,6.384},
				{143967.390,8938.258,10.000,5.827},
				{144835.796,8684.057,10.000,5.307},
				{145678.191,8423.951,10.000,4.823},
				{146494.273,8160.816,10.000,4.374},
				{147284.255,7899.827,10.000,3.957},
				{148048.585,7643.293,10.000,3.571},
				{148787.690,7391.049,10.000,3.215},
				{149501.982,7142.927,10.000,2.886},
				{150191.859,6898.765,10.000,2.584},
				{150857.699,6658.403,10.000,2.306},
				{151499.868,6421.689,10.000,2.052},
				{152118.715,6188.473,10.000,1.819},
				{152714.577,5958.616,10.000,1.607},
				{153287.775,5731.982,10.000,1.414},
				{153838.620,5508.443,10.000,1.239},
				{154367.407,5287.876,10.000,1.081},
				{154874.423,5070.164,10.000,0.938},
				{155359.943,4855.195,10.000,0.810},
				{155824.229,4642.863,10.000,0.696},
				{156267.536,4433.063,10.000,0.595},
				{156690.105,4225.695,10.000,0.505},
				{157092.171,4020.663,10.000,0.425},
				{157473.959,3817.872,10.000,0.356},
				{157835.681,3617.227,10.000,0.296},
				{158177.545,3418.636,10.000,0.243},
				{158499.745,3222.006,10.000,0.199},
				{158802.470,3027.246,10.000,0.160},
				{159085.896,2834.262,10.000,0.128},
				{159350.192,2642.960,10.000,0.101},
				{159595.517,2453.246,10.000,0.079},
				{159822.019,2265.022,10.000,0.061},
				{160029.838,2078.190,10.000,0.046},
				{160219.103,1892.649,10.000,0.034},
				{160390.169,1710.666,10.000,0.025},
				{160543.831,1536.614,10.000,0.018},
				{160681.078,1372.470,10.000,0.013},
				{160802.886,1218.083,10.000,0.009},
				{160910.218,1073.317,10.000,0.006},
				{161004.023,938.051,10.000,0.004},
				{161085.241,812.177,10.000,0.003},
				{161154.801,695.601,10.000,0.002},
				{161213.625,588.241,10.000,0.001},
				{161262.627,490.027,10.000,0.001},
				{161302.717,400.899,10.000,0.000},
				{161334.798,320.808,10.000,0.000},
				{161359.770,249.714,10.000,0.000},
				{161378.528,187.585,10.000,0.000},
				{161391.968,134.397,10.000,0.000},
				{161400.981,90.132,10.000,0.000},
				{161406.459,54.776,10.000,0.000},
				{161409.291,28.323,10.000,0.000},
				{161410.368,10.767,10.000,-0.000},
				{161410.578,2.107,10.000,-0.000},
				{161410.578,0.000,10.000,-0.000}		};

}