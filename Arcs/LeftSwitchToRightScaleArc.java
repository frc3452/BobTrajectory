package org.usfirst.frc.team319.arcs;

import org.usfirst.frc.team319.models.SrxMotionProfile;
import org.usfirst.frc.team319.models.SrxTrajectory;

public class LeftSwitchToRightScaleArc extends SrxTrajectory{
	
	// WAYPOINTS:
	// (X,Y,degrees)
	// (9.00,18.03,-89.99)
	// (9.00,13.03,-89.99)
	// (9.00,9.03,-89.99)
	// (13.00,5.03,0.00)
	// (17.00,5.03,0.00)
	
    public LeftSwitchToRightScaleArc() {
		super();
		this.highGear = true;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}

	
    public LeftSwitchToRightScaleArc(boolean flipped) {
		super();
		this.highGear = true;
		this.flipped = flipped;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}

	public boolean highGear = true;

	double[][] centerPoints = {
				{0.562,11.250,10.000,-89.990},
				{2.812,33.749,10.000,-89.990},
				{7.875,67.498,10.000,-89.990},
				{16.875,112.497,10.000,-89.990},
				{30.937,168.746,10.000,-89.990},
				{51.186,236.244,10.000,-89.990},
				{78.748,314.992,10.000,-89.990},
				{114.747,404.990,10.000,-89.990},
				{160.308,506.237,10.000,-89.990},
				{216.557,618.734,10.000,-89.990},
				{284.618,742.481,10.000,-89.990},
				{365.616,877.477,10.000,-89.990},
				{460.676,1023.724,10.000,-89.990},
				{570.923,1181.220,10.000,-89.990},
				{696.920,1338.715,10.000,-89.990},
				{838.666,1496.211,10.000,-89.990},
				{996.162,1653.707,10.000,-89.990},
				{1169.407,1811.203,10.000,-89.990},
				{1358.402,1968.699,10.000,-89.990},
				{1563.147,2126.195,10.000,-89.990},
				{1783.642,2283.691,10.000,-89.990},
				{2019.885,2441.187,10.000,-89.990},
				{2271.879,2598.683,10.000,-89.990},
				{2539.622,2756.179,10.000,-89.990},
				{2823.115,2913.675,10.000,-89.990},
				{3122.357,3071.171,10.000,-89.990},
				{3437.349,3228.667,10.000,-89.990},
				{3768.090,3386.163,10.000,-89.990},
				{4114.581,3543.659,10.000,-89.990},
				{4476.822,3701.155,10.000,-89.991},
				{4854.812,3858.651,10.000,-89.991},
				{5248.552,4016.146,10.000,-89.991},
				{5658.042,4173.642,10.000,-89.991},
				{6083.281,4331.138,10.000,-89.991},
				{6524.269,4488.634,10.000,-89.991},
				{6981.007,4646.130,10.000,-89.991},
				{7453.495,4803.626,10.000,-89.991},
				{7941.733,4961.122,10.000,-89.992},
				{8445.720,5118.618,10.000,-89.992},
				{8965.456,5276.114,10.000,-89.992},
				{9500.943,5433.610,10.000,-89.992},
				{10052.178,5591.106,10.000,-89.993},
				{10619.164,5748.602,10.000,-89.993},
				{11201.899,5906.098,10.000,-89.993},
				{11800.383,6063.594,10.000,-89.993},
				{12414.617,6221.090,10.000,-89.994},
				{13044.601,6378.586,10.000,-89.994},
				{13690.335,6536.081,10.000,-89.994},
				{14351.817,6693.577,10.000,-89.995},
				{15029.050,6851.073,10.000,-89.995},
				{15722.032,7008.569,10.000,-89.995},
				{16430.764,7166.065,10.000,-89.996},
				{17155.245,7323.561,10.000,-89.996},
				{17895.476,7481.057,10.000,-89.997},
				{18651.457,7638.553,10.000,-89.997},
				{19423.187,7796.049,10.000,-89.997},
				{20210.666,7953.545,10.000,-89.998},
				{21013.896,8111.041,10.000,-89.998},
				{21832.875,8268.537,10.000,-89.999},
				{22667.603,8426.033,10.000,-89.999},
				{23518.081,8583.529,10.000,-90.000},
				{24384.309,8741.025,10.000,-90.000},
				{25266.286,8898.521,10.000,-90.001},
				{26164.013,9056.016,10.000,-90.001},
				{27077.489,9213.512,10.000,-90.002},
				{28006.715,9371.008,10.000,-90.002},
				{28951.691,9528.504,10.000,-90.003},
				{29912.416,9686.000,10.000,-90.003},
				{30888.891,9843.496,10.000,-90.004},
				{31881.115,10000.992,10.000,-90.004},
				{32889.089,10158.488,10.000,-90.005},
				{33912.813,10315.984,10.000,-90.005},
				{34952.286,10473.480,10.000,-90.006},
				{36007.509,10630.976,10.000,-90.006},
				{37078.481,10788.472,10.000,-90.007},
				{38165.203,10945.968,10.000,-90.007},
				{39267.675,11103.464,10.000,-90.007},
				{40385.896,11260.960,10.000,-90.008},
				{41519.867,11418.456,10.000,-90.008},
				{42669.587,11575.952,10.000,-90.008},
				{43835.057,11733.447,10.000,-90.008},
				{45016.277,11890.943,10.000,-90.008},
				{46213.246,12048.439,10.000,-90.009},
				{47425.965,12205.935,10.000,-90.009},
				{48654.433,12363.431,10.000,-90.009},
				{49898.651,12520.927,10.000,-90.009},
				{51158.618,12678.423,10.000,-90.009},
				{52434.336,12835.919,10.000,-90.009},
				{53725.802,12993.415,10.000,-90.008},
				{55033.019,13150.911,10.000,-90.008},
				{56355.984,13308.407,10.000,-90.008},
				{57694.700,13465.903,10.000,-90.008},
				{59049.165,13623.399,10.000,-90.007},
				{60419.380,13780.895,10.000,-90.007},
				{61804.781,13927.141,10.000,-90.006},
				{63204.245,14062.137,10.000,-90.006},
				{64616.646,14185.884,10.000,-90.005},
				{66040.860,14298.381,10.000,-90.005},
				{67475.760,14399.629,10.000,-90.004},
				{68920.223,14489.626,10.000,-90.003},
				{70373.123,14568.374,10.000,-90.003},
				{71833.335,14635.873,10.000,-90.002},
				{73299.735,14692.121,10.000,-90.001},
				{74771.197,14737.120,10.000,-90.000},
				{76246.597,14770.869,10.000,-89.999},
				{77724.808,14793.369,10.000,-89.998},
				{79204.708,14804.618,10.000,-89.998},
				{80685.170,14804.618,10.000,-89.997},
				{82165.631,14804.618,10.000,-89.996},
				{83646.093,14804.618,10.000,-89.995},
				{85126.555,14804.618,10.000,-89.994},
				{86607.017,14804.618,10.000,-89.993},
				{88087.479,14804.618,10.000,-89.993},
				{89567.941,14804.618,10.000,-89.992},
				{91048.402,14804.618,10.000,-89.992},
				{92528.864,14804.618,10.000,-89.991},
				{94009.326,14804.618,10.000,-89.991},
				{95489.788,14804.618,10.000,-89.990},
				{96970.250,14804.618,10.000,-89.990},
				{98450.712,14804.618,10.000,-89.990},
				{99931.173,14804.618,10.000,-89.990},
				{101411.635,14804.618,10.000,-89.990},
				{102892.097,14804.618,10.000,-89.991},
				{104372.559,14804.618,10.000,-89.991},
				{105853.021,14804.618,10.000,-89.992},
				{107333.482,14804.618,10.000,-89.993},
				{108813.944,14804.618,10.000,-89.994},
				{110294.406,14804.618,10.000,-89.995},
				{111774.868,14804.618,10.000,-89.996},
				{113255.330,14804.618,10.000,-89.997},
				{114735.792,14804.618,10.000,-89.998},
				{116216.253,14804.618,10.000,-89.999},
				{117696.715,14804.618,10.000,-90.000},
				{119177.177,14804.618,10.000,-90.001},
				{120657.639,14804.618,10.000,-90.002},
				{122138.101,14804.618,10.000,-90.003},
				{123618.563,14804.618,10.000,-90.004},
				{125099.024,14804.618,10.000,-90.005},
				{126579.486,14804.618,10.000,-90.006},
				{128059.948,14804.618,10.000,-90.006},
				{129540.410,14804.618,10.000,-90.007},
				{131020.872,14804.618,10.000,-90.008},
				{132501.334,14804.618,10.000,-90.008},
				{133981.795,14804.618,10.000,-90.008},
				{135462.257,14804.618,10.000,-90.009},
				{136942.719,14804.618,10.000,-90.009},
				{138423.181,14804.618,10.000,-90.009},
				{139903.643,14804.618,10.000,-90.009},
				{141384.104,14804.618,10.000,-90.009},
				{142864.566,14804.618,10.000,-90.008},
				{144345.028,14804.618,10.000,-90.008},
				{145825.490,14804.618,10.000,-90.007},
				{147305.952,14804.618,10.000,-90.007},
				{148786.414,14804.618,10.000,-90.006},
				{150266.875,14804.618,10.000,-90.005},
				{151747.337,14804.618,10.000,-90.005},
				{153227.799,14804.618,10.000,-90.004},
				{154708.261,14804.618,10.000,-90.003},
				{156188.723,14804.618,10.000,-90.002},
				{157669.185,14804.618,10.000,-90.001},
				{159149.646,14804.618,10.000,-90.000},
				{160630.108,14804.618,10.000,-89.999},
				{162110.570,14804.618,10.000,-89.998},
				{163591.032,14804.618,10.000,-89.996},
				{165071.494,14804.618,10.000,-89.995},
				{166551.956,14804.618,10.000,-89.994},
				{168032.417,14804.618,10.000,-89.993},
				{169512.879,14804.618,10.000,-89.993},
				{170993.341,14804.618,10.000,-89.992},
				{172473.803,14804.618,10.000,-89.991},
				{173954.265,14804.618,10.000,-89.991},
				{175434.727,14804.618,10.000,-89.990},
				{176915.188,14804.618,10.000,-89.990},
				{178395.650,14804.618,10.000,-89.986},
				{179876.112,14804.618,10.000,-89.956},
				{181356.574,14804.618,10.000,-89.897},
				{182837.036,14804.618,10.000,-89.808},
				{184317.497,14804.618,10.000,-89.690},
				{185797.959,14804.618,10.000,-89.543},
				{187278.421,14804.618,10.000,-89.367},
				{188758.883,14804.618,10.000,-89.161},
				{190239.345,14804.618,10.000,-88.925},
				{191719.807,14804.618,10.000,-88.659},
				{193200.268,14804.618,10.000,-88.361},
				{194680.730,14804.618,10.000,-88.032},
				{196161.192,14804.618,10.000,-87.670},
				{197641.654,14804.618,10.000,-87.274},
				{199122.116,14804.618,10.000,-86.843},
				{200602.578,14804.618,10.000,-86.375},
				{202083.039,14804.618,10.000,-85.870},
				{203563.501,14804.618,10.000,-85.325},
				{205043.963,14804.618,10.000,-84.739},
				{206524.425,14804.618,10.000,-84.110},
				{208004.887,14804.618,10.000,-83.435},
				{209485.349,14804.618,10.000,-82.713},
				{210965.810,14804.618,10.000,-81.940},
				{212446.272,14804.618,10.000,-81.115},
				{213926.734,14804.618,10.000,-80.235},
				{215407.196,14804.618,10.000,-79.296},
				{216887.658,14804.618,10.000,-78.295},
				{218368.120,14804.618,10.000,-77.230},
				{219848.581,14804.618,10.000,-76.097},
				{221329.043,14804.618,10.000,-74.894},
				{222809.505,14804.618,10.000,-73.616},
				{224289.967,14804.618,10.000,-72.262},
				{225770.429,14804.618,10.000,-70.828},
				{227250.890,14804.618,10.000,-69.312},
				{228731.352,14804.618,10.000,-67.713},
				{230211.814,14804.618,10.000,-66.030},
				{231692.276,14804.618,10.000,-64.263},
				{233172.738,14804.618,10.000,-62.414},
				{234653.200,14804.618,10.000,-60.485},
				{236133.661,14804.618,10.000,-58.480},
				{237614.123,14804.618,10.000,-56.406},
				{239094.585,14804.618,10.000,-54.270},
				{240575.047,14804.618,10.000,-52.081},
				{242055.509,14804.618,10.000,-49.851},
				{243535.971,14804.618,10.000,-47.592},
				{245016.432,14804.618,10.000,-45.317},
				{246496.894,14804.618,10.000,-43.040},
				{247977.356,14804.618,10.000,-40.774},
				{249457.818,14804.618,10.000,-38.535},
				{250938.280,14804.618,10.000,-36.333},
				{252418.742,14804.618,10.000,-34.181},
				{253899.203,14804.618,10.000,-32.088},
				{255379.665,14804.618,10.000,-30.063},
				{256860.127,14804.618,10.000,-28.112},
				{258340.589,14804.618,10.000,-26.240},
				{259821.051,14804.618,10.000,-24.449},
				{261301.512,14804.618,10.000,-22.743},
				{262781.974,14804.618,10.000,-21.120},
				{264262.436,14804.618,10.000,-19.581},
				{265742.898,14804.618,10.000,-18.124},
				{267223.360,14804.618,10.000,-16.747},
				{268703.822,14804.618,10.000,-15.448},
				{270184.283,14804.618,10.000,-14.224},
				{271664.745,14804.618,10.000,-13.071},
				{273145.207,14804.618,10.000,-11.987},
				{274625.669,14804.618,10.000,-10.969},
				{276106.131,14804.618,10.000,-10.013},
				{277586.593,14804.618,10.000,-9.116},
				{279067.054,14804.618,10.000,-8.275},
				{280547.516,14804.618,10.000,-7.488},
				{282027.978,14804.618,10.000,-6.752},
				{283508.440,14804.618,10.000,-6.064},
				{284988.902,14804.618,10.000,-5.422},
				{286469.364,14804.618,10.000,-4.824},
				{287949.825,14804.618,10.000,-4.268},
				{289430.287,14804.618,10.000,-3.752},
				{290910.749,14804.618,10.000,-3.274},
				{292391.211,14804.618,10.000,-2.832},
				{293871.673,14804.618,10.000,-2.427},
				{295352.135,14804.618,10.000,-2.055},
				{296832.596,14804.618,10.000,-1.717},
				{298313.058,14804.618,10.000,-1.410},
				{299793.520,14804.618,10.000,-1.135},
				{301273.982,14804.618,10.000,-0.891},
				{302754.444,14804.618,10.000,-0.677},
				{304234.905,14804.618,10.000,-0.492},
				{305715.367,14804.618,10.000,-0.337},
				{307195.829,14804.618,10.000,-0.211},
				{308676.291,14804.618,10.000,-0.115},
				{310156.753,14804.618,10.000,-0.047},
				{311637.215,14804.618,10.000,-0.009},
				{313117.454,14800.177,10.000,0.000},
				{314596.688,14784.487,10.000,0.000},
				{316073.789,14757.546,10.000,0.000},
				{317547.634,14719.356,10.000,0.000},
				{319017.098,14669.916,10.000,0.000},
				{320481.055,14609.227,10.000,0.000},
				{321938.381,14537.287,10.000,0.000},
				{323387.950,14454.098,10.000,0.000},
				{324828.638,14359.660,10.000,0.000},
				{326259.320,14253.971,10.000,0.000},
				{327678.870,14137.033,10.000,0.000},
				{329086.164,14008.845,10.000,0.000},
				{330480.076,13869.408,10.000,0.000},
				{331859.483,13718.721,10.000,0.000},
				{333223.480,13561.225,10.000,0.000},
				{334571.728,13403.729,10.000,0.000},
				{335904.226,13246.233,10.000,0.000},
				{337220.974,13088.737,10.000,0.000},
				{338521.973,12931.241,10.000,0.000},
				{339807.222,12773.745,10.000,0.000},
				{341076.722,12616.249,10.000,0.000},
				{342330.472,12458.753,10.000,0.000},
				{343568.473,12301.257,10.000,0.000},
				{344790.724,12143.761,10.000,0.000},
				{345997.225,11986.265,10.000,0.000},
				{347187.977,11828.769,10.000,0.000},
				{348362.979,11671.273,10.000,0.000},
				{349522.232,11513.778,10.000,0.000},
				{350665.734,11356.282,10.000,0.000},
				{351793.488,11198.786,10.000,0.000},
				{352905.492,11041.290,10.000,0.000},
				{354001.746,10883.794,10.000,0.000},
				{355082.250,10726.298,10.000,0.000},
				{356147.005,10568.802,10.000,0.000},
				{357196.011,10411.306,10.000,0.000},
				{358229.267,10253.810,10.000,0.000},
				{359246.773,10096.314,10.000,0.000},
				{360248.529,9938.818,10.000,0.000},
				{361234.536,9781.322,10.000,0.000},
				{362204.794,9623.826,10.000,0.000},
				{363159.302,9466.330,10.000,0.000},
				{364098.060,9308.834,10.000,0.000},
				{365021.069,9151.338,10.000,0.000},
				{365928.328,8993.843,10.000,0.000},
				{366819.837,8836.347,10.000,0.000},
				{367695.597,8678.851,10.000,0.000},
				{368555.607,8521.355,10.000,0.000},
				{369399.868,8363.859,10.000,0.000},
				{370228.379,8206.363,10.000,0.000},
				{371041.140,8048.867,10.000,0.000},
				{371838.152,7891.371,10.000,0.000},
				{372619.415,7733.875,10.000,0.000},
				{373384.927,7576.379,10.000,0.000},
				{374134.690,7418.883,10.000,0.000},
				{374868.704,7261.387,10.000,0.000},
				{375586.968,7103.891,10.000,0.000},
				{376289.482,6946.395,10.000,0.000},
				{376976.247,6788.899,10.000,0.000},
				{377647.262,6631.403,10.000,0.000},
				{378302.528,6473.907,10.000,0.000},
				{378942.043,6316.412,10.000,0.000},
				{379565.810,6158.916,10.000,0.000},
				{380173.827,6001.420,10.000,0.000},
				{380766.094,5843.924,10.000,0.000},
				{381342.611,5686.428,10.000,0.000},
				{381903.379,5528.932,10.000,0.000},
				{382448.398,5371.436,10.000,0.000},
				{382977.667,5213.940,10.000,0.000},
				{383491.186,5056.444,10.000,0.000},
				{383988.955,4898.948,10.000,0.000},
				{384470.975,4741.452,10.000,0.000},
				{384937.246,4583.956,10.000,0.000},
				{385387.767,4426.460,10.000,0.000},
				{385822.538,4268.964,10.000,0.000},
				{386241.559,4111.468,10.000,0.000},
				{386644.831,3953.972,10.000,0.000},
				{387032.354,3796.477,10.000,0.000},
				{387404.127,3638.981,10.000,0.000},
				{387760.150,3481.485,10.000,0.000},
				{388100.424,3323.989,10.000,0.000},
				{388424.948,3166.493,10.000,0.000},
				{388733.722,3008.997,10.000,0.000},
				{389026.747,2851.501,10.000,0.000},
				{389304.022,2694.005,10.000,0.000},
				{389565.548,2536.509,10.000,0.000},
				{389811.324,2379.013,10.000,0.000},
				{390041.351,2221.517,10.000,0.000},
				{390255.628,2064.021,10.000,0.000},
				{390454.155,1906.525,10.000,0.000},
				{390636.933,1749.029,10.000,0.000},
				{390803.961,1591.533,10.000,0.000},
				{390955.239,1434.037,10.000,0.000},
				{391090.768,1276.541,10.000,0.000},
				{391210.548,1119.046,10.000,0.000},
				{391314.800,965.991,10.000,0.000},
				{391404.308,824.185,10.000,0.000},
				{391480.199,693.630,10.000,0.000},
				{391543.597,574.324,10.000,0.000},
				{391595.626,466.268,10.000,0.000},
				{391637.413,369.462,10.000,0.000},
				{391670.081,283.905,10.000,0.000},
				{391694.756,209.598,10.000,0.000},
				{391712.563,146.541,10.000,0.000},
				{391724.627,94.733,10.000,0.000},
				{391732.072,54.175,10.000,0.000},
				{391736.024,24.867,10.000,0.000},
				{391737.608,6.809,10.000,0.000},
				{391737.949,0.000,10.000,0.000},
				{391737.949,0.000,10.000,0.000}		};

}