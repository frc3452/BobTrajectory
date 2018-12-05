package frc.arcs;

import com.team319.follower.SrxMotionProfile;
import com.team319.follower.SrxTrajectory;

public class Baseline_MiddleArc extends SrxTrajectory {
	
	// WAYPOINTS:
	// (X,Y,degrees)
	// (1.42,13.37,0.00)
	// (6.42,15.37,89.99)
	// (3.42,18.37,179.98)
	// (10.00,17.64,179.99)
	
    public Baseline_MiddleArc() {
		super();
		this.highGear = true;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}

	
    public Baseline_MiddleArc(boolean flipped) {
		super();
		this.highGear = true;
		this.flipped = flipped;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}

	public boolean highGear = true;

	double[][] centerPoints = {
				{0.000,0.000,10.000,0.000},
				{2.086,20.861,10.000,0.000},
				{6.258,41.722,10.000,0.000},
				{12.516,62.582,10.000,0.000},
				{20.861,83.443,10.000,0.000},
				{31.291,104.304,10.000,0.000},
				{43.808,125.165,10.000,-0.010},
				{58.410,146.025,10.000,-0.020},
				{75.099,166.886,10.000,-0.030},
				{93.873,187.747,10.000,-0.040},
				{114.734,208.608,10.000,-0.060},
				{137.681,229.468,10.000,-0.090},
				{162.714,250.329,10.000,-0.130},
				{189.833,271.190,10.000,-0.170},
				{219.038,292.051,10.000,-0.230},
				{250.329,312.911,10.000,-0.290},
				{283.706,333.772,10.000,-0.380},
				{319.170,354.633,10.000,-0.470},
				{356.719,375.494,10.000,-0.590},
				{396.354,396.354,10.000,-0.720},
				{438.076,417.215,10.000,-0.870},
				{481.883,438.076,10.000,-1.040},
				{527.777,458.937,10.000,-1.230},
				{575.757,479.797,10.000,-1.450},
				{625.823,500.658,10.000,-1.700},
				{677.975,521.519,10.000,-1.970},
				{732.213,542.380,10.000,-2.260},
				{788.537,563.240,10.000,-2.590},
				{846.947,584.101,10.000,-2.940},
				{907.443,604.962,10.000,-3.320},
				{970.025,625.823,10.000,-3.740},
				{1034.694,646.683,10.000,-4.180},
				{1101.448,667.544,10.000,-4.650},
				{1170.288,688.405,10.000,-5.140},
				{1241.215,709.266,10.000,-5.670},
				{1314.228,730.126,10.000,-6.220},
				{1389.326,750.987,10.000,-6.800},
				{1466.511,771.848,10.000,-7.400},
				{1544.739,782.278,10.000,-8.020},
				{1622.967,782.278,10.000,-8.640},
				{1701.195,782.278,10.000,-9.260},
				{1779.423,782.278,10.000,-9.880},
				{1857.650,782.278,10.000,-10.500},
				{1935.878,782.278,10.000,-11.110},
				{2014.106,782.278,10.000,-11.730},
				{2092.334,782.278,10.000,-12.330},
				{2170.562,782.278,10.000,-12.930},
				{2248.790,782.278,10.000,-13.520},
				{2327.017,782.278,10.000,-14.100},
				{2405.245,782.278,10.000,-14.670},
				{2483.473,782.278,10.000,-15.230},
				{2561.701,782.278,10.000,-15.780},
				{2639.929,782.278,10.000,-16.310},
				{2718.157,782.278,10.000,-16.840},
				{2796.384,782.278,10.000,-17.350},
				{2874.612,782.278,10.000,-17.850},
				{2952.840,782.278,10.000,-18.340},
				{3031.068,782.278,10.000,-18.810},
				{3109.296,782.278,10.000,-19.280},
				{3187.524,782.278,10.000,-19.720},
				{3265.751,782.278,10.000,-20.160},
				{3343.979,782.278,10.000,-20.580},
				{3422.207,782.278,10.000,-20.990},
				{3500.435,782.278,10.000,-21.390},
				{3578.663,782.278,10.000,-21.780},
				{3656.891,782.278,10.000,-22.150},
				{3735.118,782.278,10.000,-22.510},
				{3813.346,782.278,10.000,-22.860},
				{3891.574,782.278,10.000,-23.200},
				{3969.802,782.278,10.000,-23.520},
				{4048.030,782.278,10.000,-23.840},
				{4126.258,782.278,10.000,-24.140},
				{4204.486,782.278,10.000,-24.430},
				{4282.713,782.278,10.000,-24.710},
				{4360.941,782.278,10.000,-24.980},
				{4439.169,782.278,10.000,-25.240},
				{4517.397,782.278,10.000,-25.480},
				{4595.625,782.278,10.000,-25.720},
				{4673.853,782.278,10.000,-25.950},
				{4752.080,782.278,10.000,-26.170},
				{4830.308,782.278,10.000,-26.380},
				{4908.536,782.278,10.000,-26.580},
				{4986.764,782.278,10.000,-26.770},
				{5064.992,782.278,10.000,-26.950},
				{5143.220,782.278,10.000,-27.120},
				{5221.447,782.278,10.000,-27.280},
				{5299.675,782.278,10.000,-27.430},
				{5377.903,782.278,10.000,-27.570},
				{5456.131,782.278,10.000,-27.710},
				{5534.359,782.278,10.000,-27.840},
				{5612.587,782.278,10.000,-27.950},
				{5690.814,782.278,10.000,-28.060},
				{5769.042,782.278,10.000,-28.170},
				{5847.270,782.278,10.000,-28.260},
				{5925.498,782.278,10.000,-28.340},
				{6003.726,782.278,10.000,-28.420},
				{6081.954,782.278,10.000,-28.490},
				{6160.181,782.278,10.000,-28.550},
				{6238.409,782.278,10.000,-28.600},
				{6316.637,782.278,10.000,-28.640},
				{6394.865,782.278,10.000,-28.680},
				{6473.093,782.278,10.000,-28.700},
				{6551.321,782.278,10.000,-28.720},
				{6629.548,782.278,10.000,-28.730},
				{6707.776,782.278,10.000,-28.730},
				{6786.004,782.278,10.000,-28.730},
				{6864.232,782.278,10.000,-28.710},
				{6942.460,782.278,10.000,-28.690},
				{7020.688,782.278,10.000,-28.660},
				{7098.916,782.278,10.000,-28.610},
				{7177.143,782.278,10.000,-28.560},
				{7255.371,782.278,10.000,-28.500},
				{7333.599,782.278,10.000,-28.440},
				{7411.827,782.278,10.000,-28.360},
				{7490.055,782.278,10.000,-28.270},
				{7568.283,782.278,10.000,-28.170},
				{7646.510,782.278,10.000,-28.060},
				{7724.738,782.278,10.000,-27.940},
				{7802.966,782.278,10.000,-27.820},
				{7881.194,782.278,10.000,-27.670},
				{7959.422,782.278,10.000,-27.520},
				{8037.650,782.278,10.000,-27.360},
				{8115.877,782.278,10.000,-27.180},
				{8194.105,782.278,10.000,-27.000},
				{8272.333,782.278,10.000,-26.790},
				{8350.561,782.278,10.000,-26.580},
				{8428.789,782.278,10.000,-26.350},
				{8507.017,782.278,10.000,-26.110},
				{8585.244,782.278,10.000,-25.850},
				{8663.472,782.278,10.000,-25.570},
				{8741.700,782.278,10.000,-25.280},
				{8819.928,782.278,10.000,-24.980},
				{8898.156,782.278,10.000,-24.650},
				{8976.384,782.278,10.000,-24.300},
				{9054.611,782.278,10.000,-23.940},
				{9132.839,782.278,10.000,-23.550},
				{9211.067,782.278,10.000,-23.140},
				{9289.295,782.278,10.000,-22.710},
				{9367.523,782.278,10.000,-22.250},
				{9445.751,782.278,10.000,-21.760},
				{9523.978,782.278,10.000,-21.250},
				{9602.206,782.278,10.000,-20.710},
				{9680.434,782.278,10.000,-20.140},
				{9758.662,782.278,10.000,-19.530},
				{9836.890,782.278,10.000,-18.890},
				{9915.118,782.278,10.000,-18.210},
				{9993.345,782.278,10.000,-17.490},
				{10071.573,782.278,10.000,-16.720},
				{10149.801,782.278,10.000,-15.910},
				{10228.029,782.278,10.000,-15.060},
				{10306.257,782.278,10.000,-14.140},
				{10384.485,782.278,10.000,-13.180},
				{10462.713,782.278,10.000,-12.150},
				{10540.940,782.278,10.000,-11.060},
				{10619.168,782.278,10.000,-9.900},
				{10697.396,782.278,10.000,-8.670},
				{10775.624,782.278,10.000,-7.360},
				{10853.852,782.278,10.000,-5.970},
				{10932.080,782.278,10.000,-4.490},
				{11010.307,782.278,10.000,-2.930},
				{11088.535,782.278,10.000,-1.270},
				{11166.763,782.278,10.000,0.490},
				{11244.991,782.278,10.000,2.350},
				{11323.219,782.278,10.000,4.310},
				{11401.447,782.278,10.000,6.370},
				{11479.674,782.278,10.000,8.530},
				{11557.902,782.278,10.000,10.790},
				{11636.130,782.278,10.000,13.140},
				{11714.358,782.278,10.000,15.570},
				{11792.586,782.278,10.000,18.070},
				{11870.814,782.278,10.000,20.630},
				{11949.041,782.278,10.000,23.230},
				{12027.269,782.278,10.000,25.850},
				{12105.497,782.278,10.000,28.470},
				{12183.725,782.278,10.000,31.090},
				{12261.953,782.278,10.000,33.670},
				{12340.181,782.278,10.000,36.200},
				{12418.408,782.278,10.000,38.670},
				{12496.636,782.278,10.000,41.070},
				{12574.864,782.278,10.000,43.390},
				{12653.092,782.278,10.000,45.610},
				{12731.320,782.278,10.000,47.730},
				{12809.548,782.278,10.000,49.760},
				{12887.775,782.278,10.000,51.690},
				{12966.003,782.278,10.000,53.510},
				{13044.231,782.278,10.000,55.250},
				{13122.459,782.278,10.000,56.890},
				{13200.687,782.278,10.000,58.440},
				{13278.915,782.278,10.000,59.900},
				{13357.143,782.278,10.000,61.280},
				{13435.370,782.278,10.000,62.590},
				{13513.598,782.278,10.000,63.820},
				{13591.826,782.278,10.000,64.990},
				{13670.054,782.278,10.000,66.090},
				{13748.282,782.278,10.000,67.140},
				{13826.510,782.278,10.000,68.120},
				{13904.737,782.278,10.000,69.060},
				{13982.965,782.278,10.000,69.940},
				{14061.193,782.278,10.000,70.790},
				{14139.421,782.278,10.000,71.580},
				{14217.649,782.278,10.000,72.340},
				{14295.877,782.278,10.000,73.070},
				{14374.104,782.278,10.000,73.750},
				{14452.332,782.278,10.000,74.410},
				{14530.560,782.278,10.000,75.030},
				{14608.788,782.278,10.000,75.630},
				{14687.016,782.278,10.000,76.190},
				{14765.244,782.278,10.000,76.740},
				{14843.471,782.278,10.000,77.260},
				{14921.699,782.278,10.000,77.750},
				{14999.927,782.278,10.000,78.230},
				{15078.155,782.278,10.000,78.690},
				{15156.383,782.278,10.000,79.120},
				{15234.611,782.278,10.000,79.540},
				{15312.838,782.278,10.000,79.950},
				{15391.066,782.278,10.000,80.330},
				{15469.294,782.278,10.000,80.710},
				{15547.522,782.278,10.000,81.060},
				{15625.750,782.278,10.000,81.410},
				{15703.978,782.278,10.000,81.740},
				{15782.205,782.278,10.000,82.060},
				{15860.433,782.278,10.000,82.370},
				{15938.661,782.278,10.000,82.660},
				{16016.889,782.278,10.000,82.950},
				{16095.117,782.278,10.000,83.220},
				{16173.345,782.278,10.000,83.490},
				{16251.573,782.278,10.000,83.750},
				{16329.800,782.278,10.000,83.990},
				{16408.028,782.278,10.000,84.230},
				{16486.256,782.278,10.000,84.460},
				{16564.484,782.278,10.000,84.690},
				{16642.712,782.278,10.000,84.900},
				{16720.940,782.278,10.000,85.110},
				{16799.167,782.278,10.000,85.320},
				{16877.395,782.278,10.000,85.510},
				{16955.623,782.278,10.000,85.700},
				{17033.851,782.278,10.000,85.880},
				{17112.079,782.278,10.000,86.060},
				{17190.307,782.278,10.000,86.230},
				{17268.534,782.278,10.000,86.400},
				{17346.762,782.278,10.000,86.560},
				{17424.990,782.278,10.000,86.710},
				{17503.218,782.278,10.000,86.860},
				{17581.446,782.278,10.000,87.010},
				{17659.674,782.278,10.000,87.150},
				{17737.901,782.278,10.000,87.290},
				{17816.129,782.278,10.000,87.420},
				{17894.357,782.278,10.000,87.550},
				{17972.585,782.278,10.000,87.670},
				{18050.813,782.278,10.000,87.790},
				{18129.041,782.278,10.000,87.900},
				{18207.268,782.278,10.000,88.020},
				{18285.496,782.278,10.000,88.120},
				{18363.724,782.278,10.000,88.230},
				{18441.952,782.278,10.000,88.330},
				{18520.180,782.278,10.000,88.420},
				{18598.408,782.278,10.000,88.520},
				{18676.635,782.278,10.000,88.610},
				{18754.863,782.278,10.000,88.700},
				{18833.091,782.278,10.000,88.780},
				{18911.319,782.278,10.000,88.860},
				{18989.547,782.278,10.000,88.940},
				{19067.775,782.278,10.000,89.010},
				{19146.003,782.278,10.000,89.080},
				{19224.230,782.278,10.000,89.150},
				{19302.458,782.278,10.000,89.220},
				{19380.686,782.278,10.000,89.280},
				{19458.914,782.278,10.000,89.340},
				{19537.142,782.278,10.000,89.400},
				{19615.370,782.278,10.000,89.450},
				{19693.597,782.278,10.000,89.510},
				{19771.825,782.278,10.000,89.550},
				{19850.053,782.278,10.000,89.600},
				{19928.281,782.278,10.000,89.640},
				{20006.509,782.278,10.000,89.690},
				{20082.650,761.418,10.000,89.720},
				{20156.706,740.557,10.000,89.760},
				{20228.676,719.696,10.000,89.790},
				{20298.559,698.835,10.000,89.820},
				{20366.357,677.975,10.000,89.840},
				{20432.068,657.114,10.000,89.860},
				{20495.693,636.253,10.000,89.880},
				{20557.233,615.392,10.000,89.900},
				{20616.686,594.532,10.000,89.920},
				{20674.053,573.671,10.000,89.930},
				{20729.334,552.810,10.000,89.940},
				{20782.529,531.949,10.000,89.950},
				{20833.638,511.089,10.000,89.960},
				{20882.660,490.228,10.000,89.970},
				{20929.597,469.367,10.000,89.970},
				{20974.448,448.506,10.000,89.980},
				{21017.212,427.646,10.000,89.980},
				{21057.891,406.785,10.000,89.990},
				{21096.483,385.924,10.000,89.990},
				{21132.990,365.063,10.000,89.990},
				{21167.410,344.202,10.000,89.990},
				{21199.744,323.342,10.000,89.990},
				{21229.992,302.481,10.000,89.990},
				{21229.992,302.481,10.000,89.990},
				{21262.326,323.342,10.000,89.990},
				{21296.747,344.202,10.000,90.000},
				{21333.253,365.063,10.000,90.000},
				{21371.845,385.924,10.000,90.010},
				{21412.524,406.785,10.000,90.020},
				{21455.288,427.646,10.000,90.040},
				{21500.139,448.506,10.000,90.050},
				{21547.076,469.367,10.000,90.080},
				{21596.098,490.228,10.000,90.100},
				{21647.207,511.089,10.000,90.130},
				{21700.402,531.949,10.000,90.170},
				{21755.683,552.810,10.000,90.210},
				{21813.050,573.671,10.000,90.250},
				{21872.503,594.532,10.000,90.310},
				{21934.043,615.392,10.000,90.370},
				{21997.668,636.253,10.000,90.430},
				{22063.379,657.114,10.000,90.510},
				{22131.177,677.975,10.000,90.590},
				{22201.060,698.835,10.000,90.680},
				{22273.030,719.696,10.000,90.790},
				{22347.086,740.557,10.000,90.900},
				{22423.227,761.418,10.000,91.020},
				{22501.455,782.278,10.000,91.160},
				{22579.683,782.278,10.000,91.310},
				{22657.911,782.278,10.000,91.460},
				{22736.139,782.278,10.000,91.620},
				{22814.367,782.278,10.000,91.790},
				{22892.594,782.278,10.000,91.980},
				{22970.822,782.278,10.000,92.170},
				{23049.050,782.278,10.000,92.370},
				{23127.278,782.278,10.000,92.580},
				{23205.506,782.278,10.000,92.800},
				{23283.734,782.278,10.000,93.030},
				{23361.961,782.278,10.000,93.270},
				{23440.189,782.278,10.000,93.520},
				{23518.417,782.278,10.000,93.780},
				{23596.645,782.278,10.000,94.050},
				{23674.873,782.278,10.000,94.330},
				{23753.101,782.278,10.000,94.630},
				{23831.328,782.278,10.000,94.930},
				{23909.556,782.278,10.000,95.250},
				{23987.784,782.278,10.000,95.580},
				{24066.012,782.278,10.000,95.930},
				{24144.240,782.278,10.000,96.280},
				{24222.468,782.278,10.000,96.650},
				{24300.695,782.278,10.000,97.030},
				{24378.923,782.278,10.000,97.430},
				{24457.151,782.278,10.000,97.840},
				{24535.379,782.278,10.000,98.270},
				{24613.607,782.278,10.000,98.710},
				{24691.835,782.278,10.000,99.170},
				{24770.063,782.278,10.000,99.640},
				{24848.290,782.278,10.000,100.130},
				{24926.518,782.278,10.000,100.640},
				{25004.746,782.278,10.000,101.160},
				{25082.974,782.278,10.000,101.710},
				{25161.202,782.278,10.000,102.270},
				{25239.430,782.278,10.000,102.850},
				{25317.657,782.278,10.000,103.450},
				{25395.885,782.278,10.000,104.060},
				{25474.113,782.278,10.000,104.700},
				{25552.341,782.278,10.000,105.360},
				{25630.569,782.278,10.000,106.040},
				{25708.797,782.278,10.000,106.750},
				{25787.024,782.278,10.000,107.470},
				{25865.252,782.278,10.000,108.220},
				{25943.480,782.278,10.000,108.990},
				{26021.708,782.278,10.000,109.780},
				{26099.936,782.278,10.000,110.600},
				{26178.164,782.278,10.000,111.440},
				{26256.391,782.278,10.000,112.300},
				{26334.619,782.278,10.000,113.190},
				{26412.847,782.278,10.000,114.100},
				{26491.075,782.278,10.000,115.040},
				{26569.303,782.278,10.000,116.000},
				{26647.531,782.278,10.000,116.980},
				{26725.758,782.278,10.000,117.990},
				{26803.986,782.278,10.000,119.020},
				{26882.214,782.278,10.000,120.070},
				{26960.442,782.278,10.000,121.140},
				{27038.670,782.278,10.000,122.230},
				{27116.898,782.278,10.000,123.340},
				{27195.125,782.278,10.000,124.470},
				{27273.353,782.278,10.000,125.620},
				{27351.581,782.278,10.000,126.780},
				{27429.809,782.278,10.000,127.950},
				{27508.037,782.278,10.000,129.140},
				{27586.265,782.278,10.000,130.330},
				{27664.492,782.278,10.000,131.540},
				{27742.720,782.278,10.000,132.750},
				{27820.948,782.278,10.000,133.960},
				{27899.176,782.278,10.000,135.170},
				{27977.404,782.278,10.000,136.390},
				{28055.632,782.278,10.000,137.600},
				{28133.860,782.278,10.000,138.810},
				{28212.087,782.278,10.000,140.010},
				{28290.315,782.278,10.000,141.200},
				{28368.543,782.278,10.000,142.380},
				{28446.771,782.278,10.000,143.550},
				{28524.999,782.278,10.000,144.710},
				{28603.227,782.278,10.000,145.850},
				{28681.454,782.278,10.000,146.970},
				{28759.682,782.278,10.000,148.080},
				{28837.910,782.278,10.000,149.160},
				{28916.138,782.278,10.000,150.230},
				{28994.366,782.278,10.000,151.270},
				{29072.594,782.278,10.000,152.290},
				{29150.821,782.278,10.000,153.290},
				{29229.049,782.278,10.000,154.270},
				{29307.277,782.278,10.000,155.220},
				{29385.505,782.278,10.000,156.150},
				{29463.733,782.278,10.000,157.050},
				{29541.961,782.278,10.000,157.930},
				{29620.188,782.278,10.000,158.790},
				{29698.416,782.278,10.000,159.620},
				{29776.644,782.278,10.000,160.430},
				{29854.872,782.278,10.000,161.220},
				{29933.100,782.278,10.000,161.980},
				{30011.328,782.278,10.000,162.720},
				{30089.555,782.278,10.000,163.440},
				{30167.783,782.278,10.000,164.140},
				{30246.011,782.278,10.000,164.810},
				{30324.239,782.278,10.000,165.460},
				{30402.467,782.278,10.000,166.100},
				{30480.695,782.278,10.000,166.710},
				{30558.922,782.278,10.000,167.300},
				{30637.150,782.278,10.000,167.870},
				{30715.378,782.278,10.000,168.430},
				{30793.606,782.278,10.000,168.970},
				{30871.834,782.278,10.000,169.480},
				{30950.062,782.278,10.000,169.990},
				{31028.290,782.278,10.000,170.470},
				{31106.517,782.278,10.000,170.940},
				{31184.745,782.278,10.000,171.390},
				{31262.973,782.278,10.000,171.830},
				{31341.201,782.278,10.000,172.250},
				{31419.429,782.278,10.000,172.660},
				{31497.657,782.278,10.000,173.050},
				{31575.884,782.278,10.000,173.430},
				{31654.112,782.278,10.000,173.790},
				{31732.340,782.278,10.000,174.150},
				{31810.568,782.278,10.000,174.490},
				{31888.796,782.278,10.000,174.810},
				{31967.024,782.278,10.000,175.130},
				{32045.251,782.278,10.000,175.430},
				{32123.479,782.278,10.000,175.720},
				{32201.707,782.278,10.000,176.000},
				{32279.935,782.278,10.000,176.270},
				{32358.163,782.278,10.000,176.530},
				{32436.391,782.278,10.000,176.770},
				{32514.618,782.278,10.000,177.010},
				{32592.846,782.278,10.000,177.240},
				{32671.074,782.278,10.000,177.450},
				{32749.302,782.278,10.000,177.660},
				{32827.530,782.278,10.000,177.860},
				{32905.758,782.278,10.000,178.050},
				{32983.985,782.278,10.000,178.220},
				{33062.213,782.278,10.000,178.390},
				{33140.441,782.278,10.000,178.550},
				{33216.583,761.418,10.000,178.700},
				{33290.639,740.557,10.000,178.840},
				{33362.608,719.696,10.000,178.960},
				{33432.492,698.835,10.000,179.070},
				{33500.289,677.975,10.000,179.180},
				{33566.001,657.114,10.000,179.270},
				{33629.626,636.253,10.000,179.360},
				{33691.165,615.392,10.000,179.430},
				{33750.618,594.532,10.000,179.500},
				{33807.985,573.671,10.000,179.570},
				{33863.266,552.810,10.000,179.620},
				{33916.461,531.949,10.000,179.670},
				{33967.570,511.089,10.000,179.720},
				{34016.593,490.228,10.000,179.760},
				{34063.530,469.367,10.000,179.790},
				{34108.380,448.506,10.000,179.820},
				{34151.145,427.646,10.000,179.850},
				{34191.823,406.785,10.000,179.870},
				{34230.416,385.924,10.000,179.890},
				{34266.922,365.063,10.000,179.910},
				{34301.342,344.202,10.000,179.920},
				{34333.676,323.342,10.000,179.940},
				{34363.925,302.481,10.000,179.950},
				{34392.087,281.620,10.000,179.950},
				{34418.162,260.759,10.000,179.960},
				{34442.152,239.899,10.000,179.970},
				{34464.056,219.038,10.000,179.970},
				{34483.874,198.177,10.000,179.970},
				{34501.606,177.316,10.000,179.980},
				{34517.251,156.456,10.000,179.980},
				{34530.811,135.595,10.000,179.980},
				{34542.284,114.734,10.000,179.980},
				{34551.671,93.873,10.000,179.980},
				{34558.973,73.013,10.000,179.980},
				{34564.188,52.152,10.000,179.980},
				{34567.317,31.291,10.000,179.980},
				{34567.317,31.291,10.000,179.980},
				{34572.532,52.152,10.000,179.980},
				{34579.833,73.013,10.000,179.980},
				{34589.221,93.873,10.000,-0.020},
				{34600.694,114.734,10.000,-0.020},
				{34614.254,135.595,10.000,-0.020},
				{34629.899,156.456,10.000,-0.020},
				{34647.631,177.316,10.000,-0.020},
				{34667.449,198.177,10.000,-0.020},
				{34689.352,219.038,10.000,-0.030},
				{34713.342,239.899,10.000,-0.030},
				{34739.418,260.759,10.000,-0.040},
				{34767.580,281.620,10.000,-0.040},
				{34797.828,302.481,10.000,-0.050},
				{34830.162,323.342,10.000,-0.060},
				{34864.583,344.202,10.000,-0.070},
				{34901.089,365.063,10.000,-0.080},
				{34939.681,385.924,10.000,-0.100},
				{34980.360,406.785,10.000,-0.110},
				{35023.124,427.646,10.000,-0.140},
				{35067.975,448.506,10.000,-0.160},
				{35114.912,469.367,10.000,-0.190},
				{35163.935,490.228,10.000,-0.220},
				{35215.043,511.089,10.000,-0.250},
				{35268.238,531.949,10.000,-0.290},
				{35323.519,552.810,10.000,-0.330},
				{35380.886,573.671,10.000,-0.380},
				{35440.340,594.532,10.000,-0.440},
				{35501.879,615.392,10.000,-0.490},
				{35565.504,636.253,10.000,-0.560},
				{35631.215,657.114,10.000,-0.630},
				{35699.013,677.975,10.000,-0.700},
				{35768.896,698.835,10.000,-0.790},
				{35840.866,719.696,10.000,-0.870},
				{35914.922,740.557,10.000,-0.970},
				{35991.064,761.418,10.000,-1.070},
				{36069.291,782.278,10.000,-1.180},
				{36147.519,782.278,10.000,-1.290},
				{36225.747,782.278,10.000,-1.410},
				{36303.975,782.278,10.000,-1.530},
				{36382.203,782.278,10.000,-1.650},
				{36460.431,782.278,10.000,-1.780},
				{36538.658,782.278,10.000,-1.910},
				{36616.886,782.278,10.000,-2.050},
				{36695.114,782.278,10.000,-2.190},
				{36773.342,782.278,10.000,-2.330},
				{36851.570,782.278,10.000,-2.470},
				{36929.798,782.278,10.000,-2.610},
				{37008.025,782.278,10.000,-2.760},
				{37086.253,782.278,10.000,-2.910},
				{37164.481,782.278,10.000,-3.060},
				{37242.709,782.278,10.000,-3.220},
				{37320.937,782.278,10.000,-3.380},
				{37399.165,782.278,10.000,-3.530},
				{37477.392,782.278,10.000,-3.690},
				{37555.620,782.278,10.000,-3.850},
				{37633.848,782.278,10.000,-4.010},
				{37712.076,782.278,10.000,-4.180},
				{37790.304,782.278,10.000,-4.340},
				{37868.532,782.278,10.000,-4.510},
				{37946.759,782.278,10.000,-4.670},
				{38024.987,782.278,10.000,-4.840},
				{38103.215,782.278,10.000,-5.000},
				{38181.443,782.278,10.000,-5.170},
				{38259.671,782.278,10.000,-5.340},
				{38337.899,782.278,10.000,-5.500},
				{38416.127,782.278,10.000,-5.670},
				{38494.354,782.278,10.000,-5.830},
				{38572.582,782.278,10.000,-6.000},
				{38650.810,782.278,10.000,-6.170},
				{38729.038,782.278,10.000,-6.330},
				{38807.266,782.278,10.000,-6.490},
				{38885.494,782.278,10.000,-6.660},
				{38963.721,782.278,10.000,-6.820},
				{39041.949,782.278,10.000,-6.980},
				{39120.177,782.278,10.000,-7.140},
				{39198.405,782.278,10.000,-7.300},
				{39276.633,782.278,10.000,-7.460},
				{39354.861,782.278,10.000,-7.610},
				{39433.088,782.278,10.000,-7.770},
				{39511.316,782.278,10.000,-7.920},
				{39589.544,782.278,10.000,-8.070},
				{39667.772,782.278,10.000,-8.220},
				{39746.000,782.278,10.000,-8.370},
				{39824.228,782.278,10.000,-8.510},
				{39902.455,782.278,10.000,-8.650},
				{39980.683,782.278,10.000,-8.790},
				{40058.911,782.278,10.000,-8.930},
				{40137.139,782.278,10.000,-9.070},
				{40215.367,782.278,10.000,-9.200},
				{40293.595,782.278,10.000,-9.330},
				{40371.822,782.278,10.000,-9.460},
				{40450.050,782.278,10.000,-9.590},
				{40528.278,782.278,10.000,-9.710},
				{40606.506,782.278,10.000,-9.830},
				{40684.734,782.278,10.000,-9.950},
				{40762.962,782.278,10.000,-10.060},
				{40841.189,782.278,10.000,-10.170},
				{40919.417,782.278,10.000,-10.280},
				{40997.645,782.278,10.000,-10.390},
				{41075.873,782.278,10.000,-10.490},
				{41154.101,782.278,10.000,-10.590},
				{41232.329,782.278,10.000,-10.680},
				{41310.556,782.278,10.000,-10.780},
				{41388.784,782.278,10.000,-10.870},
				{41467.012,782.278,10.000,-10.950},
				{41545.240,782.278,10.000,-11.030},
				{41623.468,782.278,10.000,-11.110},
				{41701.696,782.278,10.000,-11.190},
				{41779.924,782.278,10.000,-11.260},
				{41858.151,782.278,10.000,-11.330},
				{41936.379,782.278,10.000,-11.390},
				{42014.607,782.278,10.000,-11.450},
				{42092.835,782.278,10.000,-11.510},
				{42171.063,782.278,10.000,-11.570},
				{42249.291,782.278,10.000,-11.620},
				{42327.518,782.278,10.000,-11.660},
				{42405.746,782.278,10.000,-11.700},
				{42483.974,782.278,10.000,-11.740},
				{42562.202,782.278,10.000,-11.780},
				{42640.430,782.278,10.000,-11.810},
				{42718.658,782.278,10.000,-11.840},
				{42796.885,782.278,10.000,-11.860},
				{42875.113,782.278,10.000,-11.880},
				{42953.341,782.278,10.000,-11.900},
				{43031.569,782.278,10.000,-11.910},
				{43109.797,782.278,10.000,-11.920},
				{43188.025,782.278,10.000,-11.920},
				{43266.252,782.278,10.000,-11.920},
				{43344.480,782.278,10.000,-11.920},
				{43422.708,782.278,10.000,-11.910},
				{43500.936,782.278,10.000,-11.900},
				{43579.164,782.278,10.000,-11.890},
				{43657.392,782.278,10.000,-11.870},
				{43735.619,782.278,10.000,-11.850},
				{43813.847,782.278,10.000,-11.820},
				{43892.075,782.278,10.000,-11.790},
				{43970.303,782.278,10.000,-11.760},
				{44048.531,782.278,10.000,-11.720},
				{44126.759,782.278,10.000,-11.680},
				{44204.986,782.278,10.000,-11.630},
				{44283.214,782.278,10.000,-11.580},
				{44361.442,782.278,10.000,-11.530},
				{44439.670,782.278,10.000,-11.480},
				{44517.898,782.278,10.000,-11.420},
				{44596.126,782.278,10.000,-11.350},
				{44674.354,782.278,10.000,-11.280},
				{44752.581,782.278,10.000,-11.210},
				{44830.809,782.278,10.000,-11.140},
				{44909.037,782.278,10.000,-11.060},
				{44987.265,782.278,10.000,-10.980},
				{45065.493,782.278,10.000,-10.900},
				{45143.721,782.278,10.000,-10.810},
				{45221.948,782.278,10.000,-10.720},
				{45300.176,782.278,10.000,-10.620},
				{45378.404,782.278,10.000,-10.530},
				{45456.632,782.278,10.000,-10.420},
				{45534.860,782.278,10.000,-10.320},
				{45613.088,782.278,10.000,-10.210},
				{45691.315,782.278,10.000,-10.100},
				{45769.543,782.278,10.000,-9.990},
				{45847.771,782.278,10.000,-9.870},
				{45925.999,782.278,10.000,-9.750},
				{46004.227,782.278,10.000,-9.630},
				{46082.455,782.278,10.000,-9.510},
				{46160.682,782.278,10.000,-9.380},
				{46238.910,782.278,10.000,-9.250},
				{46317.138,782.278,10.000,-9.120},
				{46395.366,782.278,10.000,-8.980},
				{46473.594,782.278,10.000,-8.840},
				{46551.822,782.278,10.000,-8.700},
				{46630.049,782.278,10.000,-8.560},
				{46708.277,782.278,10.000,-8.420},
				{46786.505,782.278,10.000,-8.270},
				{46864.733,782.278,10.000,-8.120},
				{46942.961,782.278,10.000,-7.970},
				{47021.189,782.278,10.000,-7.820},
				{47099.416,782.278,10.000,-7.670},
				{47177.644,782.278,10.000,-7.510},
				{47255.872,782.278,10.000,-7.350},
				{47334.100,782.278,10.000,-7.200},
				{47412.328,782.278,10.000,-7.040},
				{47490.556,782.278,10.000,-6.880},
				{47568.784,782.278,10.000,-6.710},
				{47647.011,782.278,10.000,-6.550},
				{47725.239,782.278,10.000,-6.390},
				{47803.467,782.278,10.000,-6.220},
				{47881.695,782.278,10.000,-6.060},
				{47959.923,782.278,10.000,-5.890},
				{48038.151,782.278,10.000,-5.730},
				{48116.378,782.278,10.000,-5.560},
				{48194.606,782.278,10.000,-5.390},
				{48272.834,782.278,10.000,-5.230},
				{48351.062,782.278,10.000,-5.060},
				{48429.290,782.278,10.000,-4.890},
				{48507.518,782.278,10.000,-4.730},
				{48585.745,782.278,10.000,-4.560},
				{48663.973,782.278,10.000,-4.400},
				{48742.201,782.278,10.000,-4.230},
				{48820.429,782.278,10.000,-4.070},
				{48898.657,782.278,10.000,-3.910},
				{48976.885,782.278,10.000,-3.750},
				{49055.112,782.278,10.000,-3.590},
				{49133.340,782.278,10.000,-3.430},
				{49211.568,782.278,10.000,-3.270},
				{49289.796,782.278,10.000,-3.120},
				{49368.024,782.278,10.000,-2.960},
				{49446.252,782.278,10.000,-2.810},
				{49524.479,782.278,10.000,-2.660},
				{49602.707,782.278,10.000,-2.520},
				{49680.935,782.278,10.000,-2.370},
				{49759.163,782.278,10.000,-2.230},
				{49837.391,782.278,10.000,-2.090},
				{49915.619,782.278,10.000,-1.960},
				{49993.846,782.278,10.000,-1.820},
				{50072.074,782.278,10.000,-1.690},
				{50150.302,782.278,10.000,-1.570},
				{50228.530,782.278,10.000,-1.450},
				{50306.758,782.278,10.000,-1.330},
				{50384.986,782.278,10.000,-1.210},
				{50463.213,782.278,10.000,-1.100},
				{50541.441,782.278,10.000,-1.000},
				{50617.583,761.418,10.000,-0.900},
				{50691.639,740.557,10.000,-0.800},
				{50763.608,719.696,10.000,-0.720},
				{50833.492,698.835,10.000,-0.640},
				{50901.289,677.975,10.000,-0.570},
				{50967.001,657.114,10.000,-0.500},
				{51030.626,636.253,10.000,-0.440},
				{51092.165,615.392,10.000,-0.380},
				{51151.618,594.532,10.000,-0.330},
				{51208.986,573.671,10.000,-0.290},
				{51264.267,552.810,10.000,-0.250},
				{51317.461,531.949,10.000,-0.210},
				{51368.570,511.089,10.000,-0.180},
				{51417.593,490.228,10.000,-0.150},
				{51464.530,469.367,10.000,-0.120},
				{51509.380,448.506,10.000,-0.100},
				{51552.145,427.646,10.000,-0.080},
				{51592.823,406.785,10.000,-0.070},
				{51631.416,385.924,10.000,-0.050},
				{51667.922,365.063,10.000,-0.040},
				{51702.342,344.202,10.000,-0.030},
				{51734.677,323.342,10.000,-0.030},
				{51764.925,302.481,10.000,-0.020},
				{51793.087,281.620,10.000,-0.020},
				{51819.163,260.759,10.000,-0.010},
				{51843.153,239.899,10.000,-0.010},
				{51865.056,219.038,10.000,-0.010},
				{51884.874,198.177,10.000,-0.010},
				{51902.606,177.316,10.000,-0.010},
				{51918.251,156.456,10.000,-0.010},
				{51931.811,135.595,10.000,-0.010},
				{51943.284,114.734,10.000,-0.010},
				{51952.672,93.873,10.000,-0.010},
				{51959.973,73.013,10.000,-0.010},
				{51965.188,52.152,10.000,-0.010}		};

}