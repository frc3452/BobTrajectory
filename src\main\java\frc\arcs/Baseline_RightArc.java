package frc.arcs;

import com.team319.follower.SrxMotionProfile;
import com.team319.follower.SrxTrajectory;

public class Baseline_RightArc extends SrxTrajectory {
	
	// WAYPOINTS:
	// (X,Y,degrees)
	// (1.42,3.58,0.00)
	// (13.08,3.58,0.00)
	
    public Baseline_RightArc() {
		super();
		this.highGear = true;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}

	
    public Baseline_RightArc(boolean flipped) {
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
				{43.808,125.165,10.000,0.000},
				{58.410,146.025,10.000,0.000},
				{75.099,166.886,10.000,0.000},
				{93.873,187.747,10.000,0.000},
				{114.734,208.608,10.000,0.000},
				{137.681,229.468,10.000,0.000},
				{162.714,250.329,10.000,0.000},
				{189.833,271.190,10.000,0.000},
				{219.038,292.051,10.000,0.000},
				{250.329,312.911,10.000,0.000},
				{283.706,333.772,10.000,0.000},
				{319.170,354.633,10.000,0.000},
				{356.719,375.494,10.000,0.000},
				{396.354,396.354,10.000,0.000},
				{438.076,417.215,10.000,0.000},
				{481.883,438.076,10.000,0.000},
				{527.777,458.937,10.000,0.000},
				{575.757,479.797,10.000,0.000},
				{625.823,500.658,10.000,0.000},
				{677.975,521.519,10.000,0.000},
				{732.213,542.380,10.000,0.000},
				{788.537,563.240,10.000,0.000},
				{846.947,584.101,10.000,0.000},
				{907.443,604.962,10.000,0.000},
				{970.025,625.823,10.000,0.000},
				{1034.694,646.683,10.000,0.000},
				{1101.448,667.544,10.000,0.000},
				{1170.288,688.405,10.000,0.000},
				{1241.215,709.266,10.000,0.000},
				{1314.228,730.126,10.000,0.000},
				{1389.326,750.987,10.000,0.000},
				{1466.511,771.848,10.000,0.000},
				{1545.782,792.709,10.000,0.000},
				{1627.139,813.570,10.000,0.000},
				{1710.582,834.430,10.000,0.000},
				{1796.111,855.291,10.000,0.000},
				{1883.726,876.152,10.000,0.000},
				{1973.428,897.013,10.000,0.000},
				{2065.215,917.873,10.000,0.000},
				{2159.088,938.734,10.000,0.000},
				{2255.048,959.595,10.000,0.000},
				{2353.093,980.456,10.000,0.000},
				{2453.225,1001.316,10.000,0.000},
				{2555.443,1022.177,10.000,0.000},
				{2659.746,1043.038,10.000,0.000},
				{2766.136,1063.899,10.000,0.000},
				{2874.612,1084.759,10.000,0.000},
				{2985.174,1105.620,10.000,0.000},
				{3097.822,1126.481,10.000,0.000},
				{3212.557,1147.342,10.000,0.000},
				{3329.377,1168.202,10.000,0.000},
				{3448.283,1189.063,10.000,0.000},
				{3569.275,1209.924,10.000,0.000},
				{3692.354,1230.785,10.000,0.000},
				{3817.518,1251.645,10.000,0.000},
				{3944.769,1272.506,10.000,0.000},
				{4074.106,1293.367,10.000,0.000},
				{4205.529,1314.228,10.000,0.000},
				{4339.037,1335.088,10.000,0.000},
				{4474.632,1355.949,10.000,0.000},
				{4612.313,1376.810,10.000,0.000},
				{4752.080,1397.671,10.000,0.000},
				{4893.934,1418.531,10.000,0.000},
				{5037.873,1439.392,10.000,0.000},
				{5183.898,1460.253,10.000,0.000},
				{5332.009,1481.114,10.000,0.000},
				{5482.207,1501.974,10.000,0.000},
				{5634.490,1522.835,10.000,0.000},
				{5788.860,1543.696,10.000,0.000},
				{5945.316,1564.557,10.000,0.000},
				{6103.857,1585.418,10.000,0.000},
				{6264.485,1606.278,10.000,0.000},
				{6427.199,1627.139,10.000,0.000},
				{6591.999,1648.000,10.000,0.000},
				{6758.885,1668.861,10.000,0.000},
				{6927.857,1689.721,10.000,0.000},
				{7098.916,1710.582,10.000,0.000},
				{7272.060,1731.443,10.000,0.000},
				{7447.290,1752.304,10.000,0.000},
				{7624.607,1773.164,10.000,0.000},
				{7804.009,1794.025,10.000,0.000},
				{7985.498,1814.886,10.000,0.000},
				{8169.072,1835.747,10.000,0.000},
				{8354.733,1856.607,10.000,0.000},
				{8542.480,1877.468,10.000,0.000},
				{8732.313,1898.329,10.000,0.000},
				{8924.232,1919.190,10.000,0.000},
				{9118.237,1940.050,10.000,0.000},
				{9314.328,1960.911,10.000,0.000},
				{9512.505,1981.772,10.000,0.000},
				{9712.768,2002.633,10.000,0.000},
				{9915.118,2023.493,10.000,0.000},
				{10119.553,2044.354,10.000,0.000},
				{10326.075,2065.215,10.000,0.000},
				{10534.682,2086.076,10.000,0.000},
				{10745.376,2106.936,10.000,0.000},
				{10958.155,2127.797,10.000,0.000},
				{11173.021,2148.658,10.000,0.000},
				{11389.973,2169.519,10.000,0.000},
				{11609.011,2190.379,10.000,0.000},
				{11830.135,2211.240,10.000,0.000},
				{12053.345,2232.101,10.000,0.000},
				{12278.641,2252.962,10.000,0.000},
				{12506.024,2273.822,10.000,0.000},
				{12735.492,2294.683,10.000,0.000},
				{12967.046,2315.544,10.000,0.000},
				{13200.687,2336.405,10.000,0.000},
				{13436.413,2357.266,10.000,0.000},
				{13674.226,2378.126,10.000,0.000},
				{13914.125,2398.987,10.000,0.000},
				{14156.109,2419.848,10.000,0.000},
				{14400.180,2440.709,10.000,0.000},
				{14646.337,2461.569,10.000,0.000},
				{14894.580,2482.430,10.000,0.000},
				{15144.909,2503.291,10.000,0.000},
				{15396.827,2519.176,10.000,0.000},
				{15646.658,2498.315,10.000,0.000},
				{15894.404,2477.454,10.000,0.000},
				{16140.063,2456.593,10.000,0.000},
				{16383.636,2435.733,10.000,0.000},
				{16625.124,2414.872,10.000,0.000},
				{16864.525,2394.011,10.000,0.000},
				{17101.840,2373.150,10.000,0.000},
				{17337.069,2352.290,10.000,0.000},
				{17570.212,2331.429,10.000,0.000},
				{17801.268,2310.568,10.000,0.000},
				{18030.239,2289.707,10.000,0.000},
				{18257.124,2268.847,10.000,0.000},
				{18481.922,2247.986,10.000,0.000},
				{18704.635,2227.125,10.000,0.000},
				{18925.261,2206.264,10.000,0.000},
				{19143.802,2185.404,10.000,0.000},
				{19360.256,2164.543,10.000,0.000},
				{19574.624,2143.682,10.000,0.000},
				{19786.906,2122.821,10.000,0.000},
				{19997.102,2101.961,10.000,0.000},
				{20205.212,2081.100,10.000,0.000},
				{20411.236,2060.239,10.000,0.000},
				{20615.174,2039.378,10.000,0.000},
				{20817.026,2018.518,10.000,0.000},
				{21016.792,1997.657,10.000,0.000},
				{21214.471,1976.796,10.000,0.000},
				{21410.065,1955.935,10.000,0.000},
				{21603.572,1935.075,10.000,0.000},
				{21794.994,1914.214,10.000,0.000},
				{21984.329,1893.353,10.000,0.000},
				{22171.578,1872.492,10.000,0.000},
				{22356.741,1851.631,10.000,0.000},
				{22539.818,1830.771,10.000,0.000},
				{22720.809,1809.910,10.000,0.000},
				{22899.714,1789.049,10.000,0.000},
				{23076.533,1768.188,10.000,0.000},
				{23251.266,1747.328,10.000,0.000},
				{23423.913,1726.467,10.000,0.000},
				{23594.473,1705.606,10.000,0.000},
				{23762.948,1684.745,10.000,0.000},
				{23929.336,1663.885,10.000,0.000},
				{24093.639,1643.024,10.000,0.000},
				{24255.855,1622.163,10.000,0.000},
				{24415.985,1601.302,10.000,0.000},
				{24574.029,1580.442,10.000,0.000},
				{24729.987,1559.581,10.000,0.000},
				{24883.859,1538.720,10.000,0.000},
				{25035.645,1517.859,10.000,0.000},
				{25185.345,1496.999,10.000,0.000},
				{25332.959,1476.138,10.000,0.000},
				{25478.487,1455.277,10.000,0.000},
				{25621.928,1434.416,10.000,0.000},
				{25763.284,1413.556,10.000,0.000},
				{25902.553,1392.695,10.000,0.000},
				{26039.737,1371.834,10.000,0.000},
				{26174.834,1350.973,10.000,0.000},
				{26307.845,1330.113,10.000,0.000},
				{26438.771,1309.252,10.000,0.000},
				{26567.610,1288.391,10.000,0.000},
				{26694.363,1267.530,10.000,0.000},
				{26819.030,1246.670,10.000,0.000},
				{26941.611,1225.809,10.000,0.000},
				{27062.105,1204.948,10.000,0.000},
				{27180.514,1184.087,10.000,0.000},
				{27296.837,1163.227,10.000,0.000},
				{27411.073,1142.366,10.000,0.000},
				{27523.224,1121.505,10.000,0.000},
				{27633.288,1100.644,10.000,0.000},
				{27741.267,1079.783,10.000,0.000},
				{27847.159,1058.923,10.000,0.000},
				{27950.965,1038.062,10.000,0.000},
				{28052.685,1017.201,10.000,0.000},
				{28152.319,996.340,10.000,0.000},
				{28249.867,975.480,10.000,0.000},
				{28345.329,954.619,10.000,0.000},
				{28438.705,933.758,10.000,0.000},
				{28529.995,912.897,10.000,0.000},
				{28619.198,892.037,10.000,0.000},
				{28706.316,871.176,10.000,0.000},
				{28791.347,850.315,10.000,0.000},
				{28874.293,829.454,10.000,0.000},
				{28955.152,808.594,10.000,0.000},
				{29033.925,787.733,10.000,0.000},
				{29110.613,766.872,10.000,0.000},
				{29185.214,746.011,10.000,0.000},
				{29257.729,725.151,10.000,0.000},
				{29328.158,704.290,10.000,0.000},
				{29396.501,683.429,10.000,0.000},
				{29462.758,662.568,10.000,0.000},
				{29526.928,641.708,10.000,0.000},
				{29589.013,620.847,10.000,0.000},
				{29649.012,599.986,10.000,0.000},
				{29706.924,579.125,10.000,0.000},
				{29762.751,558.265,10.000,0.000},
				{29816.491,537.404,10.000,0.000},
				{29868.145,516.543,10.000,0.000},
				{29917.714,495.682,10.000,0.000},
				{29965.196,474.822,10.000,0.000},
				{30010.592,453.961,10.000,0.000},
				{30053.902,433.100,10.000,0.000},
				{30095.126,412.239,10.000,0.000},
				{30134.264,391.379,10.000,0.000},
				{30171.315,370.518,10.000,0.000},
				{30206.281,349.657,10.000,0.000},
				{30239.161,328.796,10.000,0.000},
				{30269.954,307.935,10.000,0.000},
				{30298.662,287.075,10.000,0.000},
				{30325.283,266.214,10.000,0.000},
				{30349.818,245.353,10.000,0.000},
				{30372.268,224.492,10.000,0.000},
				{30392.631,203.632,10.000,0.000},
				{30410.908,182.771,10.000,0.000},
				{30427.099,161.910,10.000,0.000},
				{30441.204,141.049,10.000,0.000},
				{30453.223,120.189,10.000,0.000},
				{30463.156,99.328,10.000,0.000},
				{30471.002,78.467,10.000,0.000},
				{30476.763,57.606,10.000,0.000},
				{30480.438,36.746,10.000,0.000}		};

}