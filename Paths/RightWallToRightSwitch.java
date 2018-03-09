package org.usfirst.frc.team319.paths;

import org.usfirst.frc.team319.models.SrxMotionProfile;
import org.usfirst.frc.team319.models.SrxTrajectory;

public class RightWallToRightSwitch extends SrxTrajectory{
	
	// WAYPOINTS:
	// (X,Y,degrees)
	// (0.00,0.00,0.00)
	// (17.00,-1.00,-30.00)
	
	public RightWallToRightSwitch() {
		this(false);
	}
			
    public RightWallToRightSwitch(boolean flipped) {
		super();
		
		double[][] leftPoints = {
				{-0.567,-11.339,10.000},
				{-2.835,-22.679,10.000},
				{-7.938,-51.027,10.000},
				{-17.009,-90.717,10.000},
				{-31.184,-141.749,10.000},
				{-51.597,-204.126,10.000},
				{-79.382,-277.853,10.000},
				{-115.676,-362.935,10.000},
				{-161.614,-459.380,10.000},
				{-218.333,-567.199,10.000},
				{-286.974,-686.404,10.000},
				{-368.675,-817.010,10.000},
				{-464.579,-959.036,10.000},
				{-575.829,-1112.502,10.000},
				{-703.004,-1271.756,10.000},
				{-846.118,-1431.138,10.000},
				{-1005.185,-1590.665,10.000},
				{-1180.220,-1750.350,10.000},
				{-1371.241,-1910.210,10.000},
				{-1578.267,-2070.259,10.000},
				{-1801.318,-2230.510,10.000},
				{-2040.415,-2390.978,10.000},
				{-2295.583,-2551.676,10.000},
				{-2566.845,-2712.617,10.000},
				{-2854.226,-2873.814,10.000},
				{-3157.754,-3035.278,10.000},
				{-3477.456,-3197.022,10.000},
				{-3813.362,-3359.057,10.000},
				{-4165.501,-3521.392,10.000},
				{-4533.905,-3684.039,10.000},
				{-4918.605,-3847.006,10.000},
				{-5319.636,-4010.301,10.000},
				{-5737.029,-4173.935,10.000},
				{-6170.820,-4337.912,10.000},
				{-6621.044,-4502.242,10.000},
				{-7087.737,-4666.928,10.000},
				{-7570.935,-4831.978,10.000},
				{-8070.674,-4997.394,10.000},
				{-8586.993,-5163.182,10.000},
				{-9119.927,-5329.343,10.000},
				{-9669.515,-5495.880,10.000},
				{-10235.794,-5662.795,10.000},
				{-10818.803,-5830.088,10.000},
				{-11418.579,-5997.757,10.000},
				{-12035.159,-6165.803,10.000},
				{-12668.581,-6334.222,10.000},
				{-13318.883,-6503.012,10.000},
				{-13986.099,-6672.168,10.000},
				{-14670.268,-6841.685,10.000},
				{-15371.424,-7011.558,10.000},
				{-16089.602,-7181.778,10.000},
				{-16824.835,-7352.338,10.000},
				{-17577.158,-7523.230,10.000},
				{-18346.603,-7694.442,10.000},
				{-19133.199,-7865.964,10.000},
				{-19936.977,-8037.784,10.000},
				{-20757.966,-8209.888,10.000},
				{-21596.193,-8382.263,10.000},
				{-22451.682,-8554.892,10.000},
				{-23324.458,-8727.760,10.000},
				{-24214.543,-8900.850,10.000},
				{-25121.957,-9074.142,10.000},
				{-26046.719,-9247.618,10.000},
				{-26988.844,-9421.257,10.000},
				{-27948.348,-9595.037,10.000},
				{-28925.242,-9768.936,10.000},
				{-29919.535,-9942.931,10.000},
				{-30931.234,-10116.996,10.000},
				{-31960.345,-10291.107,10.000},
				{-33006.869,-10465.236,10.000},
				{-34070.804,-10639.357,10.000},
				{-35152.149,-10813.441,10.000},
				{-36250.894,-10987.459,10.000},
				{-37367.032,-11161.380,10.000},
				{-38500.550,-11335.174,10.000},
				{-39650.835,-11502.851,10.000},
				{-40816.675,-11658.402,10.000},
				{-41996.852,-11801.772,10.000},
				{-43190.143,-11932.909,10.000},
				{-44395.319,-12051.762,10.000},
				{-45611.148,-12158.290,10.000},
				{-46836.394,-12252.453,10.000},
				{-48069.815,-12334.217,10.000},
				{-49310.171,-12403.554,10.000},
				{-50556.215,-12460.440,10.000},
				{-51806.700,-12504.857,10.000},
				{-53060.380,-12536.792,10.000},
				{-54316.004,-12556.239,10.000},
				{-55572.323,-12563.195,10.000},
				{-56828.688,-12563.645,10.000},
				{-58085.046,-12563.582,10.000},
				{-59341.347,-12563.012,10.000},
				{-60597.541,-12561.944,10.000},
				{-61853.580,-12560.388,10.000},
				{-63109.415,-12558.352,10.000},
				{-64365.000,-12555.845,10.000},
				{-65620.287,-12552.875,10.000},
				{-66875.233,-12549.451,10.000},
				{-68129.791,-12545.583,10.000},
				{-69383.919,-12541.279,10.000},
				{-70637.574,-12536.548,10.000},
				{-71890.713,-12531.398,10.000},
				{-73143.297,-12525.839,10.000},
				{-74395.285,-12519.878,10.000},
				{-75646.638,-12513.526,10.000},
				{-76897.317,-12506.790,10.000},
				{-78147.284,-12499.679,10.000},
				{-79396.505,-12492.201,10.000},
				{-80644.941,-12484.365,10.000},
				{-81892.559,-12476.180,10.000},
				{-83139.324,-12467.652,10.000},
				{-84385.203,-12458.791,10.000},
				{-85630.164,-12449.605,10.000},
				{-86874.174,-12440.100,10.000},
				{-88117.203,-12430.286,10.000},
				{-89359.220,-12420.170,10.000},
				{-90600.195,-12409.759,10.000},
				{-91840.101,-12399.060,10.000},
				{-93078.909,-12388.081,10.000},
				{-94316.592,-12376.828,10.000},
				{-95553.123,-12365.310,10.000},
				{-96788.476,-12353.532,10.000},
				{-98022.626,-12341.500,10.000},
				{-99255.549,-12329.223,10.000},
				{-100487.219,-12316.705,10.000},
				{-101717.614,-12303.952,10.000},
				{-102946.712,-12290.972,10.000},
				{-104174.489,-12277.769,10.000},
				{-105400.923,-12264.349,10.000},
				{-106625.995,-12250.717,10.000},
				{-107849.683,-12236.880,10.000},
				{-109071.967,-12222.841,10.000},
				{-110292.828,-12208.606,10.000},
				{-111512.246,-12194.179,10.000},
				{-112730.202,-12179.566,10.000},
				{-113946.679,-12164.770,10.000},
				{-115161.659,-12149.796,10.000},
				{-116375.124,-12134.648,10.000},
				{-117587.057,-12119.330,10.000},
				{-118797.441,-12103.846,10.000},
				{-120006.261,-12088.200,10.000},
				{-121213.501,-12072.394,10.000},
				{-122419.144,-12056.433,10.000},
				{-123623.176,-12040.320,10.000},
				{-124825.582,-12024.057,10.000},
				{-126026.347,-12007.648,10.000},
				{-127225.456,-11991.096,10.000},
				{-128422.897,-11974.404,10.000},
				{-129618.654,-11957.573,10.000},
				{-130812.715,-11940.608,10.000},
				{-132005.066,-11923.510,10.000},
				{-133195.694,-11906.281,10.000},
				{-134384.586,-11888.925,10.000},
				{-135571.730,-11871.442,10.000},
				{-136757.114,-11853.836,10.000},
				{-137940.725,-11836.109,10.000},
				{-139122.551,-11818.261,10.000},
				{-140302.581,-11800.297,10.000},
				{-141480.802,-11782.217,10.000},
				{-142657.205,-11764.023,10.000},
				{-143831.777,-11745.718,10.000},
				{-145004.507,-11727.303,10.000},
				{-146175.385,-11708.780,10.000},
				{-147344.400,-11690.151,10.000},
				{-148511.542,-11671.418,10.000},
				{-149676.800,-11652.583,10.000},
				{-150840.165,-11633.647,10.000},
				{-152001.626,-11614.614,10.000},
				{-153161.174,-11595.484,10.000},
				{-154318.800,-11576.260,10.000},
				{-155474.495,-11556.945,10.000},
				{-156628.249,-11537.539,10.000},
				{-157780.054,-11518.047,10.000},
				{-158929.900,-11498.470,10.000},
				{-160077.781,-11478.810,10.000},
				{-161223.689,-11459.071,10.000},
				{-162367.614,-11439.255,10.000},
				{-163509.551,-11419.366,10.000},
				{-164649.491,-11399.406,10.000},
				{-165787.429,-11379.378,10.000},
				{-166923.358,-11359.287,10.000},
				{-168057.271,-11339.135,10.000},
				{-169189.164,-11318.927,10.000},
				{-170319.031,-11298.667,10.000},
				{-171446.867,-11278.359,10.000},
				{-172572.667,-11258.007,10.000},
				{-173696.429,-11237.616,10.000},
				{-174818.148,-11217.192,10.000},
				{-175937.822,-11196.740,10.000},
				{-177055.449,-11176.264,10.000},
				{-178171.026,-11155.772,10.000},
				{-179284.553,-11135.269,10.000},
				{-180396.029,-11114.762,10.000},
				{-181505.455,-11094.257,10.000},
				{-182612.831,-11073.762,10.000},
				{-183718.159,-11053.283,10.000},
				{-184821.442,-11032.830,10.000},
				{-185922.683,-11012.409,10.000},
				{-187021.886,-10992.030,10.000},
				{-188119.056,-10971.702,10.000},
				{-189214.199,-10951.432,10.000},
				{-190307.323,-10931.232,10.000},
				{-191398.434,-10911.110,10.000},
				{-192487.541,-10891.078,10.000},
				{-193574.656,-10871.146,10.000},
				{-194659.789,-10851.325,10.000},
				{-195742.951,-10831.626,10.000},
				{-196824.157,-10812.061,10.000},
				{-197903.421,-10792.642,10.000},
				{-198980.760,-10773.382,10.000},
				{-200056.189,-10754.294,10.000},
				{-201129.728,-10735.390,10.000},
				{-202201.396,-10716.684,10.000},
				{-203271.215,-10698.189,10.000},
				{-204339.207,-10679.920,10.000},
				{-205405.396,-10661.892,10.000},
				{-206469.808,-10644.117,10.000},
				{-207532.469,-10626.612,10.000},
				{-208593.408,-10609.391,10.000},
				{-209652.655,-10592.469,10.000},
				{-210710.242,-10575.861,10.000},
				{-211766.200,-10559.583,10.000},
				{-212820.565,-10543.651,10.000},
				{-213873.373,-10528.078,10.000},
				{-214924.661,-10512.882,10.000},
				{-215974.469,-10498.078,10.000},
				{-217022.837,-10483.680,10.000},
				{-218069.807,-10469.705,10.000},
				{-219115.424,-10456.167,10.000},
				{-220159.732,-10443.082,10.000},
				{-221202.778,-10430.463,10.000},
				{-222244.611,-10418.326,10.000},
				{-223285.280,-10406.685,10.000},
				{-224324.835,-10395.554,10.000},
				{-225363.330,-10384.946,10.000},
				{-226400.817,-10374.874,10.000},
				{-227437.352,-10365.350,10.000},
				{-228472.991,-10356.387,10.000},
				{-229507.790,-10347.997,10.000},
				{-230541.809,-10340.189,10.000},
				{-231575.107,-10332.974,10.000},
				{-232607.743,-10326.363,10.000},
				{-233639.779,-10320.363,10.000},
				{-234671.278,-10314.983,10.000},
				{-235702.301,-10310.230,10.000},
				{-236732.912,-10306.112,10.000},
				{-237763.175,-10302.633,10.000},
				{-238793.155,-10299.798,10.000},
				{-239822.916,-10297.613,10.000},
				{-240852.524,-10296.080,10.000},
				{-241882.044,-10295.201,10.000},
				{-242911.542,-10294.980,10.000},
				{-243941.084,-10295.415,10.000},
				{-244970.734,-10296.507,10.000},
				{-246000.560,-10298.255,10.000},
				{-247030.626,-10300.657,10.000},
				{-248060.997,-10303.711,10.000},
				{-249091.738,-10307.414,10.000},
				{-250122.914,-10311.760,10.000},
				{-251154.589,-10316.745,10.000},
				{-252186.825,-10322.363,10.000},
				{-253219.686,-10328.608,10.000},
				{-254253.233,-10335.472,10.000},
				{-255287.528,-10342.948,10.000},
				{-256322.630,-10351.027,10.000},
				{-257358.600,-10359.700,10.000},
				{-258395.496,-10368.958,10.000},
				{-259433.375,-10378.790,10.000},
				{-260472.294,-10389.187,10.000},
				{-261512.307,-10400.136,10.000},
				{-262553.470,-10411.628,10.000},
				{-263595.835,-10423.649,10.000},
				{-264639.454,-10436.189,10.000},
				{-265684.377,-10449.235,10.000},
				{-266730.655,-10462.775,10.000},
				{-267778.335,-10476.797,10.000},
				{-268827.463,-10491.287,10.000},
				{-269878.087,-10506.234,10.000},
				{-270930.249,-10521.624,10.000},
				{-271983.994,-10537.446,10.000},
				{-273039.362,-10553.686,10.000},
				{-274096.395,-10570.332,10.000},
				{-275155.133,-10587.372,10.000},
				{-276215.612,-10604.795,10.000},
				{-277277.871,-10622.587,10.000},
				{-278341.945,-10640.738,10.000},
				{-279407.868,-10659.236,10.000},
				{-280475.675,-10678.071,10.000},
				{-281545.399,-10697.231,10.000},
				{-282617.069,-10716.707,10.000},
				{-283690.718,-10736.489,10.000},
				{-284766.349,-10756.311,10.000},
				{-285843.452,-10771.026,10.000},
				{-286921.021,-10775.694,10.000},
				{-287998.044,-10770.226,10.000},
				{-289073.497,-10754.531,10.000},
				{-290146.349,-10728.520,10.000},
				{-291215.559,-10692.098,10.000},
				{-292280.076,-10645.173,10.000},
				{-293338.841,-10587.654,10.000},
				{-294390.786,-10519.446,10.000},
				{-295434.832,-10440.458,10.000},
				{-296469.892,-10350.600,10.000},
				{-297494.870,-10249.780,10.000},
				{-298508.661,-10137.910,10.000},
				{-299510.177,-10015.167,10.000},
				{-300498.852,-9886.746,10.000},
				{-301474.615,-9757.633,10.000},
				{-302437.398,-9627.827,10.000},
				{-303387.131,-9497.326,10.000},
				{-304323.744,-9366.132,10.000},
				{-305247.169,-9234.247,10.000},
				{-306157.336,-9101.671,10.000},
				{-307054.176,-8968.407,10.000},
				{-307937.622,-8834.459,10.000},
				{-308807.605,-8699.829,10.000},
				{-309664.057,-8564.522,10.000},
				{-310506.912,-8428.542,10.000},
				{-311336.101,-8291.893,10.000},
				{-312151.559,-8154.580,10.000},
				{-312953.220,-8016.609,10.000},
				{-313741.018,-7877.985,10.000},
				{-314514.890,-7738.714,10.000},
				{-315274.770,-7598.803,10.000},
				{-316020.596,-7458.256,10.000},
				{-316752.304,-7317.082,10.000},
				{-317469.832,-7175.286,10.000},
				{-318173.120,-7032.876,10.000},
				{-318862.106,-6889.859,10.000},
				{-319536.730,-6746.242,10.000},
				{-320196.934,-6602.034,10.000},
				{-320842.658,-6457.241,10.000},
				{-321473.845,-6311.873,10.000},
				{-322090.439,-6165.937,10.000},
				{-322692.383,-6019.442,10.000},
				{-323279.623,-5872.397,10.000},
				{-323852.104,-5724.811,10.000},
				{-324409.773,-5576.693,10.000},
				{-324952.578,-5428.053,10.000},
				{-325480.468,-5278.901,10.000},
				{-325993.393,-5129.246,10.000},
				{-326491.303,-4979.100,10.000},
				{-326974.150,-4828.473,10.000},
				{-327441.888,-4677.376,10.000},
				{-327894.470,-4525.819,10.000},
				{-328331.851,-4373.816,10.000},
				{-328753.989,-4221.377,10.000},
				{-329160.840,-4068.515,10.000},
				{-329552.365,-3915.242,10.000},
				{-329928.522,-3761.570,10.000},
				{-330289.273,-3607.514,10.000},
				{-330634.582,-3453.086,10.000},
				{-330964.412,-3298.300,10.000},
				{-331278.729,-3143.170,10.000},
				{-331577.499,-2987.710,10.000},
				{-331860.693,-2831.934,10.000},
				{-332128.279,-2675.858,10.000},
				{-332380.228,-2519.497,10.000},
				{-332616.515,-2362.866,10.000},
				{-332837.113,-2205.980,10.000},
				{-333041.999,-2048.855,10.000},
				{-333231.149,-1891.508,10.000},
				{-333404.545,-1733.954,10.000},
				{-333562.166,-1576.211,10.000},
				{-333703.995,-1418.295,10.000},
				{-333830.018,-1260.222,10.000},
				{-333940.247,-1102.293,10.000},
				{-334035.266,-950.189,10.000},
				{-334116.197,-809.315,10.000},
				{-334184.167,-679.695,10.000},
				{-334240.302,-561.349,10.000},
				{-334285.731,-454.292,10.000},
				{-334321.585,-358.538,10.000},
				{-334348.994,-274.097,10.000},
				{-334369.092,-200.979,10.000},
				{-334383.011,-139.190,10.000},
				{-334391.885,-88.733,10.000},
				{-334396.846,-49.613,10.000},
				{-334399.029,-21.831,10.000},
				{-334399.568,-5.387,10.000},
				{-334399.568,-0.000,10.000}
		};
		
		double[][] rightPoints = {
				{-0.567,-11.339,10.000},
				{-2.835,-22.679,10.000},
				{-7.937,-51.026,10.000},
				{-17.009,-90.712,10.000},
				{-31.182,-141.734,10.000},
				{-51.591,-204.089,10.000},
				{-79.368,-277.774,10.000},
				{-115.646,-362.781,10.000},
				{-161.557,-459.104,10.000},
				{-218.230,-566.732,10.000},
				{-286.795,-685.652,10.000},
				{-368.380,-815.851,10.000},
				{-464.111,-957.308,10.000},
				{-575.111,-1110.003,10.000},
				{-701.936,-1268.250,10.000},
				{-844.573,-1426.368,10.000},
				{-1003.007,-1584.343,10.000},
				{-1177.223,-1742.157,10.000},
				{-1367.203,-1899.798,10.000},
				{-1572.928,-2057.251,10.000},
				{-1794.378,-2214.500,10.000},
				{-2031.531,-2371.533,10.000},
				{-2284.365,-2528.336,10.000},
				{-2552.854,-2684.895,10.000},
				{-2836.974,-2841.199,10.000},
				{-3136.698,-2997.235,10.000},
				{-3451.997,-3152.992,10.000},
				{-3782.843,-3308.458,10.000},
				{-4129.205,-3463.624,10.000},
				{-4491.053,-3618.478,10.000},
				{-4868.354,-3773.012,10.000},
				{-5261.076,-3927.216,10.000},
				{-5669.184,-4081.084,10.000},
				{-6092.645,-4234.607,10.000},
				{-6531.423,-4387.778,10.000},
				{-6985.482,-4540.592,10.000},
				{-7454.786,-4693.044,10.000},
				{-7939.299,-4845.128,10.000},
				{-8438.983,-4996.841,10.000},
				{-8953.801,-5148.181,10.000},
				{-9483.716,-5299.144,10.000},
				{-10028.689,-5449.730,10.000},
				{-10588.683,-5599.938,10.000},
				{-11163.659,-5749.769,10.000},
				{-11753.582,-5899.224,10.000},
				{-12358.412,-6048.306,10.000},
				{-12978.114,-6197.016,10.000},
				{-13612.650,-6345.361,10.000},
				{-14261.985,-6493.345,10.000},
				{-14926.082,-6640.973,10.000},
				{-15604.907,-6788.253,10.000},
				{-16298.427,-6935.193,10.000},
				{-17006.607,-7081.803,10.000},
				{-17729.416,-7228.091,10.000},
				{-18466.823,-7374.070,10.000},
				{-19218.798,-7519.750,10.000},
				{-19985.313,-7665.147,10.000},
				{-20766.340,-7810.273,10.000},
				{-21561.854,-7955.144,10.000},
				{-22371.832,-8099.777,10.000},
				{-23196.251,-8244.188,10.000},
				{-24035.090,-8388.396,10.000},
				{-24888.332,-8532.421,10.000},
				{-25755.961,-8676.282,10.000},
				{-26637.961,-8820.003,10.000},
				{-27534.321,-8963.604,10.000},
				{-28445.032,-9107.110,10.000},
				{-29370.087,-9250.545,10.000},
				{-30309.480,-9393.935,10.000},
				{-31263.211,-9537.306,10.000},
				{-32231.280,-9680.686,10.000},
				{-33213.690,-9824.102,10.000},
				{-34210.448,-9967.585,10.000},
				{-35221.565,-10111.164,10.000},
				{-36247.052,-10254.870,10.000},
				{-37286.387,-10393.355,10.000},
				{-38338.516,-10521.286,10.000},
				{-39402.388,-10638.720,10.000},
				{-40476.959,-10745.709,10.000},
				{-41561.189,-10842.302,10.000},
				{-42654.043,-10928.542,10.000},
				{-43754.490,-11004.469,10.000},
				{-44861.502,-11070.115,10.000},
				{-45974.053,-11125.511,10.000},
				{-47091.121,-11170.679,10.000},
				{-48211.684,-11205.637,10.000},
				{-49334.724,-11230.398,10.000},
				{-50459.221,-11244.969,10.000},
				{-51584.156,-11249.352,10.000},
				{-52709.046,-11248.902,10.000},
				{-53833.943,-11248.965,10.000},
				{-54958.896,-11249.535,10.000},
				{-56083.957,-11250.603,10.000},
				{-57209.173,-11252.159,10.000},
				{-58334.592,-11254.195,10.000},
				{-59460.262,-11256.702,10.000},
				{-60586.230,-11259.672,10.000},
				{-61712.539,-11263.096,10.000},
				{-62839.236,-11266.964,10.000},
				{-63966.362,-11271.268,10.000},
				{-65093.962,-11276.000,10.000},
				{-66222.077,-11281.150,10.000},
				{-67350.748,-11286.709,10.000},
				{-68480.015,-11292.670,10.000},
				{-69609.917,-11299.022,10.000},
				{-70740.493,-11305.758,10.000},
				{-71871.780,-11312.869,10.000},
				{-73003.815,-11320.347,10.000},
				{-74136.633,-11328.183,10.000},
				{-75270.270,-11336.369,10.000},
				{-76404.760,-11344.897,10.000},
				{-77540.136,-11353.758,10.000},
				{-78676.430,-11362.945,10.000},
				{-79813.675,-11372.449,10.000},
				{-80951.901,-11382.263,10.000},
				{-82091.139,-11392.380,10.000},
				{-83231.418,-11402.791,10.000},
				{-84372.767,-11413.490,10.000},
				{-85515.214,-11424.469,10.000},
				{-86658.787,-11435.722,10.000},
				{-87803.511,-11447.241,10.000},
				{-88949.412,-11459.019,10.000},
				{-90096.518,-11471.050,10.000},
				{-91244.850,-11483.328,10.000},
				{-92394.435,-11495.846,10.000},
				{-93545.295,-11508.599,10.000},
				{-94697.453,-11521.580,10.000},
				{-95850.931,-11534.783,10.000},
				{-97005.751,-11548.203,10.000},
				{-98161.935,-11561.835,10.000},
				{-99319.502,-11575.672,10.000},
				{-100478.473,-11589.711,10.000},
				{-101638.868,-11603.947,10.000},
				{-102800.705,-11618.373,10.000},
				{-103964.004,-11632.987,10.000},
				{-105128.782,-11647.783,10.000},
				{-106295.058,-11662.757,10.000},
				{-107462.848,-11677.905,10.000},
				{-108632.171,-11693.223,10.000},
				{-109803.041,-11708.707,10.000},
				{-110975.477,-11724.353,10.000},
				{-112149.493,-11740.159,10.000},
				{-113325.105,-11756.120,10.000},
				{-114502.328,-11772.234,10.000},
				{-115681.178,-11788.497,10.000},
				{-116861.668,-11804.905,10.000},
				{-118043.814,-11821.457,10.000},
				{-119227.629,-11838.150,10.000},
				{-120413.127,-11854.980,10.000},
				{-121600.321,-11871.946,10.000},
				{-122789.226,-11889.044,10.000},
				{-123979.853,-11906.273,10.000},
				{-125172.216,-11923.629,10.000},
				{-126366.327,-11941.112,10.000},
				{-127562.199,-11958.718,10.000},
				{-128759.844,-11976.445,10.000},
				{-129959.273,-11994.292,10.000},
				{-131160.498,-12012.257,10.000},
				{-132363.532,-12030.337,10.000},
				{-133568.385,-12048.530,10.000},
				{-134775.069,-12066.835,10.000},
				{-135983.594,-12085.250,10.000},
				{-137193.971,-12103.773,10.000},
				{-138406.211,-12122.402,10.000},
				{-139620.325,-12141.135,10.000},
				{-140836.322,-12159.970,10.000},
				{-142054.212,-12178.905,10.000},
				{-143274.006,-12197.939,10.000},
				{-144495.713,-12217.068,10.000},
				{-145719.342,-12236.292,10.000},
				{-146944.903,-12255.607,10.000},
				{-148172.404,-12275.012,10.000},
				{-149401.855,-12294.505,10.000},
				{-150633.263,-12314.082,10.000},
				{-151866.637,-12333.741,10.000},
				{-153101.985,-12353.480,10.000},
				{-154339.314,-12373.295,10.000},
				{-155578.633,-12393.184,10.000},
				{-156819.947,-12413.144,10.000},
				{-158063.264,-12433.171,10.000},
				{-159308.591,-12453.262,10.000},
				{-160555.932,-12473.414,10.000},
				{-161805.294,-12493.621,10.000},
				{-163056.682,-12513.881,10.000},
				{-164310.101,-12534.189,10.000},
				{-165565.555,-12554.540,10.000},
				{-166823.048,-12574.930,10.000},
				{-168082.584,-12595.354,10.000},
				{-169344.164,-12615.806,10.000},
				{-170607.792,-12636.281,10.000},
				{-171873.470,-12656.773,10.000},
				{-173141.197,-12677.275,10.000},
				{-174410.975,-12697.782,10.000},
				{-175682.804,-12718.286,10.000},
				{-176956.682,-12738.781,10.000},
				{-178232.608,-12759.259,10.000},
				{-179510.579,-12779.712,10.000},
				{-180790.592,-12800.132,10.000},
				{-182072.643,-12820.510,10.000},
				{-183356.727,-12840.839,10.000},
				{-184642.838,-12861.107,10.000},
				{-185930.969,-12881.307,10.000},
				{-187221.111,-12901.428,10.000},
				{-188513.257,-12921.459,10.000},
				{-189807.396,-12941.391,10.000},
				{-191103.518,-12961.212,10.000},
				{-192401.609,-12980.910,10.000},
				{-193701.656,-13000.474,10.000},
				{-195003.645,-13019.892,10.000},
				{-196307.560,-13039.152,10.000},
				{-197613.384,-13058.239,10.000},
				{-198921.099,-13077.143,10.000},
				{-200230.683,-13095.848,10.000},
				{-201542.118,-13114.342,10.000},
				{-202855.379,-13132.610,10.000},
				{-204170.442,-13150.638,10.000},
				{-205487.284,-13168.412,10.000},
				{-206805.875,-13185.916,10.000},
				{-208126.189,-13203.137,10.000},
				{-209448.195,-13220.058,10.000},
				{-210771.861,-13236.665,10.000},
				{-212097.155,-13252.942,10.000},
				{-213424.043,-13268.874,10.000},
				{-214752.487,-13284.446,10.000},
				{-216082.451,-13299.641,10.000},
				{-217413.896,-13314.445,10.000},
				{-218746.780,-13328.842,10.000},
				{-220081.062,-13342.817,10.000},
				{-221416.697,-13356.354,10.000},
				{-222753.641,-13369.439,10.000},
				{-224091.847,-13382.057,10.000},
				{-225431.266,-13394.193,10.000},
				{-226771.849,-13405.833,10.000},
				{-228113.546,-13416.964,10.000},
				{-229456.303,-13427.572,10.000},
				{-230800.067,-13437.644,10.000},
				{-232144.784,-13447.167,10.000},
				{-233490.397,-13456.129,10.000},
				{-234836.849,-13464.519,10.000},
				{-236184.082,-13472.327,10.000},
				{-237532.036,-13479.541,10.000},
				{-238880.651,-13486.152,10.000},
				{-240229.866,-13492.152,10.000},
				{-241579.619,-13497.531,10.000},
				{-242929.847,-13502.284,10.000},
				{-244280.488,-13506.402,10.000},
				{-245631.476,-13509.881,10.000},
				{-246982.747,-13512.715,10.000},
				{-248334.237,-13514.901,10.000},
				{-249685.881,-13516.434,10.000},
				{-251037.612,-13517.312,10.000},
				{-252389.365,-13517.534,10.000},
				{-253741.075,-13517.099,10.000},
				{-255092.676,-13516.007,10.000},
				{-256444.102,-13514.258,10.000},
				{-257795.287,-13511.856,10.000},
				{-259146.167,-13508.802,10.000},
				{-260496.677,-13505.100,10.000},
				{-261846.753,-13500.754,10.000},
				{-263196.330,-13495.770,10.000},
				{-264545.345,-13490.152,10.000},
				{-265893.736,-13483.907,10.000},
				{-267241.440,-13477.044,10.000},
				{-268588.397,-13469.568,10.000},
				{-269934.546,-13461.489,10.000},
				{-271279.828,-13452.817,10.000},
				{-272624.184,-13443.559,10.000},
				{-273967.556,-13433.727,10.000},
				{-275309.889,-13423.331,10.000},
				{-276651.128,-13412.382,10.000},
				{-277991.217,-13400.891,10.000},
				{-279330.104,-13388.870,10.000},
				{-280667.737,-13376.331,10.000},
				{-282004.065,-13363.285,10.000},
				{-283339.040,-13349.746,10.000},
				{-284672.612,-13335.725,10.000},
				{-286004.736,-13321.235,10.000},
				{-287335.365,-13306.289,10.000},
				{-288664.455,-13290.900,10.000},
				{-289991.963,-13275.079,10.000},
				{-291317.847,-13258.840,10.000},
				{-292642.066,-13242.194,10.000},
				{-293964.582,-13225.154,10.000},
				{-295285.355,-13207.733,10.000},
				{-296604.349,-13189.941,10.000},
				{-297921.528,-13171.791,10.000},
				{-299236.857,-13153.293,10.000},
				{-300550.303,-13134.459,10.000},
				{-301861.833,-13115.300,10.000},
				{-303171.416,-13095.825,10.000},
				{-304479.020,-13076.044,10.000},
				{-305784.586,-13055.657,10.000},
				{-307087.433,-13028.473,10.000},
				{-308386.299,-12988.657,10.000},
				{-309679.929,-12936.299,10.000},
				{-310967.078,-12871.488,10.000},
				{-312246.509,-12794.317,10.000},
				{-313516.997,-12704.876,10.000},
				{-314777.323,-12603.260,10.000},
				{-316026.279,-12489.561,10.000},
				{-317262.666,-12363.870,10.000},
				{-318485.294,-12226.281,10.000},
				{-319692.983,-12076.885,10.000},
				{-320884.560,-11915.771,10.000},
				{-322058.862,-11743.028,10.000},
				{-323214.767,-11559.045,10.000},
				{-324351.763,-11369.966,10.000},
				{-325469.921,-11181.579,10.000},
				{-326569.310,-10993.886,10.000},
				{-327649.998,-10806.886,10.000},
				{-328712.056,-10620.579,10.000},
				{-329755.553,-10434.965,10.000},
				{-330780.557,-10250.041,10.000},
				{-331787.137,-10065.804,10.000},
				{-332775.363,-9882.252,10.000},
				{-333745.301,-9699.381,10.000},
				{-334697.019,-9517.188,10.000},
				{-335630.586,-9335.668,10.000},
				{-336546.068,-9154.816,10.000},
				{-337443.531,-8974.629,10.000},
				{-338323.041,-8795.099,10.000},
				{-339184.663,-8616.222,10.000},
				{-340028.462,-8437.993,10.000},
				{-340854.503,-8260.404,10.000},
				{-341662.848,-8083.450,10.000},
				{-342453.560,-7907.124,10.000},
				{-343226.702,-7731.419,10.000},
				{-343982.335,-7556.328,10.000},
				{-344720.519,-7381.844,10.000},
				{-345441.315,-7207.960,10.000},
				{-346144.782,-7034.668,10.000},
				{-346830.978,-6861.960,10.000},
				{-347499.961,-6689.828,10.000},
				{-348151.787,-6518.263,10.000},
				{-348786.513,-6347.258,10.000},
				{-349404.193,-6176.802,10.000},
				{-350004.882,-6006.888,10.000},
				{-350588.632,-5837.505,10.000},
				{-351155.497,-5668.644,10.000},
				{-351705.526,-5500.296,10.000},
				{-352238.771,-5332.449,10.000},
				{-352755.281,-5165.095,10.000},
				{-353255.103,-4998.221,10.000},
				{-353738.285,-4831.818,10.000},
				{-354204.872,-4665.873,10.000},
				{-354654.909,-4500.376,10.000},
				{-355088.441,-4335.314,10.000},
				{-355505.508,-4170.676,10.000},
				{-355906.153,-4006.448,10.000},
				{-356290.415,-3842.619,10.000},
				{-356658.333,-3679.174,10.000},
				{-357009.943,-3516.102,10.000},
				{-357345.281,-3353.387,10.000},
				{-357664.383,-3191.016,10.000},
				{-357967.281,-3028.976,10.000},
				{-358254.006,-2867.250,10.000},
				{-358524.588,-2705.826,10.000},
				{-358779.057,-2544.686,10.000},
				{-359017.439,-2383.817,10.000},
				{-359239.759,-2223.202,10.000},
				{-359446.041,-2062.826,10.000},
				{-359636.309,-1902.673,10.000},
				{-359810.581,-1742.725,10.000},
				{-359968.878,-1582.968,10.000},
				{-360111.216,-1423.384,10.000},
				{-360237.612,-1263.955,10.000},
				{-360348.107,-1104.949,10.000},
				{-360443.309,-952.023,10.000},
				{-360524.363,-810.544,10.000},
				{-360592.412,-680.490,10.000},
				{-360648.597,-561.842,10.000},
				{-360694.055,-454.583,10.000},
				{-360729.925,-358.699,10.000},
				{-360757.343,-274.180,10.000},
				{-360777.445,-201.018,10.000},
				{-360791.365,-139.205,10.000},
				{-360800.239,-88.738,10.000},
				{-360805.200,-49.614,10.000},
				{-360807.383,-21.831,10.000},
				{-360807.922,-5.387,10.000},
				{-360807.922,-0.000,10.000}
		};
		
		if (flipped) {
			rightProfile = new SrxMotionProfile(leftPoints.length, leftPoints);
			leftProfile = new SrxMotionProfile(rightPoints.length, rightPoints);
		} else {
			leftProfile = new SrxMotionProfile(leftPoints.length, leftPoints);
			rightProfile = new SrxMotionProfile(rightPoints.length, rightPoints);
		}
	}

}