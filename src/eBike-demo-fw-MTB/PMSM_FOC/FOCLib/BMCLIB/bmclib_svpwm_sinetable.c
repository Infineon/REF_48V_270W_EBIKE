/**
 * @file bmclib_svpwm_sinetable.c
 * @date 09 May, 2019
 *
 * @cond
 *********************************************************************************************************************
 * PMSM_FOC Motor Control Library
 *
 * Copyright (c) 2015-2020, Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice, this list of conditions and the  following
 *   disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *   following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *   Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 *   products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * @endcond
 ***********************************************************************************************************************/
/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include <PMSM_FOC/Configuration/pmsm_foc_config.h>
#include <PMSM_FOC/Configuration/pmsm_foc_const.h>
#include <xmc_common.h>

/*********************************************************************************************************************
 * MACROS
 *********************************************************************************************************************/


/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
/*
 * This is the sine Look-Up Table (LUT) for SVM calculations. Instead, can use CORDIC for the same calculations
 * (save >0.4kbyte Flash).
 * It contains angles of 0� to 60�. Array size 256 or 1024. Angle resolution is 60�/256 = 0.234� or 60�/1024 = 0.0586�.
 */

const uint16_t SIN60_TAB[]=
{
    #if(USER_SVM_SINE_LUT_SIZE == 256)
      /* Look-Up Table (LUT) array size 256 */
    0,135,269,404,538,673,807,942,1076,1211,
    1345,1480,1614,1749,1883,2017,2152,2286,2420,2554,
    2688,
2822,
2956,
3090,
3224,
3358,
3492,
3626,
3760,
3893,
4027,
4160,
4294,
4427,
4560,
4694,
4827,
4960,
5093,
5226,
5359,
5491,
5624,
5756,
5889,
6021,
6153,
6285,
6417,
6549,
6681,
6813,
6944,
7076,
7207,
7338,
7469,
7600,
7731,
7862,
7993,
8123,
8253,
8383,
8513,
8643,
8773,
8903,
9032,
9161,
9290,
9419,
9548,
9677,
9805,
9934,
10062,
10190,
10318,
10445,
10573,
10700,
10827,
10954,
11081,
11207,
11334,
11460,
11586,
11712,
11837,
11963,
12088,
12213,
12337,
12462,
12586,
12711,
12834,
12958,
13082,
13205,
13328,
13451,
13573,
13696,
13818,
13940,
14061,
14183,
14304,
14425,
14546,
14666,
14786,
14906,
15026,
15145,
15265,
15384,
15502,
15621,
15739,
15857,
15974,
16092,
16209,
16326,
16442,
16558,
16674,
16790,
16906,
17021,
17136,
17250,
17364,
17478,
17592,
17705,
17819,
17931,
18044,
18156,
18268,
18379,
18491,
18602,
18712,
18823,
18932,
19042,
19152,
19261,
19369,
19478,
19586,
19693,
19801,
19908,
20015,
20121,
20227,
20333,
20438,
20543,
20648,
20752,
20856,
20960,
21063,
21166,
21268,
21371,
21472,
21574,
21675,
21776,
21876,
21976,
22076,
22175,
22274,
22372,
22470,
22568,
22666,
22763,
22859,
22955,
23051,
23147,
23242,
23336,
23431,
23525,
23618,
23711,
23804,
23896,
23988,
24079,
24170,
24261,
24351,
24441,
24531,
24620,
24708,
24796,
24884,
24972,
25058,
25145,
25231,
25317,
25402,
25487,
25571,
25655,
25738,
25822,
25904,
25986,
26068,
26149,
26230,
26311,
26391,
26470,
26549,
26628,
26706,
26784,
26861,
26938,
27014,
27090,
27166,
27241,
27315,
27390,
27463,
27536,
27609,
27681,
27753,
27824,
27895,
27966,
28036,
28105,
28174,
28242,
28310,
28378


    #else
    /* Look-Up Table (LUT) array size 1024 */
    0,34,67,101,134,168,201,235,268,302,
    335,369,403,436,470,503,537,570,604,637,
671,
704,
738,
771,
805,
838,
872,
906,
939,
973,
1006,
1040,
1073,
1107,
1140,
1174,
1207,
1241,
1274,
1308,
1341,
1375,
1408,
1442,
1475,
1509,
1542,
1576,
1609,
1643,
1676,
1710,
1743,
1777,
1810,
1844,
1877,
1911,
1944,
1978,
2011,
2045,
2078,
2112,
2145,
2179,
2212,
2246,
2279,
2313,
2346,
2379,
2413,
2446,
2480,
2513,
2547,
2580,
2614,
2647,
2680,
2714,
2747,
2781,
2814,
2848,
2881,
2914,
2948,
2981,
3015,
3048,
3081,
3115,
3148,
3182,
3215,
3248,
3282,
3315,
3348,
3382,
3415,
3449,
3482,
3515,
3549,
3582,
3615,
3649,
3682,
3715,
3749,
3782,
3815,
3849,
3882,
3915,
3948,
3982,
4015,
4048,
4082,
4115,
4148,
4181,
4215,
4248,
4281,
4314,
4348,
4381,
4414,
4447,
4481,
4514,
4547,
4580,
4614,
4647,
4680,
4713,
4746,
4780,
4813,
4846,
4879,
4912,
4945,
4979,
5012,
5045,
5078,
5111,
5144,
5177,
5211,
5244,
5277,
5310,
5343,
5376,
5409,
5442,
5475,
5508,
5541,
5574,
5608,
5641,
5674,
5707,
5740,
5773,
5806,
5839,
5872,
5905,
5938,
5971,
6004,
6037,
6070,
6103,
6135,
6168,
6201,
6234,
6267,
6300,
6333,
6366,
6399,
6432,
6465,
6498,
6530,
6563,
6596,
6629,
6662,
6695,
6728,
6760,
6793,
6826,
6859,
6892,
6924,
6957,
6990,
7023,
7055,
7088,
7121,
7154,
7186,
7219,
7252,
7285,
7317,
7350,
7383,
7415,
7448,
7481,
7513,
7546,
7579,
7611,
7644,
7676,
7709,
7742,
7774,
7807,
7839,
7872,
7905,
7937,
7970,
8002,
8035,
8067,
8100,
8132,
8165,
8197,
8230,
8262,
8295,
8327,
8359,
8392,
8424,
8457,
8489,
8521,
8554,
8586,
8619,
8651,
8683,
8716,
8748,
8780,
8813,
8845,
8877,
8909,
8942,
8974,
9006,
9039,
9071,
9103,
9135,
9167,
9200,
9232,
9264,
9296,
9328,
9360,
9393,
9425,
9457,
9489,
9521,
9553,
9585,
9617,
9649,
9681,
9713,
9746,
9778,
9810,
9842,
9874,
9905,
9937,
9969,
10001,
10033,
10065,
10097,
10129,
10161,
10193,
10225,
10257,
10288,
10320,
10352,
10384,
10416,
10448,
10479,
10511,
10543,
10575,
10606,
10638,
10670,
10702,
10733,
10765,
10797,
10828,
10860,
10892,
10923,
10955,
10986,
11018,
11050,
11081,
11113,
11144,
11176,
11207,
11239,
11270,
11302,
11333,
11365,
11396,
11428,
11459,
11491,
11522,
11553,
11585,
11616,
11647,
11679,
11710,
11741,
11773,
11804,
11835,
11867,
11898,
11929,
11960,
11992,
12023,
12054,
12085,
12116,
12147,
12179,
12210,
12241,
12272,
12303,
12334,
12365,
12396,
12427,
12458,
12489,
12520,
12551,
12582,
12613,
12644,
12675,
12706,
12737,
12768,
12799,
12830,
12861,
12891,
12922,
12953,
12984,
13015,
13045,
13076,
13107,
13138,
13168,
13199,
13230,
13260,
13291,
13322,
13352,
13383,
13414,
13444,
13475,
13505,
13536,
13567,
13597,
13628,
13658,
13689,
13719,
13749,
13780,
13810,
13841,
13871,
13902,
13932,
13962,
13993,
14023,
14053,
14084,
14114,
14144,
14174,
14205,
14235,
14265,
14295,
14325,
14356,
14386,
14416,
14446,
14476,
14506,
14536,
14566,
14596,
14626,
14656,
14686,
14716,
14746,
14776,
14806,
14836,
14866,
14896,
14926,
14956,
14985,
15015,
15045,
15075,
15105,
15134,
15164,
15194,
15223,
15253,
15283,
15313,
15342,
15372,
15401,
15431,
15461,
15490,
15520,
15549,
15579,
15608,
15638,
15667,
15697,
15726,
15756,
15785,
15814,
15844,
15873,
15902,
15932,
15961,
15990,
16020,
16049,
16078,
16107,
16136,
16166,
16195,
16224,
16253,
16282,
16311,
16340,
16369,
16399,
16428,
16457,
16486,
16515,
16544,
16572,
16601,
16630,
16659,
16688,
16717,
16746,
16775,
16803,
16832,
16861,
16890,
16918,
16947,
16976,
17005,
17033,
17062,
17090,
17119,
17148,
17176,
17205,
17233,
17262,
17290,
17319,
17347,
17376,
17404,
17433,
17461,
17489,
17518,
17546,
17574,
17603,
17631,
17659,
17687,
17716,
17744,
17772,
17800,
17828,
17857,
17885,
17913,
17941,
17969,
17997,
18025,
18053,
18081,
18109,
18137,
18165,
18193,
18221,
18248,
18276,
18304,
18332,
18360,
18388,
18415,
18443,
18471,
18498,
18526,
18554,
18581,
18609,
18637,
18664,
18692,
18719,
18747,
18774,
18802,
18829,
18857,
18884,
18912,
18939,
18966,
18994,
19021,
19048,
19076,
19103,
19130,
19157,
19184,
19212,
19239,
19266,
19293,
19320,
19347,
19374,
19401,
19428,
19455,
19482,
19509,
19536,
19563,
19590,
19617,
19644,
19671,
19697,
19724,
19751,
19778,
19805,
19831,
19858,
19885,
19911,
19938,
19965,
19991,
20018,
20044,
20071,
20097,
20124,
20150,
20177,
20203,
20229,
20256,
20282,
20309,
20335,
20361,
20387,
20414,
20440,
20466,
20492,
20518,
20545,
20571,
20597,
20623,
20649,
20675,
20701,
20727,
20753,
20779,
20805,
20831,
20857,
20882,
20908,
20934,
20960,
20986,
21011,
21037,
21063,
21089,
21114,
21140,
21165,
21191,
21217,
21242,
21268,
21293,
21319,
21344,
21370,
21395,
21420,
21446,
21471,
21496,
21522,
21547,
21572,
21598,
21623,
21648,
21673,
21698,
21723,
21749,
21774,
21799,
21824,
21849,
21874,
21899,
21924,
21948,
21973,
21998,
22023,
22048,
22073,
22098,
22122,
22147,
22172,
22196,
22221,
22246,
22270,
22295,
22319,
22344,
22369,
22393,
22418,
22442,
22466,
22491,
22515,
22540,
22564,
22588,
22613,
22637,
22661,
22685,
22709,
22734,
22758,
22782,
22806,
22830,
22854,
22878,
22902,
22926,
22950,
22974,
22998,
23022,
23046,
23069,
23093,
23117,
23141,
23165,
23188,
23212,
23236,
23259,
23283,
23306,
23330,
23354,
23377,
23401,
23424,
23447,
23471,
23494,
23518,
23541,
23564,
23588,
23611,
23634,
23657,
23681,
23704,
23727,
23750,
23773,
23796,
23819,
23842,
23865,
23888,
23911,
23934,
23957,
23980,
24003,
24026,
24048,
24071,
24094,
24117,
24139,
24162,
24185,
24207,
24230,
24252,
24275,
24297,
24320,
24342,
24365,
24387,
24410,
24432,
24454,
24477,
24499,
24521,
24543,
24566,
24588,
24610,
24632,
24654,
24676,
24698,
24720,
24742,
24764,
24786,
24808,
24830,
24852,
24874,
24896,
24918,
24939,
24961,
24983,
25004,
25026,
25048,
25069,
25091,
25113,
25134,
25156,
25177,
25199,
25220,
25241,
25263,
25284,
25305,
25327,
25348,
25369,
25390,
25412,
25433,
25454,
25475,
25496,
25517,
25538,
25559,
25580,
25601,
25622,
25643,
25664,
25685,
25705,
25726,
25747,
25768,
25788,
25809,
25830,
25850,
25871,
25892,
25912,
25933,
25953,
25974,
25994,
26014,
26035,
26055,
26076,
26096,
26116,
26136,
26157,
26177,
26197,
26217,
26237,
26257,
26277,
26297,
26317,
26337,
26357,
26377,
26397,
26417,
26437,
26457,
26476,
26496,
26516,
26535,
26555,
26575,
26594,
26614,
26634,
26653,
26673,
26692,
26711,
26731,
26750,
26770,
26789,
26808,
26828,
26847,
26866,
26885,
26904,
26923,
26943,
26962,
26981,
27000,
27019,
27038,
27057,
27076,
27094,
27113,
27132,
27151,
27170,
27188,
27207,
27226,
27244,
27263,
27282,
27300,
27319,
27337,
27356,
27374,
27393,
27411,
27429,
27448,
27466,
27484,
27503,
27521,
27539,
27557,
27575,
27593,
27611,
27629,
27648,
27665,
27683,
27701,
27719,
27737,
27755,
27773,
27791,
27808,
27826,
27844,
27861,
27879,
27897,
27914,
27932,
27949,
27967,
27984,
28002,
28019,
28037,
28054,
28071,
28089,
28106,
28123,
28140,
28157,
28175,
28192,
28209,
28226,
28243,
28260,
28277,
28294,
28311,
28327,
28344,
28361,
28378
    #endif
};
