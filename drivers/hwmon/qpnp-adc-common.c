/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/hwmon.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/spmi.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/platform_device.h>

/* Min ADC code represets 0V */
#define QPNP_VADC_MIN_ADC_CODE			0x6000
/* Max ADC code represents full-scale range of 1.8V */
#define QPNP_VADC_MAX_ADC_CODE			0xA800
#define KELVINMIL_DEGMIL	273160

/* Units for temperature below (on x axis) is in 0.1DegC as
   required by the battery driver. Note the resolution used
   here to compute the table was done for DegC to milli-volts.
   In consideration to limit the size of the table for the given
   temperature range below, the result is linearly interpolated
   and provided to the battery driver in the units desired for
   their framework which is 0.1DegC. True resolution of 0.1DegC
   will result in the below table size to increase by 10 times */
static const struct qpnp_vadc_map_pt adcmap_btm_threshold[] = {
	{ -400 , 1676 },
	{ -390 , 1670 },
	{ -380 , 1664 },
	{ -370 , 1658 },
	{ -360 , 1651 },
	{ -350 , 1645 },
	{ -340 , 1638 },
	{ -330 , 1630 },
	{ -320 , 1623 },
	{ -310 , 1615 },
	{ -300 , 1607 },
	{ -290 , 1599 },
	{ -280 , 1590 },
	{ -270 , 1582 },
	{ -260 , 1573 },
	{ -250 , 1563 },
	{ -240 , 1554 },
	{ -230 , 1544 },
	{ -220 , 1534 },
	{ -210 , 1523 },
	{ -200 , 1513 },
	{ -190 , 1502 },
	{ -180 , 1491 },
	{ -170 , 1479 },
	{ -160 , 1468 },
	{ -150 , 1456 },
	{ -140 , 1444 },
	{ -130 , 1432 },
	{ -120 , 1419 },
	{ -110 , 1406 },
	{ -100 , 1393 },
	{ -90 , 1380 },
	{ -80 , 1367 },
	{ -70 , 1354 },
	{ -60 , 1340 },
	{ -50 , 1326 },
	{ -40 , 1312 },
	{ -30 , 1298 },
	{ -20 , 1284 },
	{ -10 , 1270 },
	{ 0 , 1256 },
	{ 10 , 1241 },
	{ 20 , 1227 },
	{ 30 , 1212 },
	{ 40 , 1198 },
	{ 50 , 1183 },
	{ 60 , 1168 },
	{ 70 , 1154 },
	{ 80 , 1139 },
	{ 90 , 1125 },
	{ 100 , 1110 },
	{ 110 , 1096 },
	{ 120 , 1082 },
	{ 130 , 1067 },
	{ 140 , 1053 },
	{ 150 , 1039 },
	{ 160 , 1025 },
	{ 170 , 1011 },
	{ 180 , 997 },
	{ 190 , 983 },
	{ 200 , 970 },
	{ 210 , 957 },
	{ 220 , 943 },
	{ 230 , 930 },
	{ 240 , 917 },
	{ 250 , 905 },
	{ 260 , 892 },
	{ 270 , 880 },
	{ 280 , 868 },
	{ 290 , 856 },
	{ 300 , 844 },
	{ 310 , 832 },
	{ 320 , 821 },
	{ 330 , 810 },
	{ 340 , 799 },
	{ 350 , 788 },
	{ 360 , 778 },
	{ 370 , 767 },
	{ 380 , 757 },
	{ 390 , 747 },
	{ 400 , 738 },
	{ 410 , 728 },
	{ 420 , 719 },
	{ 430 , 710 },
	{ 440 , 701 },
	{ 450 , 692 },
	{ 460 , 684 },
	{ 470 , 676 },
	{ 480 , 668 },
	{ 490 , 660 },
	{ 500 , 652 },
	{ 510 , 645 },
	{ 520 , 637 },
	{ 530 , 630 },
	{ 540 , 623 },
	{ 550 , 616 },
	{ 560 , 610 },
	{ 570 , 603 },
	{ 580 , 597 },
	{ 590 , 591 },
	{ 600 , 585 },
	{ 610 , 579 },
	{ 620 , 574 },
	{ 630 , 568 },
	{ 640 , 563 },
	{ 650 , 558 },
	{ 660 , 552 },
	{ 670 , 548 },
	{ 680 , 543 },
	{ 690 , 538 },
	{ 700 , 533 },
	{ 710 , 529 },
	{ 720 , 525 },
	{ 730 , 521 },
	{ 740 , 517 },
	{ 750 , 513 },
	{ 760 , 509 },
	{ 770 , 505 },
	{ 780 , 501 },
	{ 790 , 498 },
	{ 800 , 494 },
	{ 810 , 491 },
	{ 820 , 488 },
	{ 830 , 485 },
	{ 840 , 482 },
	{ 850 , 479 },
	{ 860 , 476 },
	{ 870 , 473 },
	{ 880 , 470 },
	{ 890 , 467 },
	{ 900 , 465 },
	{ 910 , 462 },
	{ 920 , 460 },
	{ 930 , 458 },
	{ 940 , 455 },
	{ 950 , 453 },
	{ 960 , 451 },
	{ 970 , 449 },
	{ 980 , 447 },
	{ 990 , 444 },
	{ 1000 , 442 },
	{ 1010 , 441 },
	{ 1020 , 439 },
	{ 1030 , 437 },
	{ 1040 , 435 },
	{ 1050 , 433 },
	{ 1060 , 432 },
	{ 1070 , 430 },
	{ 1080 , 429 },
	{ 1090 , 427 },
	{ 1100 , 425 },
	{ 1110 , 424 },
	{ 1120 , 423 },
	{ 1130 , 421 },
	{ 1140 , 420 },
	{ 1150 , 419 },
	{ 1160 , 417 },
	{ 1170 , 416 },
	{ 1180 , 415 },
	{ 1190 , 414 },
	{ 1200 , 412 },
	{ 1210 , 411 },
	{ 1220 , 410 },
	{ 1230 , 409 },
	{ 1240 , 408 },
    { 1250 , 407 }
};

static const struct qpnp_vadc_map_pt adcmap_qrd_btm_threshold[] = {
	{ -400 , 1676 },
	{ -390 , 1670 },
	{ -380 , 1664 },
	{ -370 , 1658 },
	{ -360 , 1651 },
	{ -350 , 1645 },
	{ -340 , 1638 },
	{ -330 , 1630 },
	{ -320 , 1623 },
	{ -310 , 1615 },
	{ -300 , 1607 },
	{ -290 , 1599 },
	{ -280 , 1590 },
	{ -270 , 1582 },
	{ -260 , 1573 },
	{ -250 , 1563 },
	{ -240 , 1554 },
	{ -230 , 1544 },
	{ -220 , 1534 },
	{ -210 , 1523 },
	{ -200 , 1513 },
	{ -190 , 1502 },
	{ -180 , 1491 },
	{ -170 , 1479 },
	{ -160 , 1468 },
	{ -150 , 1456 },
	{ -140 , 1444 },
	{ -130 , 1432 },
	{ -120 , 1419 },
	{ -110 , 1406 },
	{ -100 , 1393 },
	{ -90 , 1380 },
	{ -80 , 1367 },
	{ -70 , 1354 },
	{ -60 , 1340 },
	{ -50 , 1326 },
	{ -40 , 1312 },
	{ -30 , 1298 },
	{ -20 , 1284 },
	{ -10 , 1270 },
	{ 0 , 1256 },
	{ 10 , 1241 },
	{ 20 , 1227 },
	{ 30 , 1212 },
	{ 40 , 1198 },
	{ 50 , 1183 },
	{ 60 , 1168 },
	{ 70 , 1154 },
	{ 80 , 1139 },
	{ 90 , 1125 },
	{ 100 , 1110 },
	{ 110 , 1096 },
	{ 120 , 1082 },
	{ 130 , 1067 },
	{ 140 , 1053 },
	{ 150 , 1039 },
	{ 160 , 1025 },
	{ 170 , 1011 },
	{ 180 , 997 },
	{ 190 , 983 },
	{ 200 , 970 },
	{ 210 , 957 },
	{ 220 , 943 },
	{ 230 , 930 },
	{ 240 , 917 },
	{ 250 , 905 },
	{ 260 , 892 },
	{ 270 , 880 },
	{ 280 , 868 },
	{ 290 , 856 },
	{ 300 , 844 },
	{ 310 , 832 },
	{ 320 , 821 },
	{ 330 , 810 },
	{ 340 , 799 },
	{ 350 , 788 },
	{ 360 , 778 },
	{ 370 , 767 },
	{ 380 , 757 },
	{ 390 , 747 },
	{ 400 , 738 },
	{ 410 , 728 },
	{ 420 , 719 },
	{ 430 , 710 },
	{ 440 , 701 },
	{ 450 , 692 },
	{ 460 , 684 },
	{ 470 , 676 },
	{ 480 , 668 },
	{ 490 , 660 },
	{ 500 , 652 },
	{ 510 , 645 },
	{ 520 , 637 },
	{ 530 , 630 },
	{ 540 , 623 },
	{ 550 , 616 },
	{ 560 , 610 },
	{ 570 , 603 },
	{ 580 , 597 },
	{ 590 , 591 },
	{ 600 , 585 },
	{ 610 , 579 },
	{ 620 , 574 },
	{ 630 , 568 },
	{ 640 , 563 },
	{ 650 , 558 },
	{ 660 , 552 },
	{ 670 , 548 },
	{ 680 , 543 },
	{ 690 , 538 },
	{ 700 , 533 },
	{ 710 , 529 },
	{ 720 , 525 },
	{ 730 , 521 },
	{ 740 , 517 },
	{ 750 , 513 },
	{ 760 , 509 },
	{ 770 , 505 },
	{ 780 , 501 },
	{ 790 , 498 },
	{ 800 , 494 },
	{ 810 , 491 },
	{ 820 , 488 },
	{ 830 , 485 },
	{ 840 , 482 },
	{ 850 , 479 },
	{ 860 , 476 },
	{ 870 , 473 },
	{ 880 , 470 },
	{ 890 , 467 },
	{ 900 , 465 },
	{ 910 , 462 },
	{ 920 , 460 },
	{ 930 , 458 },
	{ 940 , 455 },
	{ 950 , 453 },
	{ 960 , 451 },
	{ 970 , 449 },
	{ 980 , 447 },
	{ 990 , 444 },
	{ 1000 , 442 },
	{ 1010 , 441 },
	{ 1020 , 439 },
	{ 1030 , 437 },
	{ 1040 , 435 },
	{ 1050 , 433 },
	{ 1060 , 432 },
	{ 1070 , 430 },
	{ 1080 , 429 },
	{ 1090 , 427 },
	{ 1100 , 425 },
	{ 1110 , 424 },
	{ 1120 , 423 },
	{ 1130 , 421 },
	{ 1140 , 420 },
	{ 1150 , 419 },
	{ 1160 , 417 },
	{ 1170 , 416 },
	{ 1180 , 415 },
	{ 1190 , 414 },
	{ 1200 , 412 },
	{ 1210 , 411 },
	{ 1220 , 410 },
	{ 1230 , 409 },
	{ 1240 , 408 },
    { 1250 , 407 }
};

static const struct qpnp_vadc_map_pt adcmap_qrd_skuaa_btm_threshold[] = {
	{ -400 , 1676 },
	{ -390 , 1670 },
	{ -380 , 1664 },
	{ -370 , 1658 },
	{ -360 , 1651 },
	{ -350 , 1645 },
	{ -340 , 1638 },
	{ -330 , 1630 },
	{ -320 , 1623 },
	{ -310 , 1615 },
	{ -300 , 1607 },
	{ -290 , 1599 },
	{ -280 , 1590 },
	{ -270 , 1582 },
	{ -260 , 1573 },
	{ -250 , 1563 },
	{ -240 , 1554 },
	{ -230 , 1544 },
	{ -220 , 1534 },
	{ -210 , 1523 },
	{ -200 , 1513 },
	{ -190 , 1502 },
	{ -180 , 1491 },
	{ -170 , 1479 },
	{ -160 , 1468 },
	{ -150 , 1456 },
	{ -140 , 1444 },
	{ -130 , 1432 },
	{ -120 , 1419 },
	{ -110 , 1406 },
	{ -100 , 1393 },
	{ -90 , 1380 },
	{ -80 , 1367 },
	{ -70 , 1354 },
	{ -60 , 1340 },
	{ -50 , 1326 },
	{ -40 , 1312 },
	{ -30 , 1298 },
	{ -20 , 1284 },
	{ -10 , 1270 },
	{ 0 , 1256 },
	{ 10 , 1241 },
	{ 20 , 1227 },
	{ 30 , 1212 },
	{ 40 , 1198 },
	{ 50 , 1183 },
	{ 60 , 1168 },
	{ 70 , 1154 },
	{ 80 , 1139 },
	{ 90 , 1125 },
	{ 100 , 1110 },
	{ 110 , 1096 },
	{ 120 , 1082 },
	{ 130 , 1067 },
	{ 140 , 1053 },
	{ 150 , 1039 },
	{ 160 , 1025 },
	{ 170 , 1011 },
	{ 180 , 997 },
	{ 190 , 983 },
	{ 200 , 970 },
	{ 210 , 957 },
	{ 220 , 943 },
	{ 230 , 930 },
	{ 240 , 917 },
	{ 250 , 905 },
	{ 260 , 892 },
	{ 270 , 880 },
	{ 280 , 868 },
	{ 290 , 856 },
	{ 300 , 844 },
	{ 310 , 832 },
	{ 320 , 821 },
	{ 330 , 810 },
	{ 340 , 799 },
	{ 350 , 788 },
	{ 360 , 778 },
	{ 370 , 767 },
	{ 380 , 757 },
	{ 390 , 747 },
	{ 400 , 738 },
	{ 410 , 728 },
	{ 420 , 719 },
	{ 430 , 710 },
	{ 440 , 701 },
	{ 450 , 692 },
	{ 460 , 684 },
	{ 470 , 676 },
	{ 480 , 668 },
	{ 490 , 660 },
	{ 500 , 652 },
	{ 510 , 645 },
	{ 520 , 637 },
	{ 530 , 630 },
	{ 540 , 623 },
	{ 550 , 616 },
	{ 560 , 610 },
	{ 570 , 603 },
	{ 580 , 597 },
	{ 590 , 591 },
	{ 600 , 585 },
	{ 610 , 579 },
	{ 620 , 574 },
	{ 630 , 568 },
	{ 640 , 563 },
	{ 650 , 558 },
	{ 660 , 552 },
	{ 670 , 548 },
	{ 680 , 543 },
	{ 690 , 538 },
	{ 700 , 533 },
	{ 710 , 529 },
	{ 720 , 525 },
	{ 730 , 521 },
	{ 740 , 517 },
	{ 750 , 513 },
	{ 760 , 509 },
	{ 770 , 505 },
	{ 780 , 501 },
	{ 790 , 498 },
	{ 800 , 494 },
	{ 810 , 491 },
	{ 820 , 488 },
	{ 830 , 485 },
	{ 840 , 482 },
	{ 850 , 479 },
	{ 860 , 476 },
	{ 870 , 473 },
	{ 880 , 470 },
	{ 890 , 467 },
	{ 900 , 465 },
	{ 910 , 462 },
	{ 920 , 460 },
	{ 930 , 458 },
	{ 940 , 455 },
	{ 950 , 453 },
	{ 960 , 451 },
	{ 970 , 449 },
	{ 980 , 447 },
	{ 990 , 444 },
	{ 1000 , 442 },
	{ 1010 , 441 },
	{ 1020 , 439 },
	{ 1030 , 437 },
	{ 1040 , 435 },
	{ 1050 , 433 },
	{ 1060 , 432 },
	{ 1070 , 430 },
	{ 1080 , 429 },
	{ 1090 , 427 },
	{ 1100 , 425 },
	{ 1110 , 424 },
	{ 1120 , 423 },
	{ 1130 , 421 },
	{ 1140 , 420 },
	{ 1150 , 419 },
	{ 1160 , 417 },
	{ 1170 , 416 },
	{ 1180 , 415 },
	{ 1190 , 414 },
	{ 1200 , 412 },
	{ 1210 , 411 },
	{ 1220 , 410 },
	{ 1230 , 409 },
	{ 1240 , 408 },
    { 1250 , 407 }
};

static const struct qpnp_vadc_map_pt adcmap_qrd_skug_btm_threshold[] = {
	{-200,	1338},
	{-180,	1307},
	{-160,	1276},
	{-140,	1244},
	{-120,	1213},
	{-100,	1182},
	{-80,	1151},
	{-60,	1121},
	{-40,	1092},
	{-20,	1063},
	{0,	1035},
	{20,	1008},
	{40,	982},
	{60,	957},
	{80,	933},
	{100,	910},
	{120,	889},
	{140,	868},
	{160,	848},
	{180,	830},
	{200,	812},
	{220,	795},
	{240,	780},
	{260,	765},
	{280,	751},
	{300,	738},
	{320,	726},
	{340,	714},
	{360,	704},
	{380,	694},
	{400,	684},
	{420,	675},
	{440,	667},
	{460,	659},
	{480,	652},
	{500,	645},
	{520,	639},
	{540,	633},
	{560,	627},
	{580,	622},
	{600,	617},
	{620,	613},
	{640,	608},
	{660,	604},
	{680,	600},
	{700,	597},
	{720,	593},
	{740,	590},
	{760,	587},
	{780,	585},
	{800,	582},
};

/* Voltage to temperature */
static const struct qpnp_vadc_map_pt adcmap_100k_104ef_104fb[] = {
	{1758,	-40},
	{1742,	-35},
	{1719,	-30},
	{1691,	-25},
	{1654,	-20},
	{1608,	-15},
	{1551,	-10},
	{1483,	-5},
	{1404,	0},
	{1315,	5},
	{1218,	10},
	{1114,	15},
	{1007,	20},
	{900,	25},
	{795,	30},
	{696,	35},
	{605,	40},
	{522,	45},
	{448,	50},
	{383,	55},
	{327,	60},
	{278,	65},
	{237,	70},
	{202,	75},
	{172,	80},
	{146,	85},
	{125,	90},
	{107,	95},
	{92,	100},
	{79,	105},
	{68,	110},
	{59,	115},
	{51,	120},
	{44,	125}
};

/* Voltage to temperature */
static const struct qpnp_vadc_map_pt adcmap_150k_104ef_104fb[] = {
	{1738,	-40},
	{1714,	-35},
	{1682,	-30},
	{1641,	-25},
	{1589,	-20},
	{1526,	-15},
	{1451,	-10},
	{1363,	-5},
	{1266,	0},
	{1159,	5},
	{1048,	10},
	{936,	15},
	{825,	20},
	{720,	25},
	{622,	30},
	{533,	35},
	{454,	40},
	{385,	45},
	{326,	50},
	{275,	55},
	{232,	60},
	{195,	65},
	{165,	70},
	{139,	75},
	{118,	80},
	{100,	85},
	{85,	90},
	{73,	95},
	{62,	100},
	{53,	105},
	{46,	110},
	{40,	115},
	{34,	120},
	{30,	125}
};

static int32_t qpnp_adc_map_voltage_temp(const struct qpnp_vadc_map_pt *pts,
		uint32_t tablesize, int32_t input, int64_t *output)
{
	bool descending = 1;
	uint32_t i = 0;

	if (pts == NULL)
		return -EINVAL;

	/* Check if table is descending or ascending */
	if (tablesize > 1) {
		if (pts[0].x < pts[1].x)
			descending = 0;
	}

	while (i < tablesize) {
		if ((descending == 1) && (pts[i].x < input)) {
			/* table entry is less than measured
				value and table is descending, stop */
			break;
		} else if ((descending == 0) &&
				(pts[i].x > input)) {
			/* table entry is greater than measured
				value and table is ascending, stop */
			break;
		} else {
			i++;
		}
	}

	if (i == 0)
		*output = pts[0].y;
	else if (i == tablesize)
		*output = pts[tablesize-1].y;
	else {
		/* result is between search_index and search_index-1 */
		/* interpolate linearly */
		*output = (((int32_t) ((pts[i].y - pts[i-1].y)*
			(input - pts[i-1].x))/
			(pts[i].x - pts[i-1].x))+
			pts[i-1].y);
	}

	return 0;
}

static int32_t qpnp_adc_map_temp_voltage(const struct qpnp_vadc_map_pt *pts,
		uint32_t tablesize, int32_t input, int64_t *output)
{
	bool descending = 1;
	uint32_t i = 0;

	if (pts == NULL)
		return -EINVAL;

	/* Check if table is descending or ascending */
	if (tablesize > 1) {
		if (pts[0].y < pts[1].y)
			descending = 0;
	}

	while (i < tablesize) {
		if ((descending == 1) && (pts[i].y < input)) {
			/* table entry is less than measured
				value and table is descending, stop */
			break;
		} else if ((descending == 0) && (pts[i].y > input)) {
			/* table entry is greater than measured
				value and table is ascending, stop */
			break;
		} else {
			i++;
		}
	}

	if (i == 0) {
		*output = pts[0].x;
	} else if (i == tablesize) {
		*output = pts[tablesize-1].x;
	} else {
		/* result is between search_index and search_index-1 */
		/* interpolate linearly */
		*output = (((int32_t) ((pts[i].x - pts[i-1].x)*
			(input - pts[i-1].y))/
			(pts[i].y - pts[i-1].y))+
			pts[i-1].x);
	}

	return 0;
}

static int64_t qpnp_adc_scale_ratiometric_calib(int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties)
{
	int64_t adc_voltage = 0;
	bool negative_offset = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties)
		return -EINVAL;

	adc_voltage = (adc_code -
		chan_properties->adc_graph[CALIB_RATIOMETRIC].adc_gnd)
		* adc_properties->adc_vdd_reference;
	if (adc_voltage < 0) {
		negative_offset = 1;
		adc_voltage = -adc_voltage;
	}
	do_div(adc_voltage,
		chan_properties->adc_graph[CALIB_RATIOMETRIC].dy);
	if (negative_offset)
		adc_voltage = -adc_voltage;

	return adc_voltage;
}

int32_t qpnp_adc_scale_pmic_therm(struct qpnp_vadc_chip *vadc,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t pmic_voltage = 0;
	bool negative_offset = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties
		|| !adc_chan_result
		|| !chan_properties->adc_graph[CALIB_ABSOLUTE].dy)
		return -EINVAL;

	pmic_voltage = (adc_code -
		chan_properties->adc_graph[CALIB_ABSOLUTE].adc_gnd)
		* chan_properties->adc_graph[CALIB_ABSOLUTE].dx;
	if (pmic_voltage < 0) {
		negative_offset = 1;
		pmic_voltage = -pmic_voltage;
	}
	do_div(pmic_voltage,
		chan_properties->adc_graph[CALIB_ABSOLUTE].dy);
	if (negative_offset)
		pmic_voltage = -pmic_voltage;
	pmic_voltage += chan_properties->adc_graph[CALIB_ABSOLUTE].dx;

	if (pmic_voltage > 0) {
		/* 2mV/K */
		adc_chan_result->measurement = pmic_voltage*
			chan_properties->offset_gain_denominator;

		do_div(adc_chan_result->measurement,
			chan_properties->offset_gain_numerator * 2);
	} else {
		adc_chan_result->measurement = 0;
	}
	/* Change to .001 deg C */
	adc_chan_result->measurement -= KELVINMIL_DEGMIL;
	adc_chan_result->physical = (int32_t)adc_chan_result->measurement;

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_pmic_therm);

int32_t qpnp_adc_scale_millidegc_pmic_voltage_thr(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph btm_param;
	int64_t low_output = 0, high_output = 0;
	int rc = 0, sign = 0;

	rc = qpnp_get_vadc_gain_and_offset(chip, &btm_param, CALIB_ABSOLUTE);
	if (rc < 0) {
		pr_err("Could not acquire gain and offset\n");
		return rc;
	}

	/* Convert to Kelvin and account for voltage to be written as 2mV/K */
	low_output = (param->low_temp + KELVINMIL_DEGMIL) * 2;
	/* Convert to voltage threshold */
	low_output = (low_output - QPNP_ADC_625_UV) * btm_param.dy;
	if (low_output < 0) {
		sign = 1;
		low_output = -low_output;
	}
	do_div(low_output, QPNP_ADC_625_UV);
	if (sign)
		low_output = -low_output;
	low_output += btm_param.adc_gnd;

	sign = 0;
	/* Convert to Kelvin and account for voltage to be written as 2mV/K */
	high_output = (param->high_temp + KELVINMIL_DEGMIL) * 2;
	/* Convert to voltage threshold */
	high_output = (high_output - QPNP_ADC_625_UV) * btm_param.dy;
	if (high_output < 0) {
		sign = 1;
		high_output = -high_output;
	}
	do_div(high_output, QPNP_ADC_625_UV);
	if (sign)
		high_output = -high_output;
	high_output += btm_param.adc_gnd;

	*low_threshold = (uint32_t) low_output;
	*high_threshold = (uint32_t) high_output;
	pr_debug("high_temp:%d, low_temp:%d\n", param->high_temp,
				param->low_temp);
	pr_debug("adc_code_high:%x, adc_code_low:%x\n", *high_threshold,
				*low_threshold);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_millidegc_pmic_voltage_thr);

/* Scales the ADC code to degC using the mapping
 * table for the XO thermistor.
 */
int32_t qpnp_adc_tdkntcg_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t xo_thm = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties
		|| !adc_chan_result)
		return -EINVAL;

	xo_thm = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	qpnp_adc_map_voltage_temp(adcmap_100k_104ef_104fb,
		ARRAY_SIZE(adcmap_100k_104ef_104fb),
		xo_thm, &adc_chan_result->physical);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_tdkntcg_therm);

int32_t qpnp_adc_scale_batt_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t bat_voltage = 0;

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	return qpnp_adc_map_temp_voltage(
			adcmap_btm_threshold,
			ARRAY_SIZE(adcmap_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL(qpnp_adc_scale_batt_therm);

int32_t qpnp_adc_scale_qrd_batt_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t bat_voltage = 0;

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	return qpnp_adc_map_temp_voltage(
			adcmap_qrd_btm_threshold,
			ARRAY_SIZE(adcmap_qrd_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL(qpnp_adc_scale_qrd_batt_therm);

int32_t qpnp_adc_scale_qrd_skuaa_batt_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t bat_voltage = 0;

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	return qpnp_adc_map_temp_voltage(
			adcmap_qrd_skuaa_btm_threshold,
			ARRAY_SIZE(adcmap_qrd_skuaa_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL(qpnp_adc_scale_qrd_skuaa_batt_therm);

int32_t qpnp_adc_scale_qrd_skug_batt_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t bat_voltage = 0;

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	return qpnp_adc_map_temp_voltage(
			adcmap_qrd_skug_btm_threshold,
			ARRAY_SIZE(adcmap_qrd_skug_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL(qpnp_adc_scale_qrd_skug_batt_therm);
int32_t qpnp_adc_scale_therm_pu1(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t therm_voltage = 0;

	therm_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	qpnp_adc_map_voltage_temp(adcmap_150k_104ef_104fb,
		ARRAY_SIZE(adcmap_150k_104ef_104fb),
		therm_voltage, &adc_chan_result->physical);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_therm_pu1);

int32_t qpnp_adc_scale_therm_pu2(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t therm_voltage = 0;

	therm_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	qpnp_adc_map_voltage_temp(adcmap_100k_104ef_104fb,
		ARRAY_SIZE(adcmap_100k_104ef_104fb),
		therm_voltage, &adc_chan_result->physical);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_therm_pu2);

int32_t qpnp_adc_tm_scale_voltage_therm_pu2(struct qpnp_vadc_chip *chip,
					uint32_t reg, int64_t *result)
{
	int64_t adc_voltage = 0;
	struct qpnp_vadc_linear_graph param1;
	int negative_offset;

	qpnp_get_vadc_gain_and_offset(chip, &param1, CALIB_RATIOMETRIC);

	adc_voltage = (reg - param1.adc_gnd) * param1.adc_vref;
	if (adc_voltage < 0) {
		negative_offset = 1;
		adc_voltage = -adc_voltage;
	}

	do_div(adc_voltage, param1.dy);

	qpnp_adc_map_voltage_temp(adcmap_100k_104ef_104fb,
		ARRAY_SIZE(adcmap_100k_104ef_104fb),
		adc_voltage, result);
	if (negative_offset)
		adc_voltage = -adc_voltage;

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_tm_scale_voltage_therm_pu2);

int32_t qpnp_adc_tm_scale_therm_voltage_pu2(struct qpnp_vadc_chip *chip,
				struct qpnp_adc_tm_config *param)
{
	struct qpnp_vadc_linear_graph param1;
	int rc;

	qpnp_get_vadc_gain_and_offset(chip, &param1, CALIB_RATIOMETRIC);

	rc = qpnp_adc_map_temp_voltage(adcmap_100k_104ef_104fb,
		ARRAY_SIZE(adcmap_100k_104ef_104fb),
		param->low_thr_temp, &param->low_thr_voltage);
	if (rc)
		return rc;

	param->low_thr_voltage *= param1.dy;
	do_div(param->low_thr_voltage, param1.adc_vref);
	param->low_thr_voltage += param1.adc_gnd;

	rc = qpnp_adc_map_temp_voltage(adcmap_100k_104ef_104fb,
		ARRAY_SIZE(adcmap_100k_104ef_104fb),
		param->high_thr_temp, &param->high_thr_voltage);
	if (rc)
		return rc;

	param->high_thr_voltage *= param1.dy;
	do_div(param->high_thr_voltage, param1.adc_vref);
	param->high_thr_voltage += param1.adc_gnd;

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_tm_scale_therm_voltage_pu2);

int32_t qpnp_adc_scale_batt_id(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t batt_id_voltage = 0;

	batt_id_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);
	adc_chan_result->physical = batt_id_voltage;
	adc_chan_result->physical = adc_chan_result->measurement;

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_batt_id);

int32_t qpnp_adc_scale_default(struct qpnp_vadc_chip *vadc,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	bool negative_rawfromoffset = 0, negative_offset = 0;
	int64_t scale_voltage = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties
		|| !adc_chan_result)
		return -EINVAL;

	scale_voltage = (adc_code -
		chan_properties->adc_graph[CALIB_ABSOLUTE].adc_gnd)
		* chan_properties->adc_graph[CALIB_ABSOLUTE].dx;
	if (scale_voltage < 0) {
		negative_offset = 1;
		scale_voltage = -scale_voltage;
	}
	do_div(scale_voltage,
		chan_properties->adc_graph[CALIB_ABSOLUTE].dy);
	if (negative_offset)
		scale_voltage = -scale_voltage;
	scale_voltage += chan_properties->adc_graph[CALIB_ABSOLUTE].dx;

	if (scale_voltage < 0) {
		if (adc_properties->bipolar) {
			scale_voltage = -scale_voltage;
			negative_rawfromoffset = 1;
		} else {
			scale_voltage = 0;
		}
	}

	adc_chan_result->measurement = scale_voltage *
				chan_properties->offset_gain_denominator;

	/* do_div only perform positive integer division! */
	do_div(adc_chan_result->measurement,
				chan_properties->offset_gain_numerator);

	if (negative_rawfromoffset)
		adc_chan_result->measurement = -adc_chan_result->measurement;

	/*
	 * Note: adc_chan_result->measurement is in the unit of
	 * adc_properties.adc_reference. For generic channel processing,
	 * channel measurement is a scale/ratio relative to the adc
	 * reference input
	 */
	adc_chan_result->physical = adc_chan_result->measurement;

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_default);

int32_t qpnp_adc_usb_scaler(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph usb_param;

	qpnp_get_vadc_gain_and_offset(chip, &usb_param, CALIB_RATIOMETRIC);

	*low_threshold = param->low_thr * usb_param.dy;
	do_div(*low_threshold, usb_param.adc_vref);
	*low_threshold += usb_param.adc_gnd;

	*high_threshold = param->high_thr * usb_param.dy;
	do_div(*high_threshold, usb_param.adc_vref);
	*high_threshold += usb_param.adc_gnd;

	pr_debug("high_volt:%d, low_volt:%d\n", param->high_thr,
				param->low_thr);
	return 0;
}
EXPORT_SYMBOL(qpnp_adc_usb_scaler);

int32_t qpnp_adc_vbatt_rscaler(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph vbatt_param;
	int rc = 0, sign = 0;
	int64_t low_thr = 0, high_thr = 0;

	rc = qpnp_get_vadc_gain_and_offset(chip, &vbatt_param, CALIB_ABSOLUTE);
	if (rc < 0)
		return rc;

	low_thr = (((param->low_thr/3) - QPNP_ADC_625_UV) *
				vbatt_param.dy);
	if (low_thr < 0) {
		sign = 1;
		low_thr = -low_thr;
	}
	do_div(low_thr, QPNP_ADC_625_UV);
	if (sign)
		low_thr = -low_thr;
	*low_threshold = low_thr + vbatt_param.adc_gnd;

	sign = 0;
	high_thr = (((param->high_thr/3) - QPNP_ADC_625_UV) *
				vbatt_param.dy);
	if (high_thr < 0) {
		sign = 1;
		high_thr = -high_thr;
	}
	do_div(high_thr, QPNP_ADC_625_UV);
	if (sign)
		high_thr = -high_thr;
	*high_threshold = high_thr + vbatt_param.adc_gnd;

	pr_debug("high_volt:%d, low_volt:%d\n", param->high_thr,
				param->low_thr);
	pr_debug("adc_code_high:%x, adc_code_low:%x\n", *high_threshold,
				*low_threshold);
	return 0;
}
EXPORT_SYMBOL(qpnp_adc_vbatt_rscaler);

int32_t qpnp_adc_btm_scaler(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph btm_param;
	int64_t low_output = 0, high_output = 0;
	int rc = 0;

	qpnp_get_vadc_gain_and_offset(chip, &btm_param, CALIB_RATIOMETRIC);

	pr_debug("warm_temp:%d and cool_temp:%d\n", param->high_temp,
				param->low_temp);
	rc = qpnp_adc_map_voltage_temp(
		adcmap_btm_threshold,
		ARRAY_SIZE(adcmap_btm_threshold),
		(param->low_temp),
		&low_output);
	if (rc) {
		pr_debug("low_temp mapping failed with %d\n", rc);
		return rc;
	}

	pr_debug("low_output:%lld\n", low_output);
	low_output *= btm_param.dy;
	do_div(low_output, btm_param.adc_vref);
	low_output += btm_param.adc_gnd;

	rc = qpnp_adc_map_voltage_temp(
		adcmap_btm_threshold,
		ARRAY_SIZE(adcmap_btm_threshold),
		(param->high_temp),
		&high_output);
	if (rc) {
		pr_debug("high temp mapping failed with %d\n", rc);
		return rc;
	}

	pr_debug("high_output:%lld\n", high_output);
	high_output *= btm_param.dy;
	do_div(high_output, btm_param.adc_vref);
	high_output += btm_param.adc_gnd;

	/* btm low temperature correspondes to high voltage threshold */
	*low_threshold = high_output;
	/* btm high temperature correspondes to low voltage threshold */
	*high_threshold = low_output;

	pr_debug("high_volt:%d, low_volt:%d\n", *high_threshold,
				*low_threshold);
	return 0;
}
EXPORT_SYMBOL(qpnp_adc_btm_scaler);

int32_t qpnp_vadc_check_result(int32_t *data)
{
	if (*data < QPNP_VADC_MIN_ADC_CODE)
		*data = QPNP_VADC_MIN_ADC_CODE;
	else if (*data > QPNP_VADC_MAX_ADC_CODE)
		*data = QPNP_VADC_MAX_ADC_CODE;

	return 0;
}
EXPORT_SYMBOL(qpnp_vadc_check_result);

int qpnp_adc_get_revid_version(struct device *dev)
{
	struct pmic_revid_data *revid_data;
	struct device_node *revid_dev_node;

	revid_dev_node = of_parse_phandle(dev->of_node,
						"qcom,pmic-revid", 0);
	if (!revid_dev_node) {
		pr_debug("Missing qcom,pmic-revid property\n");
		return -EINVAL;
	}

	revid_data = get_revid_data(revid_dev_node);
	if (IS_ERR(revid_data)) {
		pr_err("revid error rc = %ld\n", PTR_ERR(revid_data));
		return -EINVAL;
	}

	if ((revid_data->rev1 == PM8941_V3P1_REV1) &&
		(revid_data->rev2 == PM8941_V3P1_REV2) &&
		(revid_data->rev3 == PM8941_V3P1_REV3) &&
		(revid_data->rev4 == PM8941_V3P1_REV4) &&
		(revid_data->pmic_type == PM8941_V3P1_TYPE) &&
		(revid_data->pmic_subtype == PM8941_V3P1_SUBTYPE))
			return QPNP_REV_ID_8941_3_1;
	else if ((revid_data->rev1 == PM8226_V2P1_REV1) &&
		(revid_data->rev2 == PM8226_V2P1_REV2) &&
		(revid_data->rev3 == PM8226_V2P1_REV3) &&
		(revid_data->rev4 == PM8226_V2P1_REV4) &&
		(revid_data->pmic_type == PM8226_V2P1_TYPE) &&
		(revid_data->pmic_subtype == PM8226_V2P1_SUBTYPE))
			return QPNP_REV_ID_8026_2_1;
	else if ((revid_data->rev1 == PM8226_V2P0_REV1) &&
		(revid_data->rev2 == PM8226_V2P0_REV2) &&
		(revid_data->rev3 == PM8226_V2P0_REV3) &&
		(revid_data->rev4 == PM8226_V2P0_REV4) &&
		(revid_data->pmic_type == PM8226_V2P0_TYPE) &&
		(revid_data->pmic_subtype == PM8226_V2P0_SUBTYPE))
			return QPNP_REV_ID_8026_2_0;
	else if ((revid_data->rev1 == PM8226_V1P0_REV1) &&
		(revid_data->rev2 == PM8226_V1P0_REV2) &&
		(revid_data->rev3 == PM8226_V1P0_REV3) &&
		(revid_data->rev4 == PM8226_V1P0_REV4) &&
		(revid_data->pmic_type == PM8226_V1P0_TYPE) &&
		(revid_data->pmic_subtype == PM8226_V1P0_SUBTYPE))
			return QPNP_REV_ID_8026_1_0;
	else if ((revid_data->rev1 == PM8110_V1P0_REV1) &&
		(revid_data->rev2 == PM8110_V1P0_REV2) &&
		(revid_data->rev3 == PM8110_V1P0_REV3) &&
		(revid_data->rev4 == PM8110_V1P0_REV4) &&
		(revid_data->pmic_type == PM8110_V1P0_TYPE) &&
		(revid_data->pmic_subtype == PM8110_V1P0_SUBTYPE))
			return QPNP_REV_ID_8110_1_0;
	else
		return -EINVAL;
}
EXPORT_SYMBOL(qpnp_adc_get_revid_version);

int32_t qpnp_adc_get_devicetree_data(struct spmi_device *spmi,
			struct qpnp_adc_drv *adc_qpnp)
{
	struct device_node *node = spmi->dev.of_node;
	struct resource *res;
	struct device_node *child;
	struct qpnp_adc_amux *adc_channel_list;
	struct qpnp_adc_properties *adc_prop;
	struct qpnp_adc_amux_properties *amux_prop;
	int count_adc_channel_list = 0, decimation, rc = 0, i = 0;

	if (!node)
		return -EINVAL;

	for_each_child_of_node(node, child)
		count_adc_channel_list++;

	if (!count_adc_channel_list) {
		pr_err("No channel listing\n");
		return -EINVAL;
	}

	adc_qpnp->spmi = spmi;

	adc_prop = devm_kzalloc(&spmi->dev, sizeof(struct qpnp_adc_properties),
					GFP_KERNEL);
	if (!adc_prop) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}
	adc_channel_list = devm_kzalloc(&spmi->dev,
		((sizeof(struct qpnp_adc_amux)) * count_adc_channel_list),
				GFP_KERNEL);
	if (!adc_channel_list) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	amux_prop = devm_kzalloc(&spmi->dev,
		sizeof(struct qpnp_adc_amux_properties) +
		sizeof(struct qpnp_vadc_chan_properties), GFP_KERNEL);
	if (!amux_prop) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	adc_qpnp->adc_channels = adc_channel_list;
	adc_qpnp->amux_prop = amux_prop;

	for_each_child_of_node(node, child) {
		int channel_num, scaling, post_scaling, hw_settle_time;
		int fast_avg_setup, calib_type, rc;
		const char *calibration_param, *channel_name;

		channel_name = of_get_property(child,
				"label", NULL) ? : child->name;
		if (!channel_name) {
			pr_err("Invalid channel name\n");
			return -EINVAL;
		}

		rc = of_property_read_u32(child, "reg", &channel_num);
		if (rc) {
			pr_err("Invalid channel num\n");
			return -EINVAL;
		}
		rc = of_property_read_u32(child, "qcom,decimation",
								&decimation);
		if (rc) {
			pr_err("Invalid channel decimation property\n");
			return -EINVAL;
		}
		rc = of_property_read_u32(child,
				"qcom,pre-div-channel-scaling", &scaling);
		if (rc) {
			pr_err("Invalid channel scaling property\n");
			return -EINVAL;
		}
		rc = of_property_read_u32(child,
				"qcom,scale-function", &post_scaling);
		if (rc) {
			pr_err("Invalid channel post scaling property\n");
			return -EINVAL;
		}
		rc = of_property_read_u32(child,
				"qcom,hw-settle-time", &hw_settle_time);
		if (rc) {
			pr_err("Invalid channel hw settle time property\n");
			return -EINVAL;
		}
		rc = of_property_read_u32(child,
				"qcom,fast-avg-setup", &fast_avg_setup);
		if (rc) {
			pr_err("Invalid channel fast average setup\n");
			return -EINVAL;
		}
		rc = of_property_read_string(child, "qcom,calibration-type",
							&calibration_param);
		if (rc) {
			pr_err("Invalid calibration type\n");
			return -EINVAL;
		}
		if (!strncmp(calibration_param, "absolute", 8))
			calib_type = CALIB_ABSOLUTE;
		else if (!strncmp(calibration_param, "ratiometric", 11))
			calib_type = CALIB_RATIOMETRIC;
		else {
			pr_err("%s: Invalid calibration property\n", __func__);
			return -EINVAL;
		}
		/* Individual channel properties */
		adc_channel_list[i].name = (char *)channel_name;
		adc_channel_list[i].channel_num = channel_num;
		adc_channel_list[i].chan_path_prescaling = scaling;
		adc_channel_list[i].adc_decimation = decimation;
		adc_channel_list[i].adc_scale_fn = post_scaling;
		adc_channel_list[i].hw_settle_time = hw_settle_time;
		adc_channel_list[i].fast_avg_setup = fast_avg_setup;
		i++;
	}

	/* Get the ADC VDD reference voltage and ADC bit resolution */
	rc = of_property_read_u32(node, "qcom,adc-vdd-reference",
			&adc_prop->adc_vdd_reference);
	if (rc) {
		pr_err("Invalid adc vdd reference property\n");
		return -EINVAL;
	}
	rc = of_property_read_u32(node, "qcom,adc-bit-resolution",
			&adc_prop->bitresolution);
	if (rc) {
		pr_err("Invalid adc bit resolution property\n");
		return -EINVAL;
	}
	adc_qpnp->adc_prop = adc_prop;

	/* Get the peripheral address */
	res = spmi_get_resource(spmi, 0, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("No base address definition\n");
		return -EINVAL;
	}

	adc_qpnp->slave = spmi->sid;
	adc_qpnp->offset = res->start;

	/* Register the ADC peripheral interrupt */
	adc_qpnp->adc_irq_eoc = spmi_get_irq_byname(spmi, NULL,
						"eoc-int-en-set");
	if (adc_qpnp->adc_irq_eoc < 0) {
		pr_err("Invalid irq\n");
		return -ENXIO;
	}

	init_completion(&adc_qpnp->adc_rslt_completion);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_get_devicetree_data);
