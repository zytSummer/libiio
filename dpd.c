// SPDX-License-Identifier: LGPL-2.1-or-later
/*
 * libiio - Library for interfacing industrial I/O (IIO) devices
 *
 * Copyright (C) 2014 Analog Devices, Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 */

#include "debug.h"
#include "iio-config.h"
#include "iio-private.h"
#include "sort.h"
#include "dpd_t_p.h"
#include "dpd.h"
#include "dpd_top.h"
#include "dpd_utils.h"
#include "dpd_t.h"

#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <time.h>


#define IIO_DPD_BUF_SIZE 128

static ssize_t _dpd_TrackCfg_attr_20_show(char *dst);
static ssize_t _dpd_TrackCfg_attr_20_store(const char *dst);

static ssize_t _dpd_TrackCfg_attr_21_show(char *dst);
static ssize_t _dpd_TrackCfg_attr_21_store(const char *dst);

static ssize_t _dpd_TrackCfg_attr_25_show(char *dst);
static ssize_t _dpd_TrackCfg_attr_25_store(const char *dst);

static ssize_t _dpd_dev_attr_12_show(char *dst);
static ssize_t _dpd_dev_attr_12_store(const char *src);

static ssize_t _dpd_dev_attr_13_show(char *dst);

static ssize_t _dpd_dev_attr_14_show(char *dst);
static ssize_t _dpd_dev_attr_14_store(const char *src);

/* generate the global dpd data */
DECLARE_IIO_DPD_DATA;

#ifndef __GNUC__
#define __typeof__ typeof
#endif

/* create dpd channel TrackCfg attribute */
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, numFilterCoefficients, 0);
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, numFilterOutput, 1);
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, pdEstCtrl.pdEstIterStep, 2);
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, pdEstCtrl.pdEstPerIter, 3);
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, pdEstCtrl.xCorrLagRange, 4);
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, pdEstCtrl.intDelayRangeLoop, 5);
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, filterSel, 6);
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, txPwrSelect, 7);
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, delta, 8);
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, decayP, 9);
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, mThreshold, 10);
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, indirectErrSamples, 11);
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, satThresh, 12);
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, outlierThresh, 13);
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, satXCntLimit, 14);
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, satYCntLimit, 15);
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, errCntLimit, 16);
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, txMinAvgSignalLevel, 17);
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, orxMinAvgSignalLevel, 18);
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, direct, 19);
IIO_DPD_ADD_UNIQUE_CHAN_ATTR(TrackCfg, indirectRegValue, _dpd_TrackCfg_attr_20_show, _dpd_TrackCfg_attr_20_store, 20);
IIO_DPD_ADD_UNIQUE_CHAN_ATTR(TrackCfg, directRegValue, _dpd_TrackCfg_attr_21_show, _dpd_TrackCfg_attr_21_store, 21);
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, mu, 22);
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, magGain, 23);
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, absScale, 24);
IIO_DPD_ADD_UNIQUE_CHAN_ATTR(TrackCfg, absOffset, _dpd_TrackCfg_attr_25_show, _dpd_TrackCfg_attr_25_store, 25);
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, linearTerm, 26);

ADD_CHAN_ATTR_ARRAY_ELEMENT_START(TrackCfg)
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, numFilterCoefficients, 0),
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, numFilterOutput, 1),
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, pdEstCtrl.pdEstIterStep, 2),
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, pdEstCtrl.pdEstPerIter, 3),
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, pdEstCtrl.xCorrLagRange, 4),
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, pdEstCtrl.intDelayRangeLoop, 5),
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, filterSel, 6),
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, txPwrSelect, 7),
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, delta, 8),
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, decayP, 9),
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, mThreshold, 10),
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, indirectErrSamples, 11),
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, satThresh, 12),
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, outlierThresh, 13),
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, satXCntLimit, 14),
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, satYCntLimit, 15),
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, errCntLimit, 16),
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, txMinAvgSignalLevel, 17),
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, orxMinAvgSignalLevel, 18),
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, direct, 19),
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, indirectRegValue, 20),
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, directRegValue, 21),
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, mu, 22),
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, magGain, 23),
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, absScale, 24),
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, absOffset, 25),
ADD_CHAN_ATTR_ARRAY_ELEMENT(TrackCfg, linearTerm, 26),
ADD_CHAN_ATTR_ARRAY_ELEMENT_END(TrackCfg);

/* generate dpd channel ModelDesc attribute */
IIO_DPD_ADD_CHAN_ATTR(DpdModelDesc, features, 0);
IIO_DPD_ADD_CHAN_ATTR(DpdModelDesc, dpdPartial.partial, 1);
IIO_DPD_ADD_CHAN_ATTR(DpdModelDesc, dpdPartial.updateOrder, 2);
IIO_DPD_ADD_CHAN_ATTR(DpdModelDesc, dpdPartial.modelIndex, 3);
IIO_DPD_ADD_CHAN_ATTR(DpdModelDesc, mode, 4);     
IIO_DPD_ADD_CHAN_ATTR(DpdModelDesc, actDepth, 5); 
IIO_DPD_ADD_CHAN_ATTR(DpdModelDesc, lutIDelay, 6);
IIO_DPD_ADD_CHAN_ATTR(DpdModelDesc, lutJDelay, 7);

ADD_CHAN_ATTR_ARRAY_ELEMENT_START(DpdModelDesc)
ADD_CHAN_ATTR_ARRAY_ELEMENT(DpdModelDesc, features, 0),
ADD_CHAN_ATTR_ARRAY_ELEMENT(DpdModelDesc, dpdPartial.partial, 1),
ADD_CHAN_ATTR_ARRAY_ELEMENT(DpdModelDesc, dpdPartial.updateOrder, 2),
ADD_CHAN_ATTR_ARRAY_ELEMENT(DpdModelDesc, dpdPartial.modelIndex, 3),
ADD_CHAN_ATTR_ARRAY_ELEMENT(DpdModelDesc, mode, 4),
ADD_CHAN_ATTR_ARRAY_ELEMENT(DpdModelDesc, actDepth, 5),
ADD_CHAN_ATTR_ARRAY_ELEMENT(DpdModelDesc, lutIDelay, 6),
ADD_CHAN_ATTR_ARRAY_ELEMENT(DpdModelDesc, lutJDelay, 7),
ADD_CHAN_ATTR_ARRAY_ELEMENT_END(DpdModelDesc);

/* generate dpd channel ActModelCfg */
IIO_DPD_ADD_CHAN_ATTR(ActModelCfg, txCh, 0);
IIO_DPD_ADD_CHAN_ATTR(ActModelCfg, gmpDdrMode, 1);
IIO_DPD_ADD_CHAN_ATTR(ActModelCfg, ddrOutShift, 2);
IIO_DPD_ADD_CHAN_ATTR(ActModelCfg, gmpOutShift, 3);
IIO_DPD_ADD_CHAN_ATTR(ActModelCfg, magGain, 4);
IIO_DPD_ADD_CHAN_ATTR(ActModelCfg, modelIndex, 5);
IIO_DPD_ADD_CHAN_ATTR(ActModelCfg, bankIndex, 6);
IIO_DPD_ADD_CHAN_ATTR(ActModelCfg, actDepth, 7);
IIO_DPD_ADD_CHAN_ATTR(ActModelCfg, lutLinearTerm, 8);
IIO_DPD_ADD_CHAN_ATTR(ActModelCfg, lutInterpMagRound, 9);
IIO_DPD_ADD_CHAN_ATTR(ActModelCfg, lutInterpBypass, 10);
IIO_DPD_ADD_CHAN_ATTR(ActModelCfg, lutEn, 11);
IIO_DPD_ADD_CHAN_ATTR(ActModelCfg, ddr_j_prog_delay_from_ddr_i, 12);
IIO_DPD_ADD_CHAN_ATTR(ActModelCfg, ddr_i_prog_delay_from_gmp_i, 13);
IIO_DPD_ADD_CHAN_ATTR(ActModelCfg, gmp_1_j_prog_delay_from_gmp_j, 14);
IIO_DPD_ADD_CHAN_ATTR(ActModelCfg, gmp_1_i_prog_delay_from_gmp_i, 15);
IIO_DPD_ADD_CHAN_ATTR(ActModelCfg, gmp_j_prog_delay, 16);
IIO_DPD_ADD_CHAN_ATTR(ActModelCfg, gmp_i_prog_delay, 17);

ADD_CHAN_ATTR_ARRAY_ELEMENT_START(ActModelCfg)
ADD_CHAN_ATTR_ARRAY_ELEMENT(ActModelCfg, txCh, 0),
ADD_CHAN_ATTR_ARRAY_ELEMENT(ActModelCfg, gmpDdrMode, 1),
ADD_CHAN_ATTR_ARRAY_ELEMENT(ActModelCfg, ddrOutShift, 2),
ADD_CHAN_ATTR_ARRAY_ELEMENT(ActModelCfg, gmpOutShift, 3),
ADD_CHAN_ATTR_ARRAY_ELEMENT(ActModelCfg, magGain, 4),
ADD_CHAN_ATTR_ARRAY_ELEMENT(ActModelCfg, modelIndex, 5),
ADD_CHAN_ATTR_ARRAY_ELEMENT(ActModelCfg, bankIndex, 6),
ADD_CHAN_ATTR_ARRAY_ELEMENT(ActModelCfg, actDepth, 7),
ADD_CHAN_ATTR_ARRAY_ELEMENT(ActModelCfg, lutLinearTerm, 8),
ADD_CHAN_ATTR_ARRAY_ELEMENT(ActModelCfg, lutInterpMagRound, 9),
ADD_CHAN_ATTR_ARRAY_ELEMENT(ActModelCfg, lutInterpBypass, 10),
ADD_CHAN_ATTR_ARRAY_ELEMENT(ActModelCfg, lutEn, 11),
ADD_CHAN_ATTR_ARRAY_ELEMENT(ActModelCfg, ddr_j_prog_delay_from_ddr_i, 12),
ADD_CHAN_ATTR_ARRAY_ELEMENT(ActModelCfg, ddr_i_prog_delay_from_gmp_i, 13),
ADD_CHAN_ATTR_ARRAY_ELEMENT(ActModelCfg, gmp_1_j_prog_delay_from_gmp_j, 14),
ADD_CHAN_ATTR_ARRAY_ELEMENT(ActModelCfg, gmp_1_i_prog_delay_from_gmp_i, 15),
ADD_CHAN_ATTR_ARRAY_ELEMENT(ActModelCfg, gmp_j_prog_delay, 16),
ADD_CHAN_ATTR_ARRAY_ELEMENT(ActModelCfg, gmp_i_prog_delay, 17),
ADD_CHAN_ATTR_ARRAY_ELEMENT_END(ActModelCfg);

IIO_DPD_ADD_DEV_CHAN(TrackCfg,0);
IIO_DPD_ADD_DEV_CHAN(DpdModelDesc,1);
IIO_DPD_ADD_DEV_CHAN(ActModelCfg,2);

IIO_DPD_ADD_DEV_DEFAULT_ATTR(actLutSatFlag,3);
IIO_DPD_ADD_DEV_DEFAULT_ATTR(capCfg.capDepth,4);
IIO_DPD_ADD_DEV_DEFAULT_ATTR(capCfg.capBatch,5);
IIO_DPD_ADD_DEV_DEFAULT_ATTR(pathDelayMode,6);
IIO_DPD_ADD_DEV_DEFAULT_ATTR(alignedSampleCount,7);
IIO_DPD_ADD_DEV_DEFAULT_ATTR(featureSampleCount,8);
IIO_DPD_ADD_DEV_DEFAULT_ATTR(procState,9);
IIO_DPD_ADD_DEV_DEFAULT_ATTR(iterCount,10);
IIO_DPD_ADD_DEV_DEFAULT_ATTR(direct,11);
IIO_DPD_ADD_DEV_UNIQUE_ATTR(name, _dpd_dev_attr_12_show, _dpd_dev_attr_12_store, 12);
IIO_DPD_ADD_DEV_UNIQUE_ATTR(version, _dpd_dev_attr_13_show, NULL, 13);
/* the attribute reg is a debug reg, it is not good to implement the reg rd/wr in this way */
IIO_DPD_ADD_DEV_UNIQUE_ATTR(direct_reg_access, _dpd_dev_attr_14_show, _dpd_dev_attr_14_store, 14);

ADD_DEV_ATTR_ARRAY_ELEMENT_START()
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_CHAN, TrackCfg, 0),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_CHAN, DpdModelDesc, 1),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_CHAN, ActModelCfg, 2),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, actLutSatFlag, 3),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, capCfg.capDepth, 4),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, capCfg.capBatch, 5),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, pathDelayMode, 6),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, alignedSampleCount, 7),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, featureSampleCount, 8),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, procState, 9),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, iterCount, 10),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, direct, 11),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, name, 12),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, version, 13),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, direct_reg_access, 14),
ADD_DEV_ATTR_ARRAY_ELEMENT_END();


struct iio_dpd_device_data {
	int fd;
	struct iio_dpd_dev_attr *pdev_attr;
	uint32_t dev_attr_cnt;
};

static struct iio_dpd_device_data dpd_device_data;

static ssize_t _dpd_TrackCfg_attr_20_show(char *dst) 
{ 
	ssize_t ret; 
	uint32_t val;
	val = (uint32_t)(dpdData.pTrackCfg->indirectRegValue * 1000.0);
	ret = iio_snprintf(dst, 128, "0x%08x", val); 
	if (ret > 0) 
		dst[ret] = '\0'; 
	else 
		dst[0] = '\0'; 
	return ret ? ret : -5; 
}

static ssize_t _dpd_TrackCfg_attr_20_store(const char *src) 
{ 
	uint32_t val = 0; 
	char *end; ssize_t ret=0; 
	val = strtoul(src, &end, 16); 
	ret = strlen(src); 
	dpdData.pTrackCfg->indirectRegValue = (double)val/1000.0; 
	return ret ? ret : -5; 
}

static ssize_t _dpd_TrackCfg_attr_21_show(char *dst) 
{ 
	ssize_t ret; 
	uint32_t val;
	val = (uint32_t)(dpdData.pTrackCfg->directRegValue * 1000.0);
	ret = iio_snprintf(dst, 128, "0x%08x", val); 
	if (ret > 0) 
		dst[ret] = '\0'; 
	else 
		dst[0] = '\0'; 
	return ret ? ret : -5; 
} 

static ssize_t _dpd_TrackCfg_attr_21_store(const char *src) 
{ 
	uint32_t val = 0; 
	char *end; ssize_t ret=0; 
	val = strtoul(src, &end, 16); 
	ret = strlen(src); 
	dpdData.pTrackCfg->directRegValue = (double)val/1000.0; 
	return ret ? ret : -5; 
}

static ssize_t _dpd_TrackCfg_attr_25_show(char *dst) 
{ 
	ssize_t ret; 
	uint32_t val;
	val = (uint32_t)(dpdData.pTrackCfg->absOffset * 1000.0);
	ret = iio_snprintf(dst, 128, "0x%08x", val); 
	if (ret > 0) 
		dst[ret] = '\0'; 
	else 
		dst[0] = '\0'; 
	return ret ? ret : -5; 
} 

static ssize_t _dpd_TrackCfg_attr_25_store(const char *src) 
{ 
	uint32_t val = 0; 
	char *end; ssize_t ret=0; 
	val = strtoul(src, &end, 16); 
	ret = strlen(src); 
	dpdData.pTrackCfg->absOffset = (double)val/1000.0; 
	return ret ? ret : -5; 
}

static ssize_t _dpd_dev_attr_14_show(char *dst)
{
	uint32_t val;
	uint32_t rd_val;
	uint32_t tmp_ret;
	char buf[IIO_DPD_BUF_SIZE] = {0,};
	char *end;
	char file_path[IIO_DPD_ATTR_NAME_LEN] = {0,};
	FILE *f;
	ssize_t ret = 0;

	iio_snprintf(file_path, sizeof(file_path), "%s/%s/%s", DPD_TMPFS_PATH, DPD_DEVICE_PATH, "reg_addr");
	
	f = fopen(file_path, "re");
	if (!f)
		return -EIO;

	ret = fread(buf, 1, sizeof(buf)-1, f);
	val = strtoul(buf, &end, 16);
	tmp_ret = dpd_hw_mem_read(val, &rd_val);
	if (!tmp_ret)
	{
		memset(buf, 0x00, sizeof(IIO_DPD_BUF_SIZE));
		ret = iio_snprintf(dst, IIO_DPD_BUF_SIZE, "0x%08x", rd_val);
	}
	else
	{
		ret = tmp_ret;
	}
	fclose(f);

	if (ret > 0) 
	{
		dst[ret] = '\0';
	}
	else
		dst[0] = '\0';
	return ret ? ret : -EIO;
}

static ssize_t _dpd_dev_attr_14_store(const char *src)
{
	uint32_t val[2] = {0,};
	ssize_t ret = 0;
	uint32_t argc;
	char *str_tmp = NULL;
	char *var[2] = {NULL, NULL};
	char *rest = NULL;
	char *end;
	FILE *f;
	char file_path[IIO_DPD_ATTR_NAME_LEN] = {0,};

	str_tmp = iio_strdup(src);
	for (argc = 0; argc < 2; argc ++)
	{
		var[argc] = iio_strtok_r(str_tmp, " ", &rest); 
		if (var[argc])
			val[argc] = strtoul(var[argc], &end, 16);
		else 
		{
			argc --;
			break;
		}
	}

	iio_snprintf(file_path, sizeof(file_path), "%s/%s/%s", DPD_TMPFS_PATH, DPD_DEVICE_PATH, "reg_addr");
	
	f = fopen(file_path, "we");
	if (!f)
		return -EIO;

	fwrite(var[0], 1, strlen(var[0]), f);
	fclose(f);

	if (argc == 0) 
	{
		return -EFAULT;
	} 
	else if (argc == 1) 
	{
		ret = strlen(src);
	}
	else if (argc == 2) 
	{
		dpd_hw_mem_write(val[0], val[1]);
		ret = strlen(src);
	}
	else
	{
		ret = -EFAULT;
	}
	
	return ret;
}

static ssize_t _dpd_dev_attr_13_show(char *dst)
{
	uint32_t val;
	char file_path[IIO_DPD_ATTR_NAME_LEN] = {0,};
	FILE *f;
	ssize_t ret = 0;

	val = dpd_read_ipVersion();

	ret = iio_snprintf(dst, IIO_DPD_BUF_SIZE, "0x%08x", val);

	iio_snprintf(file_path, sizeof(file_path), "%s/%s/%s", DPD_TMPFS_PATH, DPD_DEVICE_PATH, "version");
	
	f = fopen(file_path, "re");
	if (!f)
		return -EIO;

	fwrite(dst, 1, sizeof(dst)-1, f);
	fclose(f);

	if (ret > 0) 
	{
		dst[ret] = '\0';
	}
	else
		dst[0] = '\0';
	return ret ? ret : -EIO;
}


static ssize_t _dpd_dev_attr_12_show(char *dst)
{
	char file_path[IIO_DPD_ATTR_NAME_LEN] = {0,};
	FILE *f;
	ssize_t ret = 0;

	iio_snprintf(file_path, sizeof(file_path), "%s/%s/%s", DPD_TMPFS_PATH, DPD_DEVICE_PATH, "name");
	
	f = fopen(file_path, "re");
	if (!f)
		return -EIO;

	ret = fread(dst, 1, sizeof(dst)-1, f);
	fclose(f);
	if (ret > 0) 
	{
		dst[ret] = '\0';
	}
	else
		dst[0] = '\0';
	return ret ? ret : -EIO;
}

static ssize_t _dpd_dev_attr_12_store(const char *src)
{
	char file_path[IIO_DPD_ATTR_NAME_LEN] = {0,};
	char *buf;
	FILE *f;
	ssize_t ret = 0;

	iio_snprintf(file_path, sizeof(file_path), "%s/%s/%s", DPD_TMPFS_PATH, DPD_DEVICE_PATH, "name");
	
	f = fopen(file_path, "we");
	if (!f)
		return -EIO;
	buf = iio_strdup(src);
	ret = fwrite(buf, 1, sizeof(buf)-1, f);
	fclose(f);
	return ret ? ret : -EIO;
}

static int _dpd_dev_create_fs(void)
{
	int ret;
	char cmd[100] = {0,};

	/* mount the tmpfs to the mount point /tmp folder */
	iio_snprintf(cmd, 100, "mount -t tmpfs -o size=10m tmpfs %s", DPD_TMPFS_PATH);
	ret = system(cmd);

	memset(cmd, 0x00, sizeof(cmd));
	iio_snprintf(cmd, 100, "mkdir -p %s/%s", DPD_TMPFS_PATH, DPD_DEVICE_PATH);
	ret = system(cmd);

	return ret;
}

static int _dpd_dev_chan_create_file(struct iio_dpd_channel *p_chan)
{
	int ret;
	char file_name[IIO_DPD_ATTR_NAME_LEN] = {0,};
	uint32_t lp;
	struct iio_dpd_attr *p_attr = NULL;

	char cmd[100] = {0,};

	if (!p_chan)
	{
		IIO_ERROR("NULL input parameter pointer!\n");
		return -EFAULT; 
	}

	if (!p_chan->pp_attr_array)
	{
		IIO_WARNING("NULL attr in channel[%s]!\n", p_chan->name);
		return -EFAULT; 
	}

	for (lp = 0; (p_attr = p_chan->pp_attr_array[lp]) != NULL; lp ++)
	{
		memset(file_name, 0x00, sizeof(file_name));
		iio_snprintf(file_name, sizeof(file_name), "in_%s_%s", p_chan->name, p_attr->name);
		iio_snprintf(p_attr->file_name, sizeof(file_name), "%s", file_name);
		iio_snprintf(cmd, sizeof(cmd), "touch %s/%s/%s", DPD_TMPFS_PATH, DPD_DEVICE_PATH, file_name);
		ret = system(cmd);
	}

	p_chan->attr_cnt = lp;

	return ret;
}

static int _dpd_dev_create_attr_file(struct iio_dpd_attr *p_attr)
{
	int ret;
	char cmd[100] = {0,};

	if (!p_attr)
	{
		IIO_ERROR("NULL input parameter pointer!\n");
		return -EFAULT; 
	}

	iio_snprintf(cmd, sizeof(cmd), "touch %s/%s/%s", DPD_TMPFS_PATH, DPD_DEVICE_PATH, p_attr->name);
	iio_snprintf(p_attr->file_name, sizeof(cmd), "%s", p_attr->name);
	ret = system(cmd);

	return ret;
}

static int _dpd_dev_attribut_init(void)
{
	int ret = 0;
	int tmp_ret;
	uint32_t type;
	uint32_t lp = 0;

	ret = _dpd_dev_create_fs();
	if (ret)
	{
		IIO_ERROR("Create dpd fs failed!\n");
		return ret;
	}

	for (lp = 0; lp < IIO_DPD_MAX_CHAN_ATTR_CNT; lp ++) 
	{
		if (!iio_dpd_dev_array[lp].pElement)
		{
			/* reach the end of the array */
			break;
		}
		type = iio_dpd_dev_array[lp].attr_type;
		if(type == TYPE_IS_ATTR)
		{
			tmp_ret = _dpd_dev_create_attr_file(iio_dpd_dev_array[lp].pElement);
			if(tmp_ret)
			{
				/* skip this error and continue */
				IIO_WARNING("Cann't create dpd device attribute[%d]\n", lp);
				ret |= 1<<lp;
			}
		}
		else if (type == TYPE_IS_CHAN)
		{
			tmp_ret = _dpd_dev_chan_create_file(iio_dpd_dev_array[lp].pElement);
			if(tmp_ret)
			{
				/* skip this error and continue */
				IIO_WARNING("Cann't create dpd device channel attribute[%d]\n", lp);
				ret |= 1<<lp;
			}
		}
		else
		{
			IIO_WARNING("wrong dpd device attribute[%d] type[%d]\n", lp, type);
			ret |= 1<<lp;
		}
	}

	_dpd_dev_attr_12_store("dpd");
	dpd_device_data.pdev_attr = iio_dpd_dev_array;
	dpd_device_data.dev_attr_cnt = lp;

	return ret;
}

static struct iio_dpd_attr *_dpd_get_chan_attr_by_name(const struct iio_dpd_channel *pchan, const char *attr)
{
	uint32_t lp = 0;
	uint32_t attr_cnt = 0;
	struct iio_dpd_attr *dev_attr;

	attr_cnt = pchan->attr_cnt;

	for (lp = 0; lp < attr_cnt; lp ++)
	{
		dev_attr = pchan->pp_attr_array[lp];
		if (!strcmp(attr, dev_attr->file_name))
		{
			return dev_attr;
		}
		continue;
	}

	return NULL;
}

static struct iio_dpd_attr *_dpd_get_dev_attr_by_name(const char *attr)
{
	uint32_t lp = 0;
	uint32_t attr_cnt = 0;
	struct iio_dpd_dev_attr *dev_attr;
	struct iio_dpd_attr *p_attr = NULL;

	attr_cnt = dpd_device_data.dev_attr_cnt;

	if (!attr)
	{
		IIO_ERROR("input name is null\n");
		return NULL;
	}

	for (lp = 0; lp < attr_cnt; lp ++)
	{
		dev_attr = &iio_dpd_dev_array[lp];
		if (!dev_attr->pElement)
		{
			/* reach the end of device attribute array */
			return NULL;
		}
		if (dev_attr->attr_type == TYPE_IS_ATTR)
		{
			if (!strcmp(attr, ((struct iio_dpd_attr *)(dev_attr->pElement))->file_name))
			{
				return (struct iio_dpd_attr *)(dev_attr->pElement);
			}
			continue;
		}
		else if (dev_attr->attr_type == TYPE_IS_CHAN)
		{
			p_attr = _dpd_get_chan_attr_by_name((struct iio_dpd_channel *)(dev_attr->pElement), attr);
			if (p_attr)
				return p_attr;
		}
		else
		{
			IIO_WARNING("unknown device attribute[%d] type[%d]\n", lp, dev_attr->attr_type);
			continue;
		}
	}

	return NULL;

}

int _dpd_buffer_analyze(unsigned int nb, const char *src, size_t len)
{
	while (nb--) {
		int32_t val;

		if (len < 4)
			return -EINVAL;

		val = (int32_t) iio_be32toh(*(uint32_t *) src);
		src += 4;
		len -= 4;

		if (val > 0) {
			if ((uint32_t) val > len)
				return -EINVAL;

			/* Align the length to 4 bytes */
			if (val & 3)
				val = ((val >> 2) + 1) << 2;
			len -= val;
			src += val;
		}
	}

	/* We should have analyzed the whole buffer by now */
	return !len ? 0 : -EINVAL;
}

static const char * _dpd_get_filename(const struct iio_channel *chn,
		const char *attr)
{
	unsigned int i;
	for (i = 0; i < chn->nb_attrs; i++)
		if (!strcmp(attr, chn->attrs[i].name))
			return chn->attrs[i].filename;
	return attr;
}

int iio_dpd_device_pre_init(void)
{
	int ret = 0;

	ret = dpd_hw_open();
	if (ret)
	{
		IIO_ERROR("dpd device hardware open failed!\n");
		goto out;
	}
	ret = _dpd_dev_attribut_init();
	if (ret)
	{
		IIO_ERROR("dpd device attribute init failed!\n");
		goto out;
	}

	ret = dpd_Init(&dpdData);
	if (ret)
	{
		IIO_ERROR("dpd init failed!\n");
		goto out;
	}

out:
	return ret;
}

int iio_dpd_device_post_init(struct iio_device *dev)
{
	int ret = 0;
	if (dev)
		dev->userdata = (void *) &dpd_device_data;
	else
		ret = -EFAULT;

	return ret;
}

int iio_dpd_close(const struct iio_device *dev);

int iio_dpd_open(const struct iio_device *dev,
		size_t samples_count, bool cyclic)
{
	int ret = 0;
#if 0
	unsigned int i;
	char buf[1024];
	struct iio_dpd_device_data *pdata = dev->userdata;

	if (pdata->fd != -1)
		return -EBUSY;
	/* TBD */
	ret = iio_dpd_buffer_enabled_set(dev, false);
	if (ret < 0)
		return ret;

	iio_snprintf(buf, sizeof(buf), "%lu", (unsigned long) samples_count);
	ret = iio_dpd_write_dev_attr(dev, "buffer/length",
			buf, strlen(buf) + 1, false);
	if (ret < 0)
		return ret;

	/*
	 * Set watermark to the buffer size; the driver will adjust to its
	 * maximum if it's too high without issuing an error.
	 */
	ret = iio_dpd_write_dev_attr(dev, "buffer/watermark",
				   buf, strlen(buf) + 1, false);
	if (ret < 0 && ret != -ENOENT && ret != -EACCES)
		return ret;

	pdata->cancel_fd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
	if (pdata->cancel_fd == -1)
		return -errno;

	iio_snprintf(buf, sizeof(buf), "/dev/%s", dev->id);
	pdata->fd = open(buf, O_RDWR | O_CLOEXEC | O_NONBLOCK);
	if (pdata->fd == -1) {
		close(pdata->cancel_fd);
		return -errno;
	}

	/* Disable channels */
	for (i = 0; i < dev->nb_channels; i++) {
		struct iio_channel *chn = dev->channels[i];
		if (chn->index >= 0 && !iio_channel_is_enabled(chn)) {
			ret = channel_write_state(chn, false);
			if (ret < 0)
				goto err_close;
		}
	}
	/* Enable channels */
	for (i = 0; i < dev->nb_channels; i++) {
		struct iio_channel *chn = dev->channels[i];
		if (chn->index >= 0 && iio_channel_is_enabled(chn)) {
			ret = channel_write_state(chn, true);
			if (ret < 0)
				goto err_close;
		}
	}

	pdata->cyclic = cyclic;
	pdata->cyclic_buffer_enqueued = false;
	pdata->samples_count = samples_count;

	if (WITH_LOCAL_MMAP_API) {
		ret = enable_high_speed(dev);
		if (ret < 0 && ret != -ENOSYS)
			goto err_close;

		pdata->is_high_speed = !ret;
	}

	if (!pdata->is_high_speed) {
		unsigned long size = samples_count * pdata->max_nb_blocks;
		IIO_WARNING("High-speed mode not enabled\n");

		/* Cyclic mode is only supported in high-speed mode */
		if (cyclic) {
			ret = -EPERM;
			goto err_close;
		}

		/* Increase the size of the kernel buffer, when using the
		 * low-speed interface. This avoids losing samples when
		 * refilling the iio_buffer. */
		iio_snprintf(buf, sizeof(buf), "%lu", size);
		ret = iio_dpd_write_dev_attr(dev, "buffer/length",
				buf, strlen(buf) + 1, false);
		if (ret < 0)
			goto err_close;
	}

	ret = iio_dpd_buffer_enabled_set(dev, true);
	if (ret < 0)
		goto err_close;

	return 0;
err_close:
	iio_dpd_close(dev);
#endif
	return ret;
}

int iio_dpd_close(const struct iio_device *dev)
{
	int ret = 0;
#if 0
	struct iio_dpd_device_data *pdata = dev->userdata;
	unsigned int i;
	char err_str[32];
	int ret1;

	if (pdata->fd == -1)
		return -EBADF;

	/* TBD */
	ret = 0;
	ret1 = 0;
	if (pdata->is_high_speed) {
		if (pdata->addrs) {
			for (i = 0; i < pdata->allocated_nb_blocks; i++)
				munmap(pdata->addrs[i], pdata->blocks[i].size);
		}
		if (pdata->fd > -1)
			ret = ioctl_nointr(pdata->fd, BLOCK_FREE_IOCTL, 0);
		if (ret) {
			iio_strerror(-ret, err_str, sizeof(err_str));
			IIO_ERROR("Error during ioctl(): %s\n", err_str);
		}
		pdata->allocated_nb_blocks = 0;
		free(pdata->addrs);
		pdata->addrs = NULL;
		free(pdata->blocks);
		pdata->blocks = NULL;
	}

	ret1 = close(pdata->fd);
	if (ret1) {
		ret1 = -errno;
		iio_strerror(errno, err_str, sizeof(err_str));
		IIO_ERROR("Error during close() of main FD: %s\n", err_str);
		if (ret == 0)
			ret = ret1;
	}

	pdata->fd = -1;

	if (pdata->cancel_fd > -1) {
		ret1 = close(pdata->cancel_fd);
		pdata->cancel_fd = -1;

		if (ret1) {
			ret1 = -errno;
			iio_strerror(errno, err_str, sizeof(err_str));
			IIO_ERROR("Error during close() of cancel FD): %s\n",
				  err_str);
			if (ret == 0)
				ret = ret1;
		}
	}

	ret1 = iio_dpd_buffer_enabled_set(dev, false);
	if (ret1) {
		iio_strerror(-ret1, err_str, sizeof(err_str));
		IIO_ERROR("Error during buffer disable: %s\n", err_str);
		if (ret == 0)
			ret = ret1;
	}

	for (i = 0; i < dev->nb_channels; i++) {
		struct iio_channel *chn = dev->channels[i];

		if (!chn->pdata->enable_fn)
			continue;

		ret1 = channel_write_state(chn, false);
		if (ret1 == 0)
			continue;

		ret1 = -errno;
		iio_strerror(errno, err_str, sizeof(err_str));
		IIO_ERROR("Error during channel[%u] disable: %s\n",
			  i, err_str);
		if (ret == 0)
			ret = ret1;
	}
#endif
	return ret;
}

int iio_dpd_get_fd(const struct iio_device *dev)
{
	struct iio_dpd_device_data *pdata = (struct iio_dpd_device_data *)dev->userdata;
	if (pdata->fd == -1)
		return -EBADF;
	else
		return pdata->fd;
}

int iio_dpd_get_trigger(const struct iio_device *dev,
		const struct iio_device **trigger)
{
	#if 0
	char buf[1024];
	unsigned int i;
	ssize_t nb = iio_dpd_read_dev_attr(dev, "trigger/current_trigger",
			buf, sizeof(buf), false);
	if (nb < 0) {
		*trigger = NULL;
		return (int) nb;
	}

	if (buf[0] == '\0') {
		*trigger = NULL;
		return 0;
	}

	nb = iio_context_get_devices_count(dev->ctx);
	for (i = 0; i < (size_t) nb; i++) {
		const struct iio_device *cur = iio_context_get_device(dev->ctx, i);
		if (cur->name && !strcmp(cur->name, buf)) {
			*trigger = cur;
			return 0;
		}
	}
	return -ENXIO;
	#endif 
	return 0;
}

int iio_dpd_set_trigger(const struct iio_device *dev,
		const struct iio_device *trigger)
{
	#if 0
	ssize_t nb;

	const char *value = trigger ? trigger->name : "";
	nb = iio_dpd_write_dev_attr(dev, "trigger/current_trigger",
			value, strlen(value) + 1, false);
	if (nb < 0)
		return (int) nb;
	else
	#endif
		return 0;
}


ssize_t iio_dpd_read(const struct iio_device *dev,
		void *dst, size_t len, uint32_t *mask, size_t words)
{
	ssize_t ret = 0;
#if 0
	struct iio_dpd_device_data *pdata = dev->pdata;
	uintptr_t ptr = (uintptr_t) dst;
	struct timespec start;
	ssize_t readsize;
	if (pdata->fd == -1)
		return -EBADF;
	if (words != dev->words)
		return -EINVAL;

	memcpy(mask, dev->mask, words);

	if (len == 0)
		return 0;

	clock_gettime(CLOCK_MONOTONIC, &start);

	while (len > 0) {
		ret = device_check_ready(dev, POLLIN, &start);
		if (ret < 0)
			break;

		do {
			ret = read(pdata->fd, (void *) ptr, len);
		} while (ret == -1 && errno == EINTR);

		if (ret == -1) {
			if (pdata->blocking && errno == EAGAIN)
				continue;
			ret = -errno;
			break;
		} else if (ret == 0) {
			ret = -EIO;
			break;
		}

		ptr += ret;
		len -= ret;
	}

	readsize = (ssize_t)(ptr - (uintptr_t) dst);
	if ((ret > 0 || ret == -EAGAIN) && (readsize > 0))
		return readsize;
	else
	#endif
		return ret;
}

ssize_t iio_dpd_write(const struct iio_device *dev,
		const void *src, size_t len)
{
	ssize_t ret = 0;
#if 0
	struct iio_dpd_device_data *pdata = dev->pdata;
	uintptr_t ptr = (uintptr_t) src;
	struct timespec start;
	ssize_t writtensize;
	if (pdata->fd == -1)
		return -EBADF;

	if (len == 0)
		return 0;

	if (iio_device_is_dpd(dev))
	{
		/* dpd operation */
		return iio_dpd_write(dev, src, len);
	}

	clock_gettime(CLOCK_MONOTONIC, &start);

	while (len > 0) {
		ret = device_check_ready(dev, POLLOUT, &start);
		if (ret < 0)
			break;

		do {
			ret = write(pdata->fd, (void *) ptr, len);
		} while (ret == -1 && errno == EINTR);

		if (ret == -1) {
			if (pdata->blocking && errno == EAGAIN)
				continue;

			ret = -errno;
			break;
		} else if (ret == 0) {
			ret = -EIO;
			break;
		}

		ptr += ret;
		len -= ret;
	}

	writtensize = (ssize_t)(ptr - (uintptr_t) src);
	if ((ret > 0 || ret == -EAGAIN) && (writtensize > 0))
		return writtensize;
	else
	#endif
		return ret;
}


ssize_t iio_dpd_get_buffer(const struct iio_device *dev,
		void **addr_ptr, size_t bytes_used,
		uint32_t *mask, size_t words)
{
	ssize_t ret = 0;
	#if 0
	struct block block;
	struct iio_dpd_device_data *pdata = dev->pdata;
	struct timespec start;
	char err_str[1024];
	int f = pdata->fd;


	if (!WITH_LOCAL_MMAP_API || !pdata->is_high_speed)
		return -ENOSYS;
	if (f == -1)
		return -EBADF;
	if (!addr_ptr)
		return -EINVAL;

	if (pdata->last_dequeued >= 0) {
		struct block *last_block = &pdata->blocks[pdata->last_dequeued];

		if (pdata->cyclic) {
			if (pdata->cyclic_buffer_enqueued)
				return -EBUSY;
			pdata->blocks[0].flags |= BLOCK_FLAG_CYCLIC;
			pdata->cyclic_buffer_enqueued = true;
		}

		last_block->bytes_used = bytes_used;
		ret = (ssize_t) ioctl_nointr(f,
				BLOCK_ENQUEUE_IOCTL, last_block);
		if (ret) {
			iio_strerror(-ret, err_str, sizeof(err_str));
			IIO_ERROR("Unable to enqueue block: %s\n", err_str);
			return ret;
		}

		if (pdata->cyclic) {
			*addr_ptr = pdata->addrs[pdata->last_dequeued];
			return (ssize_t) last_block->bytes_used;
		}

		pdata->last_dequeued = -1;
	}

	clock_gettime(CLOCK_MONOTONIC, &start);

	do {
		ret = (ssize_t) device_check_ready(dev, POLLIN | POLLOUT, &start);
		if (ret < 0)
			return ret;

		memset(&block, 0, sizeof(block));
		ret = (ssize_t) ioctl_nointr(f, BLOCK_DEQUEUE_IOCTL, &block);
	} while (pdata->blocking && ret == -EAGAIN);

	if (ret) {
		if ((!pdata->blocking && ret != -EAGAIN) ||
				(pdata->blocking && ret != -ETIMEDOUT)) {
			iio_strerror(-ret, err_str, sizeof(err_str));
			IIO_ERROR("Unable to dequeue block: %s\n", err_str);
		}
		return ret;
	}

	pdata->last_dequeued = block.id;
	*addr_ptr = pdata->addrs[block.id];
	return (ssize_t) block.bytes_used;
	#endif
	return ret;
}

ssize_t iio_dpd_read_all_dev_attrs(const struct iio_device *dev,
		char *dst, size_t len, enum iio_attr_type type)
{
	unsigned int i, nb;
	char **attrs;
	char *ptr = dst;

	switch (type) {
		case IIO_ATTR_TYPE_DEVICE:
			nb =  dev->attrs.num;
			attrs = dev->attrs.names;
			break;
		case IIO_ATTR_TYPE_DEBUG:
			nb =  dev->debug_attrs.num;
			attrs = dev->debug_attrs.names;
			break;
		case IIO_ATTR_TYPE_BUFFER:
			nb =  dev->buffer_attrs.num;
			attrs = dev->buffer_attrs.names;
			break;
		default:
			return -EINVAL;
			break;
	}

	for (i = 0; len >= 4 && i < nb; i++) {
		/* Recursive! */
		ssize_t ret = iio_dpd_read_dev_attr(dev, attrs[i],
				ptr + 4, len - 4, type);
		*(uint32_t *) ptr = iio_htobe32(ret);

		/* Align the length to 4 bytes */
		if (ret > 0 && ret & 3)
			ret = ((ret >> 2) + 1) << 2;
		ptr += 4 + (ret < 0 ? 0 : ret);
		len -= 4 + (ret < 0 ? 0 : ret);
	}

	return ptr - dst;
}

ssize_t iio_dpd_read_dev_attr(const struct iio_device *dev,
		const char *attr, char *dst, size_t len, enum iio_attr_type type)
{
	ssize_t ret = 0;
	struct iio_dpd_attr *pattr = NULL;

	if (!attr)
		return iio_dpd_read_all_dev_attrs(dev, dst, len, type);


	switch (type) {
		case IIO_ATTR_TYPE_DEVICE:
			pattr = _dpd_get_dev_attr_by_name(attr);
			break;
		case IIO_ATTR_TYPE_DEBUG:	/* not support */
		case IIO_ATTR_TYPE_BUFFER:	/* not support */
		default:
			return -EINVAL;
	}

	if (pattr)
	{
		if (pattr->show)
			ret = pattr->show(dst);
		else
			ret = -ESRCH;
	}

	return ret ? ret : -EIO;
}

ssize_t iio_dpd_write_all_dev_attrs(const struct iio_device *dev,
		const char *src, size_t len, enum iio_attr_type type)
{
	unsigned int i, nb;
	char **attrs;
	const char *ptr = src;

	switch (type) {
		case IIO_ATTR_TYPE_DEVICE:
			nb =  dev->attrs.num;
			attrs = dev->attrs.names;
			break;
		case IIO_ATTR_TYPE_DEBUG:
			nb =  dev->debug_attrs.num;
			attrs = dev->debug_attrs.names;
			break;
		case IIO_ATTR_TYPE_BUFFER:
			nb =  dev->buffer_attrs.num;
			attrs = dev->buffer_attrs.names;
			break;
		default:
			return -EINVAL;
			break;
	}

	/* First step: Verify that the buffer is in the correct format */
	if (_dpd_buffer_analyze(nb, src, len))
		return -EINVAL;

	/* Second step: write the attributes */
	for (i = 0; i < nb; i++) {
		int32_t val = (int32_t) iio_be32toh(*(uint32_t *) ptr);
		ptr += 4;

		if (val > 0) {
			iio_dpd_write_dev_attr(dev, attrs[i], ptr, val, type);

			/* Align the length to 4 bytes */
			if (val & 3)
				val = ((val >> 2) + 1) << 2;
			ptr += val;
		}
	}

	return ptr - src;
}

ssize_t iio_dpd_write_dev_attr(const struct iio_device *dev,
		const char *attr, const char *src, size_t len, enum iio_attr_type type)
{
	ssize_t ret = 0;
	struct iio_dpd_attr *pattr = NULL;

	if (!attr)
		return iio_dpd_write_all_dev_attrs(dev, src, len, type);

	switch (type) {
		case IIO_ATTR_TYPE_DEVICE:
			pattr = _dpd_get_dev_attr_by_name(attr);
			break;
		case IIO_ATTR_TYPE_DEBUG:	/* not support */
		case IIO_ATTR_TYPE_BUFFER:	/* not support */
		default:
			return -EINVAL;
	}

	if (pattr)
	{
		if (pattr->store)
			ret = pattr->store(src);
		else
			ret = -ESRCH;
	}

	return ret ? ret : -EIO;
}

ssize_t iio_dpd_read_all_chn_attrs(const struct iio_channel *chn,
		char *dst, size_t len)
{
	unsigned int i;
	char *ptr = dst;

	for (i = 0; len >= 4 && i < chn->nb_attrs; i++) {
		/* Recursive! */
		ssize_t ret = iio_dpd_read_chn_attr(chn,
				chn->attrs[i].name, ptr + 4, len - 4);
		*(uint32_t *) ptr = iio_htobe32(ret);

		/* Align the length to 4 bytes */
		if (ret > 0 && ret & 3)
			ret = ((ret >> 2) + 1) << 2;
		ptr += 4 + (ret < 0 ? 0 : ret);
		len -= 4 + (ret < 0 ? 0 : ret);
	}

	return ptr - dst;
}

ssize_t iio_dpd_read_chn_attr(const struct iio_channel *chn,
		const char *attr, char *dst, size_t len)
{
	if (!attr)
		return iio_dpd_read_all_chn_attrs(chn, dst, len);

	attr = _dpd_get_filename(chn, attr);
	return iio_dpd_read_dev_attr(chn->dev, attr, dst, len, false);
}


ssize_t iio_dpd_write_all_chn_attrs(const struct iio_channel *chn,
		const char *src, size_t len)
{
	unsigned int i, nb = chn->nb_attrs;
	const char *ptr = src;

	/* First step: Verify that the buffer is in the correct format */
	if (_dpd_buffer_analyze(nb, src, len))
		return -EINVAL;

	/* Second step: write the attributes */
	for (i = 0; i < nb; i++) {
		int32_t val = (int32_t) iio_be32toh(*(uint32_t *) ptr);
		ptr += 4;

		if (val > 0) {
			iio_dpd_write_chn_attr(chn, chn->attrs[i].name, ptr, val);

			/* Align the length to 4 bytes */
			if (val & 3)
				val = ((val >> 2) + 1) << 2;
			ptr += val;
		}
	}

	return ptr - src;
}

ssize_t iio_dpd_write_chn_attr(const struct iio_channel *chn,
		const char *attr, const char *src, size_t len)
{
	if (!attr)
		return iio_dpd_write_all_chn_attrs(chn, src, len);

	attr = _dpd_get_filename(chn, attr);
	return iio_dpd_write_dev_attr(chn->dev, attr, src, len, false);
}
