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
#define IIO_DPD_ORX_BUFFER_DEV 	"axi-adrv9009-rx-obs-hpc"

/* generate the global dpd data */
DECLARE_IIO_DPD_DATA;


static ssize_t _dpd_Tu_i_en_show(char *dst);
static ssize_t _dpd_Tu_i_en_store(const char *src);
static ssize_t _dpd_Tu_i_index_show(char *dst);
static ssize_t _dpd_Tu_i_index_store(const char *src);
static ssize_t _dpd_Tu_i_type_show(char *dst);
static ssize_t _dpd_Tu_i_type_store(const char *src);

static ssize_t _dpd_Tu_q_en_show(char *dst);
static ssize_t _dpd_Tu_q_en_store(const char *src);
static ssize_t _dpd_Tu_q_index_show(char *dst);
static ssize_t _dpd_Tu_q_index_store(const char *src);
static ssize_t _dpd_Tu_q_type_show(char *dst);
static ssize_t _dpd_Tu_q_type_store(const char *src);

static ssize_t _dpd_Tx_i_en_show(char *dst);
static ssize_t _dpd_Tx_i_en_store(const char *src);
static ssize_t _dpd_Tx_i_index_show(char *dst);
static ssize_t _dpd_Tx_i_index_store(const char *src);
static ssize_t _dpd_Tx_i_type_show(char *dst);
static ssize_t _dpd_Tx_i_type_store(const char *src);

static ssize_t _dpd_Tx_q_en_show(char *dst);
static ssize_t _dpd_Tx_q_en_store(const char *src);
static ssize_t _dpd_Tx_q_index_show(char *dst);
static ssize_t _dpd_Tx_q_index_store(const char *src);
static ssize_t _dpd_Tx_q_type_show(char *dst);
static ssize_t _dpd_Tx_q_type_store(const char *src);

static ssize_t _dpd_TrackCfg_indirectRegValue_show(char *dst);
static ssize_t _dpd_TrackCfg_indirectRegValue_store(const char *src);
static ssize_t _dpd_TrackCfg_directRegValue_show(char *dst);
static ssize_t _dpd_TrackCfg_directRegValue_store(const char *src);
static ssize_t _dpd_TrackCfg_absOffset_show(char *dst);
static ssize_t _dpd_TrackCfg_absOffset_store(const char *src);

static ssize_t _dpd_dev_attr_name_show(char *dst);
static ssize_t _dpd_dev_attr_name_store(const char *src);
static ssize_t _dpd_dev_attr_version_show(char *dst);
static ssize_t _dpd_dev_attr_version_store(const char *src);
static ssize_t _dpd_dev_dbg_attr_reg_show(char *dst);
static ssize_t _dpd_dev_dbg_attr_reg_store(const char *src);
static ssize_t _dpd_dev_attr_en_show(char *dst);
static ssize_t _dpd_dev_attr_en_store(const char *src);

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
IIO_DPD_ADD_UNIQUE_CHAN_ATTR(TrackCfg, indirectRegValue, _dpd_TrackCfg_indirectRegValue_show, _dpd_TrackCfg_indirectRegValue_store, 20);
IIO_DPD_ADD_UNIQUE_CHAN_ATTR(TrackCfg, directRegValue, _dpd_TrackCfg_directRegValue_show, _dpd_TrackCfg_directRegValue_store, 21);
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, mu, 22);
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, magGain, 23);
IIO_DPD_ADD_CHAN_ATTR(TrackCfg, absScale, 24);
IIO_DPD_ADD_UNIQUE_CHAN_ATTR(TrackCfg, absOffset, _dpd_TrackCfg_absOffset_show, _dpd_TrackCfg_absOffset_store, 25);
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

/* generate dpd scan channel TU I Path signal */
IIO_DPD_ADD_UNIQUE_CHAN_ATTR(Tu_i, en, _dpd_Tu_i_en_show, _dpd_Tu_i_en_store, 0);
IIO_DPD_ADD_UNIQUE_CHAN_ATTR(Tu_i, index, _dpd_Tu_i_index_show, _dpd_Tu_i_index_store, 1);
IIO_DPD_ADD_UNIQUE_CHAN_ATTR(Tu_i, type, _dpd_Tu_i_type_show, _dpd_Tu_i_type_store, 2);

ADD_CHAN_ATTR_ARRAY_ELEMENT_START(Tu_i)
ADD_CHAN_ATTR_ARRAY_ELEMENT(Tu_i, en, 0),
ADD_CHAN_ATTR_ARRAY_ELEMENT(Tu_i, index, 1),
ADD_CHAN_ATTR_ARRAY_ELEMENT(Tu_i, type, 2),
ADD_CHAN_ATTR_ARRAY_ELEMENT_END(Tu_i);

/* generate dpd scan channel TU Q Path signal */
IIO_DPD_ADD_UNIQUE_CHAN_ATTR(Tu_q, en, _dpd_Tu_q_en_show, _dpd_Tu_q_en_store, 0);
IIO_DPD_ADD_UNIQUE_CHAN_ATTR(Tu_q, index, _dpd_Tu_q_index_show, _dpd_Tu_q_index_store, 1);
IIO_DPD_ADD_UNIQUE_CHAN_ATTR(Tu_q, type, _dpd_Tu_q_type_show, _dpd_Tu_q_type_store, 2);

ADD_CHAN_ATTR_ARRAY_ELEMENT_START(Tu_q)
ADD_CHAN_ATTR_ARRAY_ELEMENT(Tu_q, en, 0),
ADD_CHAN_ATTR_ARRAY_ELEMENT(Tu_q, index, 1),
ADD_CHAN_ATTR_ARRAY_ELEMENT(Tu_q, type, 2),
ADD_CHAN_ATTR_ARRAY_ELEMENT_END(Tu_q);


/* generate dpd scan channel TX I Path signal */
IIO_DPD_ADD_UNIQUE_CHAN_ATTR(Tx_i, en, _dpd_Tx_i_en_show, _dpd_Tx_i_en_store, 0);
IIO_DPD_ADD_UNIQUE_CHAN_ATTR(Tx_i, index, _dpd_Tx_i_index_show, _dpd_Tx_i_index_store, 1);
IIO_DPD_ADD_UNIQUE_CHAN_ATTR(Tx_i, type, _dpd_Tx_i_type_show, _dpd_Tx_i_type_store, 2);

ADD_CHAN_ATTR_ARRAY_ELEMENT_START(Tx_i)
ADD_CHAN_ATTR_ARRAY_ELEMENT(Tx_i, en, 0),
ADD_CHAN_ATTR_ARRAY_ELEMENT(Tx_i, index, 1),
ADD_CHAN_ATTR_ARRAY_ELEMENT(Tx_i, type, 2),
ADD_CHAN_ATTR_ARRAY_ELEMENT_END(Tx_i);

/* generate dpd scan channel TX Q Path signal */
IIO_DPD_ADD_UNIQUE_CHAN_ATTR(Tx_q, en, _dpd_Tx_q_en_show, _dpd_Tx_q_en_store, 0);
IIO_DPD_ADD_UNIQUE_CHAN_ATTR(Tx_q, index, _dpd_Tx_q_index_show, _dpd_Tx_q_index_store, 1);
IIO_DPD_ADD_UNIQUE_CHAN_ATTR(Tx_q, type, _dpd_Tx_q_type_show, _dpd_Tx_q_type_store, 2);

ADD_CHAN_ATTR_ARRAY_ELEMENT_START(Tx_q)
ADD_CHAN_ATTR_ARRAY_ELEMENT(Tx_q, en, 0),
ADD_CHAN_ATTR_ARRAY_ELEMENT(Tx_q, index, 1),
ADD_CHAN_ATTR_ARRAY_ELEMENT(Tx_q, type, 2),
ADD_CHAN_ATTR_ARRAY_ELEMENT_END(Tx_q);


/* Add the device channels */
IIO_DPD_ADD_DEV_SCAN_CHAN(Tu_i,0, "le:s16/16>>0");
IIO_DPD_ADD_DEV_SCAN_CHAN(Tu_q,1, "le:s16/16>>0");
IIO_DPD_ADD_DEV_SCAN_CHAN(Tx_i,2, "le:s16/16>>0");
IIO_DPD_ADD_DEV_SCAN_CHAN(Tx_q,3, "le:s16/16>>0");
IIO_DPD_ADD_DEV_CHAN(TrackCfg,4);
IIO_DPD_ADD_DEV_CHAN(DpdModelDesc,5);
IIO_DPD_ADD_DEV_CHAN(ActModelCfg,6);


IIO_DPD_ADD_DEV_DEFAULT_ATTR(actLutSatFlag,7);
IIO_DPD_ADD_DEV_DEFAULT_ATTR(capCfg.capDepth,8);
IIO_DPD_ADD_DEV_DEFAULT_ATTR(capCfg.capBatch,9);
IIO_DPD_ADD_DEV_DEFAULT_ATTR(pathDlyStatus.intDelay,10);
IIO_DPD_ADD_DEV_DEFAULT_ATTR(pathDlyStatus.fracDelay,11);
IIO_DPD_ADD_DEV_DEFAULT_ATTR(pathDlyStatus.iterNum,12);
IIO_DPD_ADD_DEV_DEFAULT_ATTR(pathDelayMode,13);
IIO_DPD_ADD_DEV_DEFAULT_ATTR(alignedSampleCount,14);
IIO_DPD_ADD_DEV_DEFAULT_ATTR(featureSampleCount,15);
IIO_DPD_ADD_DEV_DEFAULT_ATTR(procState,16);
IIO_DPD_ADD_DEV_DEFAULT_ATTR(iterCount,17);
IIO_DPD_ADD_DEV_DEFAULT_ATTR(direct,18);
IIO_DPD_ADD_DEV_UNIQUE_ATTR(name, _dpd_dev_attr_name_show, _dpd_dev_attr_name_store, 19);
IIO_DPD_ADD_DEV_UNIQUE_ATTR(version, _dpd_dev_attr_version_show, _dpd_dev_attr_version_store, 20);
/* the attribute reg is a debug reg*/
IIO_DPD_ADD_DEV_DEBUG_ATTR(direct_reg_access, _dpd_dev_dbg_attr_reg_show, _dpd_dev_dbg_attr_reg_store, 21);
IIO_DPD_ADD_DEV_UNIQUE_ATTR(enable, _dpd_dev_attr_en_show, _dpd_dev_attr_en_store, 22);

ADD_DEV_ATTR_ARRAY_ELEMENT_START()
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_CHAN, Tu_i,0),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_CHAN, Tu_q,1),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_CHAN, Tx_i,2),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_CHAN, Tx_q,3),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_CHAN, TrackCfg, 4),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_CHAN, DpdModelDesc, 5),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_CHAN, ActModelCfg, 6),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, actLutSatFlag, 7),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, capCfg.capDepth, 8),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, capCfg.capBatch, 9),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, pathDlyStatus.intDelay,10),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, pathDlyStatus.fracDelay,11),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, pathDlyStatus.iterNum,12),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, pathDelayMode, 13),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, alignedSampleCount, 14),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, featureSampleCount, 15),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, procState, 16),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, iterCount, 17),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, direct, 18),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, name, 19),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, version, 20),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR, direct_reg_access, 21),
ADD_DEV_ATTR_ARRAY_ELEMENT(TYPE_IS_ATTR,enable, 22),
ADD_DEV_ATTR_ARRAY_ELEMENT_END();


struct iio_dpd_device_data {
	int fd;		/* don't move the position of this element!! */
	struct iio_dpd_dev_attr *pdev_attr;
	uint32_t dev_attr_cnt;
};

static struct iio_dpd_device_data dpd_device_data;

static ssize_t __scan_attr_read(char const *chn, const char *attr, uint32_t index, char *dst) 
{ 
	char buf[IIO_DPD_BUF_SIZE] = {0,};
	char file_name[IIO_DPD_ATTR_NAME_LEN] = {0,};
	char file_path[IIO_DPD_ATTR_NAME_LEN] = {0,};
	FILE *f;
	ssize_t ret = 0;
	
	if (attr)
		iio_snprintf(file_name, sizeof(file_name), "in_voltage%d_%s_%s", index, chn, attr);
	else
		return -EFAULT;

	iio_snprintf(file_path, sizeof(file_path), "%s/%s/%s/%s", DPD_TMPFS_PATH, DPD_DEVICE_PATH, DPD_DEVICE_SCAN_PATH, file_name);
	
	f = fopen(file_path, "re");
	if (!f)
		return -EIO;

	ret = fread(buf, 1, sizeof(buf)-1, f);
	if (ret > 0) 
	{
		memcpy(dst, buf, ret);
		dst[ret] = '\0';
	}
	else
		dst[0] = '\0';

	fclose(f);

	return ret ? ret : -EIO;
}

static ssize_t __scan_attr_write(const char *chn, const char *attr, uint32_t index, const char *src) 
{ 
	char file_name[IIO_DPD_ATTR_NAME_LEN] = {0,};
	char file_path[IIO_DPD_ATTR_NAME_LEN] = {0,};
	FILE *f;
	ssize_t ret = 0;

	if (attr)
		iio_snprintf(file_name, sizeof(file_name), "in_voltage%d_%s_%s", index, chn, attr);
	else
		return -EFAULT;

	iio_snprintf(file_path, sizeof(file_path), "%s/%s/%s/%s", DPD_TMPFS_PATH, DPD_DEVICE_PATH, DPD_DEVICE_SCAN_PATH, file_name);
	
	f = fopen(file_path, "we");
	if (!f)
		return -EIO;

	ret = fwrite(src, 1, strlen(src), f);

	fclose(f);

	return ret ? ret : -EIO;
}

/* TU Channel attribute */
static ssize_t _dpd_Tu_i_en_show(char *dst)
{
	return __scan_attr_read("Tu_i", "en", 0, dst);
}

static ssize_t _dpd_Tu_i_en_store(const char *src)
{
	return __scan_attr_write("Tu_i", "en", 0, src);
}

static ssize_t _dpd_Tu_i_index_show(char *dst)
{
	return __scan_attr_read("Tu_i", "index", 0, dst);
}

static ssize_t _dpd_Tu_i_index_store(const char *src)
{
	return __scan_attr_write("Tu_i", "index", 0, src);
}

static ssize_t _dpd_Tu_i_type_show(char *dst)
{
	return __scan_attr_read("Tu_i", "type", 0, dst);
}

static ssize_t _dpd_Tu_i_type_store(const char *src)
{
	return __scan_attr_write("Tu_i", "type", 0, src);
}

static ssize_t _dpd_Tu_q_en_show(char *dst)
{
	return __scan_attr_read("Tu_q", "en", 1, dst);
}

static ssize_t _dpd_Tu_q_en_store(const char *src)
{
	return __scan_attr_write("Tu_q", "en", 1, src);
}

static ssize_t _dpd_Tu_q_index_show(char *dst)
{
	return __scan_attr_read("Tu_q", "index", 1, dst);
}

static ssize_t _dpd_Tu_q_index_store(const char *src)
{
	return __scan_attr_write("Tu_q", "index", 1, src);
}

static ssize_t _dpd_Tu_q_type_show(char *dst)
{
	return __scan_attr_read("Tu_q", "type", 1, dst);
}

static ssize_t _dpd_Tu_q_type_store(const char *src)
{
	return __scan_attr_write("Tu_q", "type", 1, src);
}


/* TX Channel attribute */
static ssize_t _dpd_Tx_i_en_show(char *dst)
{
	return __scan_attr_read("Tx_i", "en", 2, dst);
}

static ssize_t _dpd_Tx_i_en_store(const char *src)
{
	return __scan_attr_write("Tx_i", "en", 2, src);
}

static ssize_t _dpd_Tx_i_index_show(char *dst)
{
	return __scan_attr_read("Tx_i", "index", 2, dst);
}

static ssize_t _dpd_Tx_i_index_store(const char *src)
{
	return __scan_attr_write("Tx_i", "index", 2, src);
}

static ssize_t _dpd_Tx_i_type_show(char *dst)
{
	return __scan_attr_read("Tx_i", "type", 2, dst);
}

static ssize_t _dpd_Tx_i_type_store(const char *src)
{
	return __scan_attr_write("Tx_i", "type", 2, src);
}


static ssize_t _dpd_Tx_q_en_show(char *dst)
{
	return __scan_attr_read("Tx_q", "en", 3, dst);
}

static ssize_t _dpd_Tx_q_en_store(const char *src)
{
	return __scan_attr_write("Tx_q", "en", 3, src);
}

static ssize_t _dpd_Tx_q_index_show(char *dst)
{
	return __scan_attr_read("Tx_q", "index", 3, dst);
}

static ssize_t _dpd_Tx_q_index_store(const char *src)
{
	return __scan_attr_write("Tx_q", "index", 3, src);
}

static ssize_t _dpd_Tx_q_type_show(char *dst)
{
	return __scan_attr_read("Tx_q", "type", 3, dst);
}

static ssize_t _dpd_Tx_q_type_store(const char *src)
{
	return __scan_attr_write("Tx_q", "type", 3, src);
}

static ssize_t _dpd_TrackCfg_indirectRegValue_show(char *dst) 
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

static ssize_t _dpd_TrackCfg_indirectRegValue_store(const char *src) 
{ 
	uint32_t val = 0; 
	char *end; ssize_t ret=0; 
	val = strtoul(src, &end, 16); 
	ret = strlen(src); 
	dpdData.pTrackCfg->indirectRegValue = (double)val/1000.0; 
	return ret ? ret : -5; 
}

static ssize_t _dpd_TrackCfg_directRegValue_show(char *dst) 
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

static ssize_t _dpd_TrackCfg_directRegValue_store(const char *src) 
{ 
	uint32_t val = 0; 
	char *end; ssize_t ret=0; 
	val = strtoul(src, &end, 16); 
	ret = strlen(src); 
	dpdData.pTrackCfg->directRegValue = (double)val/1000.0; 
	return ret ? ret : -5; 
}

static ssize_t _dpd_TrackCfg_absOffset_show(char *dst) 
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

static ssize_t _dpd_TrackCfg_absOffset_store(const char *src) 
{ 
	uint32_t val = 0; 
	char *end; ssize_t ret=0; 
	val = strtoul(src, &end, 16); 
	ret = strlen(src); 
	dpdData.pTrackCfg->absOffset = (double)val/1000.0; 
	return ret ? ret : -5; 
}

static ssize_t _dpd_dev_attr_en_show(char *dst)
{
	uint32_t val;
	uint32_t rd_val;
	uint32_t tmp_ret;
	char buf[IIO_DPD_BUF_SIZE] = {0,};
	char *end;
	char file_path[IIO_DPD_ATTR_NAME_LEN] = {0,};
	FILE *f;
	ssize_t ret = 0;

	iio_snprintf(file_path, sizeof(file_path), "%s/%s/%s", DPD_TMPFS_PATH, DPD_DEVICE_PATH, "enable");
	
	f = fopen(file_path, "re");
	if (!f)
		return -EIO;

	ret = fread(buf, 1, sizeof(buf)-1, f);
	val = strtoul(buf, &end, 0);
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

static ssize_t _dpd_dev_attr_en_store(const char *src)
{
	uint32_t val[2] = {0,};
	ssize_t ret = 0;
	int argc;
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
		str_tmp = iio_strdup(rest);
		if (var[argc])
			val[argc] = strtoul(var[argc], &end, 0);
		else 
		{
			break;
		}
	}

	iio_snprintf(file_path, sizeof(file_path), "%s/%s/%s", DPD_TMPFS_PATH, DPD_DEVICE_PATH, "enable");
	
	f = fopen(file_path, "we");
	if (!f)
		return -EIO;

	fwrite(var[0], 1, strlen(var[0]), f);
	fclose(f);

	if (argc == 1) 
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

static ssize_t _dpd_dev_dbg_attr_reg_show(char *dst)
{
	uint32_t val;
	uint32_t rd_val;
	uint32_t tmp_ret;
	char buf[IIO_DPD_BUF_SIZE] = {0,};
	char *end;
	char file_path[IIO_DPD_ATTR_NAME_LEN] = {0,};
	FILE *f;
	ssize_t ret = 0;

	iio_snprintf(file_path, sizeof(file_path), "%s/%s/%s/%s/%s", DPD_TMPFS_PATH, DPD_DEVICE_PATH, \
										DPD_DEVICE_DEBUG_PATH, DPD_DEVICE_PATH, "direct_reg_access");
	
	f = fopen(file_path, "re");
	if (!f)
		return -EIO;

	ret = fread(buf, 1, sizeof(buf)-1, f);
	val = strtoul(buf, &end, 0);
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

static ssize_t _dpd_dev_dbg_attr_reg_store(const char *src)
{
	uint32_t val[2] = {0,};
	ssize_t ret = 0;
	int argc;
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
		str_tmp = iio_strdup(rest);
		if (var[argc])
			val[argc] = strtoul(var[argc], &end, 0);
		else 
		{
			break;
		}
	}

	iio_snprintf(file_path, sizeof(file_path), "%s/%s/%s/%s/%s", DPD_TMPFS_PATH, DPD_DEVICE_PATH, \
						DPD_DEVICE_DEBUG_PATH, DPD_DEVICE_PATH, "direct_reg_access");
	
	f = fopen(file_path, "we");
	if (!f)
		return -EIO;

	fwrite(var[0], 1, strlen(var[0]), f);
	fclose(f);

	if (argc == 1) 
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

static ssize_t _dpd_dev_attr_version_show(char *dst)
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

	fwrite(dst, 1, strlen(dst), f);
	fclose(f);

	if (ret > 0) 
	{
		dst[ret] = '\0';
	}
	else
		dst[0] = '\0';
	return ret ? ret : -EIO;
}

static ssize_t _dpd_dev_attr_version_store(const char *src)
{
	return -EIO;
}

static ssize_t _dpd_dev_attr_name_show(char *dst)
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

static ssize_t _dpd_dev_attr_name_store(const char *src)
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

	/* create the dpd device folder */
	memset(cmd, 0x00, sizeof(cmd));
	iio_snprintf(cmd, 100, "mkdir -p %s/%s", DPD_TMPFS_PATH, DPD_DEVICE_PATH);
	ret = system(cmd);

	/* create the dpd device scan channel folder */
	memset(cmd, 0x00, sizeof(cmd));
	iio_snprintf(cmd, 100, "mkdir -p %s/%s/%s", DPD_TMPFS_PATH, DPD_DEVICE_PATH, DPD_DEVICE_SCAN_PATH);
	ret = system(cmd);

	/* create the dpd device debug attribute folder */
	memset(cmd, 0x00, sizeof(cmd));
	iio_snprintf(cmd, 100, "mkdir -p %s/%s/%s/%s", DPD_TMPFS_PATH, DPD_DEVICE_PATH, DPD_DEVICE_DEBUG_PATH, DPD_DEVICE_PATH);
	ret = system(cmd);

	return ret;
}

static int _dpd_dev_destroy_fs(void)
{
	int ret;
	char cmd[100] = {0,};

	iio_snprintf(cmd, 100, "rm -rf %s/%s", DPD_TMPFS_PATH, DPD_DEVICE_PATH);
	ret = system(cmd);

	memset(cmd, 0x00, sizeof(cmd));
	/* mount the tmpfs to the mount point /tmp folder */
	iio_snprintf(cmd, 100, "umount %s", DPD_TMPFS_PATH);
	ret = system(cmd);

	return ret;
}

static int _dpd_dev_chan_create_file(struct iio_dpd_channel *p_chan)
{
	char file_name[IIO_DPD_ATTR_NAME_LEN] = {0,};
	struct iio_dpd_attr *p_attr = NULL;
	uint32_t is_scan;
	uint32_t lp;
	int ret = 0;

	char cmd[100] = {0,};

	if (!p_chan)
	{
		IIO_ERROR("NULL input parameter pointer!\n");
		return -EFAULT; 
	}

	is_scan = p_chan->is_scan;

	if (!p_chan->pp_attr_array)
	{
		IIO_WARNING("NULL attr in channel[%s]!\n", p_chan->name);
		return -EFAULT; 
	}

	for (lp = 0; (p_attr = p_chan->pp_attr_array[lp]) != NULL; lp ++)
	{
		memset(file_name, 0x00, sizeof(file_name));
		memset(cmd, 0x00, sizeof(cmd));
		if (is_scan)
		{
			iio_snprintf(file_name, sizeof(file_name), "in_voltage%d_%s_%s", p_chan->id, p_chan->name, p_attr->name);
			iio_snprintf(p_attr->file_name, sizeof(file_name), "%s/%s", DPD_DEVICE_SCAN_PATH, file_name);
			iio_snprintf(cmd, sizeof(cmd), "touch %s/%s/%s/%s", DPD_TMPFS_PATH, DPD_DEVICE_PATH, DPD_DEVICE_SCAN_PATH ,file_name);
		}
		else 
		{
			iio_snprintf(file_name, sizeof(file_name), "in_%s_%s", p_chan->name, p_attr->name);
			iio_snprintf(p_attr->file_name, sizeof(file_name), "%s", file_name);
			iio_snprintf(cmd, sizeof(cmd), "touch %s/%s/%s", DPD_TMPFS_PATH, DPD_DEVICE_PATH, file_name);
		}
		ret = system(cmd);
	}

	p_chan->attr_cnt = lp;

	return ret;
}

static int _dpd_scan_attr_init(void)
{
	struct iio_dpd_channel *pchan = NULL;
	char tmp[16] = {0,};
	uint32_t lp = 0;
	int ret = 0;
	uint8_t type;

	for(lp = 0; lp < IIO_DPD_MAX_CHAN_ATTR_CNT; lp ++)
	{
		type =  iio_dpd_dev_array[lp].attr_type;
		if (!iio_dpd_dev_array[lp].pElement)
			break;
		if (type == TYPE_IS_CHAN) 
		{
			pchan = (struct iio_dpd_channel *)(iio_dpd_dev_array[lp].pElement);
			if (pchan->is_scan)
			{
				ret = __scan_attr_write(pchan->name, "type", pchan->id, pchan->scan_type);
				if (ret <= 0)
				{
					ret = -EIO;
					goto err;
				}
				iio_snprintf(tmp, sizeof(tmp), "%d", pchan->id);
				ret = __scan_attr_write(pchan->name, "index", pchan->id, tmp);
				if (ret <= 0)
				{
					ret = -EIO;
					goto err;
				}
			}
		}
	}

err:
	return ret;

}

static int _dpd_dev_create_attr_file(struct iio_dpd_attr *p_attr)
{
	char cmd[100] = {0,};
	uint32_t is_debug;
	int ret;

	if (!p_attr)
	{
		IIO_ERROR("NULL input parameter pointer!\n");
		return -EFAULT; 
	}

	is_debug = p_attr->debug;

	if (is_debug)
	{
		iio_snprintf(cmd, sizeof(cmd), "touch %s/%s/%s/%s/%s", DPD_TMPFS_PATH, DPD_DEVICE_PATH, \
						DPD_DEVICE_DEBUG_PATH,DPD_DEVICE_PATH,p_attr->name);
		iio_snprintf(p_attr->file_name, sizeof(cmd), "%s", p_attr->name);
	}
	else
	{
		iio_snprintf(cmd, sizeof(cmd), "touch %s/%s/%s", DPD_TMPFS_PATH, DPD_DEVICE_PATH, p_attr->name);
		iio_snprintf(p_attr->file_name, sizeof(cmd), "%s", p_attr->name);
	}
	ret = system(cmd);

	return ret;
}

static int _dpd_dev_attribut_init(void)
{
	int ret = 0;
	int tmp_ret = 0;
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

	_dpd_dev_attr_name_store("dpd");
	_dpd_scan_attr_init();
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

struct iio_device *_dpd_get_obs_dev(struct iio_device *dpd)
{
	if (!dpd)
	{
		IIO_ERROR("NULL input pointer!\n");
		return NULL;
	}

	return iio_context_find_device(dpd->ctx, IIO_DPD_ORX_BUFFER_DEV);
}

int _dpd_tracking_entry(struct iio_device *dev, uint32_t iter_cnt)
{
	int dpdErr = 0;
	#if 0
	ssize_t sample_size;
	struct iio_buffer *buffer;
	struct iio_device *obs;

	/* 1.bypass actuator */
	dpd_register_write(ADDR_ACT_OUT_SEL, DPD_BYPASS);

	/* 2.dpd init */
	/* Has been done in device initialization process */
	// dpdErr = dpd_Init(&dpdData);

	for(uint8_t iters = 0u; iters < iter_cnt; iters++)
	{
#if 0
		if(dpdData.direct)
		{
			/* 1.enable actuator */
			dpd_register_write(ADDR_ACT_OUT_SEL, DPD_ENABLE);
		}
		else
		{
			/* 1.bypass actuator */
			dpd_register_write(ADDR_ACT_OUT_SEL, DPD_BYPASS);
		}
#endif
		/* 3.capture */
		if(DPD_ERR_CODE_NO_ERROR == dpdErr)
		{
			obs = iio_context_find_device(dev->ctx, IIO_DPD_ORX_BUFFER_DEV);
			sample_size = iio_device_get_sample_size(obs);
			/* Zero isn't normally an error code, but in this case it is an error */
			if (sample_size == 0) {
				IIO_ERROR("Unable to get sample size, returned 0\n");
				return -EFAULT;
			} else if (sample_size < 0) {
				char buf[256];
				iio_strerror(errno, buf, sizeof(buf));
				IIO_ERROR("Unable to get sample size : %s\n", buf);
				return -EFAULT;
			}

			buffer = iio_device_create_buffer(dev, buffer_size, false);
			if (!buffer) {
				char buf[256];
				iio_strerror(errno, buf, sizeof(buf));
				IIO_ERROR("Unable to allocate buffer: %s\n", buf);
				return EXIT_FAILURE;
			}
			transfer_rx.size = DPD_CAP_SIZE * TALISE_NUM_CHANNELS / 2 *
								NO_OS_DIV_ROUND_UP(talInit.jesd204Settings.framerA.Np, 8);

			/* Read the data from the ADC DMA. */
			axi_dmac_transfer_start(rx_os_dmac, &transfer_rx);

			/* Wait until transfer finishes */
			int32_t status = axi_dmac_transfer_wait_completion(rx_os_dmac, 500);

			/* Flush cache data. */
			Xil_DCacheInvalidateRange((uintptr_t)ADC_DDR_BASEADDR,
					DPD_CAP_SIZE * TALISE_NUM_CHANNELS / 2 *
						NO_OS_DIV_ROUND_UP(talInit.jesd204Settings.framerA.Np, 8));

			if(status < 0)
			{
				dpdErr = DPD_CAPTURE_ORX_ERROR;
			}
			else
			{
				memcpy(wr_data, (uint8_t*)ADC_DDR_BASEADDR, DPD_CAP_SIZE*4); // ORx
				dpdErr = dpd_read_capture_buffer(0, capTuBuf, DPD_CAP_SIZE);
				dpdErr = dpd_read_capture_buffer(1, capTxBuf, DPD_CAP_SIZE);
			}
		}

		/* 4.coeffs estimate */
		if(DPD_ERR_CODE_NO_ERROR == dpdErr)
		{
			uint16_t data_i, data_q;
			int16_t tmp_i, tmp_q;

			if((dpdData.pTrackCfg->direct == 1) && (dpdData.iterCount > DPD_MAX_INDIRECT_COUNT - 1))
			{
				dpdData.direct = 1;
			}
			else
			{
				dpdData.direct = 0;
			}

			/* convert int32_t to double complex for Tx */
			for(uint16_t index = 0; index < DPD_CAP_SIZE; index=index+2)
			{
				if(dpdData.direct)
				{
					data_i = (capTuBuf[index] >> 0) & 0xffff;
					data_q = (capTuBuf[index+1]>> 0) & 0xffff;

					tmp_i = (int16_t)(data_i);
					tmp_q = (int16_t)(data_q);

					pTx[index] = tmp_i*1.0/32768 + I*(tmp_q*1.0/32768);

					data_i = (capTuBuf[index] >> 16) & 0xffff;
					data_q = (capTuBuf[index+1]>> 16) & 0xffff;
					tmp_i = (int16_t)(data_i);
					tmp_q = (int16_t)(data_q);

					pTx[index+1] = tmp_i*1.0/32768 + I*(tmp_q*1.0/32768);
				}
				else
				{
					data_i = (capTxBuf[index] >> 0) & 0xffff;
					data_q = (capTxBuf[index+1]>> 0) & 0xffff;

					tmp_i = (int16_t)(data_i);
					tmp_q = (int16_t)(data_q);

					pTx[index] = tmp_i*1.0/32768 + I*(tmp_q*1.0/32768);

					data_i = (capTxBuf[index] >> 16) & 0xffff;
					data_q = (capTxBuf[index+1]>> 16) & 0xffff;
					tmp_i = (int16_t)(data_i);
					tmp_q = (int16_t)(data_q);

					pTx[index+1] = tmp_i*1.0/32768 + I*(tmp_q*1.0/32768);
				}
			}

			/* convert int32_t to double complex for ORx */
			for(uint16_t index = 0; index < DPD_CAP_SIZE; index++)
			{
				// ORx
				data_i = (wr_data[index*4 + 0] << 0) | (wr_data[index*4 + 1] << 8);
				data_q = (wr_data[index*4 + 2] << 0) | (wr_data[index*4 + 3] << 8);
#if 0
				tmp_i = (data_i > 32768-1) ? (data_i-65536) : data_i;
				tmp_q = (data_q > 32768-1) ? (data_q-65536) : data_q;
#else
				tmp_i = (int16_t)(data_i);
				tmp_q = (int16_t)(data_q);
#endif
				pORx[index] = tmp_i*1.0/32768 + I*(tmp_q*1.0/32768);
			}

			/* run dpd coeffs estimation */
			uint8_t capBatch = 1;
			dpdErr = dpd_CoeffEstimate(&dpdData,
										pTx,
										pORx,
										DPD_CAP_SIZE,
										capBatch
										);
		}

		/* 5.coeffs2luts */
		uint32_t lutScale = 32738; // 2^15
		for(uint8_t lutId = 0u; lutId < DPD_LUT_MAX; lutId++)
		{
			if(DPD_ERR_CODE_NO_ERROR == dpdErr)
			{
				dpdErr = WriteVBankLuts(&dpdData,
										lutEntries,
										lutId,
										lutScale,
										DPD_LUT_DEPTH);
			}
			else
			{
				break;
			}
		}
#if 1
		/* 6.bypass actuator */
		dpd_register_write(ADDR_ACT_OUT_SEL, DPD_BYPASS);
#endif
		/* 7.luts programming */
		if(DPD_ERR_CODE_NO_ERROR == dpdErr)
		{
			for(uint8_t lutId = 0u; lutId < DPD_LUT_MAX; lutId++)
			{
				dpd_luts_write(lutId, &lutEntries[lutId*DPD_LUT_DEPTH]);
				no_os_mdelay(1);
			}
		}

		/* 8.enable actuator */
		if(DPD_ERR_CODE_NO_ERROR == dpdErr)
		{
			dpd_register_write(ADDR_ACT_OUT_SEL, DPD_ENABLE);
		}

		/* 9. goto #3 if iters < max_iters */
		if(DPD_ERR_CODE_NO_ERROR != dpdErr)
		{
			break;
		}
		no_os_mdelay(100);
	}
	#endif
	return dpdErr;
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
	struct iio_dpd_device_data *pdata = (struct iio_dpd_device_data *)(dev->pdata);

	if (dev)
		dev->userdata = (void *) &dpd_device_data;
	else
		ret = -EFAULT;
	
	/* set the file handler of dpd "device" every time it inits 
	 * so that the close function can be called when release the context */
	pdata->fd = 1;

	return ret;
}

int iio_dpd_close(const struct iio_device *dev);

int iio_dpd_open(const struct iio_device *dev,
		size_t samples_count, bool cyclic)
{
	int ret = 0;
	return ret;
}

int iio_dpd_close(const struct iio_device *dev)
{
	int ret = 0;

	ret = dpd_hw_close();
	if(ret)
	{
		IIO_ERROR("Failed to close dpd hardware\n");
		goto out;
	}

	ret = _dpd_dev_destroy_fs();
	if(ret)
	{
		IIO_ERROR("Failed to destroy dpd device file system\n");
		goto out;
	}

out:
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
	ssize_t ret = -ENOSYS;
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
		case IIO_ATTR_TYPE_DEBUG:
			pattr = _dpd_get_dev_attr_by_name(attr);
			break;
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
		case IIO_ATTR_TYPE_DEBUG:
			pattr = _dpd_get_dev_attr_by_name(attr);
			break;
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
