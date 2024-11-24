#ifndef __DPD_T_H__
#define __DPD_T_H__

#include "iio-private.h"
#include <sys/types.h>

#ifndef __weak
#define __weak __attribute__((weak))
#endif

#define IIO_DPD_MAX_DEV_ATTR_CNT					64
#define IIO_DPD_MAX_CHAN_ATTR_CNT					64

#define IIO_DPD_ATTR_LEN							128
#define IIO_DPD_ATTR_NAME_LEN						128

struct iio_dpd_attr {
	char name[IIO_DPD_ATTR_NAME_LEN];		/* don't move the definition position */
	uint32_t id;							/* don't move the definition position */
	char file_name[IIO_DPD_ATTR_NAME_LEN];
	//uint32_t order;
	//void *pParent;
	ssize_t (*store)(const char *);
	ssize_t (*show)(char *);
};

struct iio_dpd_channel {
	char name[IIO_DPD_ATTR_NAME_LEN];
	uint32_t id;						/* index of channel */
	//uint32_t order;						/* the order of channel, start from 0 when it is under dpdData */
	//void *pParent;
	uint32_t attr_cnt;
	struct iio_dpd_attr **pp_attr_array;
};

#define TYPE_IS_ATTR	0
#define TYPE_IS_CHAN	1

struct iio_dpd_dev_attr {
	uint32_t id;
	uint32_t attr_type;
	void *pElement;
};

#define DECLARE_IIO_DPD_DATA												\
dpd_TrackData_t dpdData

extern dpd_TrackData_t dpdData;



#define IIO_DPD_ADD_CHAN_ATTR(chan, attr, index) 							\
ssize_t _dpd_##chan##_attr_##index##_show(char *dst)			\
{																			\
	ssize_t ret;															\
	ret = iio_snprintf(dst, IIO_DPD_ATTR_LEN, "0x%08x", (uint32_t)(dpdData.p##chan->attr));	\
																			\
	if (ret > 0)															\
		dst[ret] = '\0';												\
	else																	\
		dst[0] = '\0';														\
	return ret ? ret : -EIO;												\
}																			\
ssize_t _dpd_##chan##_attr_##index##_store(const char *src)			\
{																			\
	__typeof__(dpdData.p##chan->attr) val = 0;								\
	char *end;																\
	ssize_t ret=0;															\
																			\
	val = (__typeof__(dpdData.p##chan->attr))strtoul(src, &end, 16);		\
	ret = strlen(src);														\
	dpdData.p##chan->attr = val;											\
	return ret ? ret : -EIO;												\
}																			\
struct iio_dpd_attr chan##_attr_##index##_t = {								\
	.name = #attr,															\
	.id = index,															\
	.show = &_dpd_##chan##_attr_##index##_show, 							\
	.store = &_dpd_##chan##_attr_##index##_store, 							\
}

#define IIO_DPD_ADD_UNIQUE_CHAN_ATTR(chan, attr, show_cb, store_cb, index) 	\
struct iio_dpd_attr chan##_attr_##index##_t = {								\
	.name = #attr,															\
	.id = index,															\
	.show = show_cb, 														\
	.store = store_cb, 														\
}

#define ADD_CHAN_ATTR_ARRAY_ELEMENT_START(chan)								\
struct iio_dpd_attr *iio_dpd_##chan##array[IIO_DPD_MAX_CHAN_ATTR_CNT] = {

#define ADD_CHAN_ATTR_ARRAY_ELEMENT(chan, attr, index) 						\
	[index] = &chan##_attr_##index##_t

#define ADD_CHAN_ATTR_ARRAY_ELEMENT_END(chan)								\
	NULL,																	\
}

#define IIO_DPD_ADD_DEV_CHAN(chan, index)									\
struct iio_dpd_channel dpd_attr_##index##_t = {								\
	.name = #chan,															\
	.id = index,															\
	.pp_attr_array = iio_dpd_##chan##array, 								\
}


#define IIO_DPD_ADD_DEV_DEFAULT_ATTR(attr, index) 							\
static ssize_t _dpd_dev_attr_##index##_show(char *dst)						\
{																			\
	ssize_t ret=0;															\
	ret = iio_snprintf(dst, IIO_DPD_ATTR_LEN, "0x%08x", (uint32_t)(dpdData.attr));	\
																			\
	if (ret > 0)															\
		dst[ret - 1] = '\0';												\
	else																	\
		dst[0] = '\0';														\
	return ret ? ret : -EIO;												\
}																			\
static ssize_t _dpd_dev_attr_##index##_store(const char *src)				\
{																			\
	__typeof__(dpdData.attr) val = 0;										\
	char *end;																\
	ssize_t ret=0;															\
																			\
	val = (__typeof__(dpdData.attr))strtoul(src, &end, 16);					\
	ret = strlen(src);														\
	dpdData.attr = val;														\
	return ret ? ret : -EIO;												\
}																			\
struct iio_dpd_attr dpd_attr_##index##_t = {								\
	.name = #attr,															\
	.id = index,															\
	.show = &_dpd_dev_attr_##index##_show, 									\
	.store = &_dpd_dev_attr_##index##_store, 								\
}


#define IIO_DPD_ADD_DEV_UNIQUE_ATTR(attr, show_cb, store_cb, index) 		\
struct iio_dpd_attr dpd_attr_##index##_t = {								\
	.name = #attr,															\
	.id = index,															\
	.show = show_cb, 														\
	.store = store_cb, 														\
}

#define ADD_DEV_ATTR_ARRAY_ELEMENT_START()									\
struct iio_dpd_dev_attr iio_dpd_dev_array[IIO_DPD_MAX_CHAN_ATTR_CNT] = {

#define ADD_DEV_ATTR_ARRAY_ELEMENT(type, attr, index) 						\
	[index] = {																\
		.id = index,														\
		.attr_type = type,													\
		.pElement = (void *)&dpd_attr_##index##_t									\
	}

#define ADD_DEV_ATTR_ARRAY_ELEMENT_END()									\
	{0,0,NULL},																	\
}


#endif
