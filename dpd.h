#ifndef __DPD_H__
#define __DPD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "iio-private.h"
#include <sys/types.h>
#include "dpd_hw.h"

#define DPD_TMPFS_PATH 	"/mnt"
#define DPD_DEVICE_PATH "dpd"
#define DPD_DEVICE_DEBUG_PATH "debug"
#define DPD_DEVICE_SCAN_PATH "scan_elements"

#ifdef WITH_DPD
int iio_dpd_open(const struct iio_device *dev, size_t samples_count, bool cyclic);
int iio_dpd_close(const struct iio_device *dev);
ssize_t iio_dpd_write(const struct iio_device *dev, const void *src, size_t len);
ssize_t iio_dpd_read(const struct iio_device *dev,
		void *dst, size_t len, uint32_t *mask, size_t words);
ssize_t iio_dpd_write_chn_attr(const struct iio_channel *chn,
		const char *attr, const char *src, size_t len);
int iio_dpd_device_pre_init(void);
int iio_dpd_device_post_init(struct iio_device *dev);
ssize_t iio_dpd_read_dev_attr(const struct iio_device *dev,
		const char *attr, char *dst, size_t len, enum iio_attr_type type);
ssize_t iio_dpd_read_all_dev_attrs(const struct iio_device *dev,
		char *dst, size_t len, enum iio_attr_type type);
ssize_t iio_dpd_write_all_dev_attrs(const struct iio_device *dev,
		const char *src, size_t len, enum iio_attr_type type);
ssize_t iio_dpd_write_dev_attr(const struct iio_device *dev,
		const char *attr, const char *src, size_t len, enum iio_attr_type type);
ssize_t iio_dpd_read_all_chn_attrs(const struct iio_channel *chn,
		char *dst, size_t len);
ssize_t iio_dpd_read_chn_attr(const struct iio_channel *chn,
		const char *attr, char *dst, size_t len);
ssize_t iio_dpd_write_all_chn_attrs(const struct iio_channel *chn,
		const char *src, size_t len);
#else
int iio_dpd_open(const struct iio_device *dev, size_t samples_count, bool cyclic) {}
int iio_dpd_close(const struct iio_device *dev) {}
inline ssize_t iio_dpd_write(const struct iio_device *dev, const void *src, size_t len) {}
inline  ssize_t iio_dpd_read(const struct iio_device *dev,
		void *dst, size_t len, uint32_t *mask, size_t words) {}
inline ssize_t iio_dpd_write_chn_attr(const struct iio_channel *chn,
		const char *attr, const char *src, size_t len) {}
inline int iio_dpd_device_pre_init(void) {}
inline int iio_dpd_device_post_init(const struct iio_device *dev) {}
inline ssize_t iio_dpd_read_dev_attr(const struct iio_device *dev,
		const char *attr, char *dst, size_t len, enum iio_attr_type type) {}
inline ssize_t iio_dpd_read_all_dev_attrs(const struct iio_device *dev,
		char *dst, size_t len, enum iio_attr_type type) {}
inline ssize_t iio_dpd_write_all_dev_attrs(const struct iio_device *dev,
		const char *src, size_t len, enum iio_attr_type type) {}
inline ssize_t iio_dpd_write_dev_attr(const struct iio_device *dev,
		const char *attr, const char *src, size_t len, enum iio_attr_type type) {}
inline ssize_t iio_dpd_read_all_chn_attrs(const struct iio_channel *chn,
		char *dst, size_t len) {}
inline ssize_t iio_dpd_read_chn_attr(const struct iio_channel *chn,
		const char *attr, char *dst, size_t len) {}
inline ssize_t iio_dpd_write_all_chn_attrs(const struct iio_channel *chn,
		const char *src, size_t len) {}
#endif

#ifdef __cplusplus
}
#endif

#endif
