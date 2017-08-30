/*!
 * @section LICENSE
 * (C) Copyright 2013 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename bmg160_driver.c
 * @date     2014/03/11 14:20
 * @id       "7bf4b97"
 * @version  1.5.6
 *
 * @brief    BMG160 Linux Driver
 */
#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/unistd.h>
#include <linux/types.h>
#include <linux/string.h>
#else
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#endif

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include "bmg160_main.h"

/* sensor specific */
#define SENSOR_NAME "bmg160"

#define SENSOR_CHIP_ID_BMG (0x0f)
#define CHECK_CHIP_ID_TIME_MAX   5

#define BMG_REG_NAME(name) BMG160_##name
#define BMG_VAL_NAME(name) BMG160_##name
#define BMG_CALL_API(name) bmg160_##name

#define BMG_SPI_WRITE_DELAY_TIME 2

/* generic */

#define BMG_DELAY_MIN (1)
#define BMG_DELAY_DEFAULT (200)

#define BMG_VALUE_MAX (32767)
#define BMG_VALUE_MIN (-32768)

#define BYTES_PER_LINE (16)

#define BMG_SELF_TEST 0

#define BMG_SOFT_RESET_VALUE                0xB6

#define MAX_FIFO_F_LEVEL 100
#define MAX_FIFO_F_BYTES 8
#define BMG160_FIFO_DAT_SEL_X                     1
#define BMG160_FIFO_DAT_SEL_Y                     2
#define BMG160_FIFO_DAT_SEL_Z                     3

/*! Bosch sensor unknown place*/
#define BOSCH_SENSOR_PLACE_UNKNOWN (-1)
/*! Bosch sensor remapping table size P0~P7*/
#define MAX_AXIS_REMAP_TAB_SZ 8


struct bosch_sensor_specific {
	char *name;
	/* 0 to 7 */
	unsigned int place:3;
	int irq;
	int (*irq_gpio_cfg)(void);
};


/*!
 * we use a typedef to hide the detail,
 * because this type might be changed
 */
struct bosch_sensor_axis_remap {
	/* src means which source will be mapped to target x, y, z axis */
	/* if an target OS axis is remapped from (-)x,
	 * src is 0, sign_* is (-)1 */
	/* if an target OS axis is remapped from (-)y,
	 * src is 1, sign_* is (-)1 */
	/* if an target OS axis is remapped from (-)z,
	 * src is 2, sign_* is (-)1 */
	int src_x:3;
	int src_y:3;
	int src_z:3;

	int sign_x:2;
	int sign_y:2;
	int sign_z:2;
};


struct bosch_sensor_data {
	union {
		int16_t v[3];
		struct {
			int16_t x;
			int16_t y;
			int16_t z;
		};
	};
};

struct bmg_device_data {
	struct bmg160_t device;
	struct spi_device *spi;
	struct input_dev *input;
	struct delayed_work work;

	atomic_t delay;

	struct bmg160_data_t value;
	u8 enable:1;
	unsigned int fifo_count;
	unsigned char fifo_datasel;

	/* controls not only reg, but also workqueue */
	struct mutex mutex_op_mode;
	struct mutex mutex_enable;
	struct bosch_sensor_specific *bst_pd;
};

static struct spi_device *bmg_device;
/* spi operation for API */
static void bmg_spi_delay(BMG160_U16 msec);
static int bmg_spi_read(struct spi_device *spi, u8 reg_addr,
		u8 *data, u8 len);
static int bmg_spi_write(struct spi_device *spi, u8 reg_addr,
		u8 *data, u8 len);

static void bmg_dump_reg(struct spi_device *spi);
static int bmg_check_chip_id(struct spi_device *spi);

/*!
* BMG160 sensor remapping function
* need to give some parameter in BSP files first.
*/
static const struct bosch_sensor_axis_remap
	bst_axis_remap_tab_dft[MAX_AXIS_REMAP_TAB_SZ] = {
	/* src_x src_y src_z  sign_x  sign_y  sign_z */
	{  0,	 1,    2,	  1,	  1,	  1 }, /* P0 */
	{  1,	 0,    2,	  1,	 -1,	  1 }, /* P1 */
	{  0,	 1,    2,	 -1,	 -1,	  1 }, /* P2 */
	{  1,	 0,    2,	 -1,	  1,	  1 }, /* P3 */

	{  0,	 1,    2,	 -1,	  1,	 -1 }, /* P4 */
	{  1,	 0,    2,	 -1,	 -1,	 -1 }, /* P5 */
	{  0,	 1,    2,	  1,	 -1,	 -1 }, /* P6 */
	{  1,	 0,    2,	  1,	  1,	 -1 }, /* P7 */
};

static void bst_remap_sensor_data(struct bosch_sensor_data *data,
			const struct bosch_sensor_axis_remap *remap)
{
	struct bosch_sensor_data tmp;

	tmp.x = data->v[remap->src_x] * remap->sign_x;
	tmp.y = data->v[remap->src_y] * remap->sign_y;
	tmp.z = data->v[remap->src_z] * remap->sign_z;

	memcpy(data, &tmp, sizeof(*data));
}

static void bst_remap_sensor_data_dft_tab(struct bosch_sensor_data *data,
			int place)
{
/* sensor with place 0 needs not to be remapped */
	if ((place <= 0) || (place >= MAX_AXIS_REMAP_TAB_SZ))
		return;
	bst_remap_sensor_data(data, &bst_axis_remap_tab_dft[place]);
}

static void bmg160_remap_sensor_data(struct bmg160_data_t *val,
		struct bmg_device_data *device_data)
{
	struct bosch_sensor_data bsd;

	if ((NULL == device_data->bst_pd) ||
			(BOSCH_SENSOR_PLACE_UNKNOWN
			 == device_data->bst_pd->place))
		return;

	bsd.x = val->datax;
	bsd.y = val->datay;
	bsd.z = val->dataz;

	bst_remap_sensor_data_dft_tab(&bsd,
			device_data->bst_pd->place);

	val->datax = bsd.x;
	val->datay = bsd.y;
	val->dataz = bsd.z;

}

static int bmg_check_chip_id(struct spi_device *spi)
{
	int err = -1;
	u8 chip_id = 0;
	u8 read_count = 0;

	while (read_count++ < CHECK_CHIP_ID_TIME_MAX) {
		bmg_spi_read(spi, BMG_REG_NAME(CHIP_ID_ADDR), &chip_id, 1);
		dev_info(&spi->dev, "read chip id result: %#x", chip_id);

		if ((chip_id & 0xff) != SENSOR_CHIP_ID_BMG) {
			mdelay(1);
		} else {
			err = 0;
			break;
		}
	}
	return err;
}

static void bmg_spi_delay(BMG160_U16 msec)
{
	mdelay(msec);
}

static void bmg_dump_reg(struct spi_device *spi)
{
	int i;
	u8 dbg_buf[64];
	u8 dbg_buf_str[64 * 3 + 1] = "";

	for (i = 0; i < BYTES_PER_LINE; i++) {
		dbg_buf[i] = i;
		sprintf(dbg_buf_str + i * 3, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	dev_dbg(&spi->dev, "%s\n", dbg_buf_str);

	bmg_spi_read(spi, BMG_REG_NAME(CHIP_ID_ADDR), dbg_buf, 64);
	for (i = 0; i < 64; i++) {
		sprintf(dbg_buf_str + i * 3, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	dev_dbg(&spi->dev, "%s\n", dbg_buf_str);
}

/*spi read routine for API*/
static int bmg_spi_read_byte(struct spi_device *spi, u8 reg_addr,
		u8 *data)
{
	struct spi_transfer	t = {0};
	s32 dummy = 0;
	u8 *frame = kzalloc(2, GFP_KERNEL);
	u8 *buf = kzalloc(2, GFP_KERNEL);

	frame[1] = 0x80 | reg_addr;
	//*frame = 0x8000 | (reg_addr << 8);

	t.tx_buf = frame;
	t.rx_buf = buf;
	t.len = 2;
	t.bits_per_word = 2 * 8;

	dummy = spi_sync_transfer(spi, &t, 1);
	if (dummy < 0) {
		dev_err(&spi->dev, "SPI read error: %d\n", dummy);
		kfree(frame);
		kfree(buf);
		return -1;
	}

	data[0] = buf[0];

	kfree(frame);
	kfree(buf);
	return 0;
}

/*spi read routine for API*/
static int bmg_spi_read(struct spi_device *spi, u8 reg_addr,
		u8 *data, u8 len)
{
	u8 i = 0;
	s32 dummy = 0;

	for(i=0 ; i<len ; i++) {
		dummy = bmg_spi_read_byte(spi, reg_addr + i, &data[i]);
		if (dummy < 0) {
			dev_err(&spi->dev, "SPI read block error: %d\n", dummy);
			return -1;
		}
	}

	return 0;
}

static int bmg_spi_write_byte(struct spi_device *spi, u8 reg_addr,
		u8 *data)
{
	struct spi_transfer	t = {0};
	s32 dummy = 0;
	u8 *frame = kzalloc(2, GFP_KERNEL);

	frame[1] = 0x7F & reg_addr;
	frame[0] = *data;
	//*frame = 0x7FFF & ((reg_addr << 8) | *data);

	t.tx_buf = frame;
	t.len = 2;
	t.bits_per_word = 2 * 8;

	dummy = spi_sync_transfer(spi, &t, 1);
	if (dummy < 0) {
		dev_err(&spi->dev, "SPI write error: %d\n", dummy);
		kfree(frame);
		return -1;
	}

	kfree(frame);
	return 0;
}

/*spi write routine for */
static int bmg_spi_write(struct spi_device *spi, u8 reg_addr,
		u8 *data, u8 len)
{
	u8 i = 0;
	s32 dummy = 0;

	for(i=0 ; i<len ; i++) {
		dummy = bmg_spi_write_byte(spi, reg_addr + i, &data[i]);
		if (dummy < 0) {
			dev_err(&spi->dev, "SPI write block error: %d\n", dummy);
			return -1;
		}
	}

	return 0;
}

static int bmg_spi_read_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	int err;
	err = bmg_spi_read(bmg_device, reg_addr, data, len);
	return err;
}

static int bmg_spi_write_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	int err;
	err = bmg_spi_write(bmg_device, reg_addr, data, len);
	return err;
}


static void bmg_work_func(struct work_struct *work)
{
	struct bmg_device_data *device_data =
		container_of((struct delayed_work *)work,
			struct bmg_device_data, work);

	unsigned long delay =
		msecs_to_jiffies(atomic_read(&device_data->delay));
	struct bmg160_data_t gyro_data;

	BMG_CALL_API(get_dataXYZ)(&gyro_data);
	/*remapping for BMG160 sensor*/
	bmg160_remap_sensor_data(&gyro_data, device_data);

	input_report_abs(device_data->input, ABS_X, gyro_data.datax);
	input_report_abs(device_data->input, ABS_Y, gyro_data.datay);
	input_report_abs(device_data->input, ABS_Z, gyro_data.dataz);
	input_sync(device_data->input);

	schedule_delayed_work(&device_data->work, delay);
}

static int bmg_set_soft_reset(struct spi_device *spi)
{
	int err = 0;
	unsigned char data = BMG_SOFT_RESET_VALUE;
	err = bmg_spi_write(spi, BMG160_BGW_SOFTRESET_ADDR, &data, 1);
	return err;
}

static ssize_t bmg_show_chip_id(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", SENSOR_CHIP_ID_BMG);
}

static ssize_t bmg_show_op_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_device_data *device_data = input_get_drvdata(input);
	u8 op_mode = 0xff;

	mutex_lock(&device_data->mutex_op_mode);
	BMG_CALL_API(get_mode)(&op_mode);
	mutex_unlock(&device_data->mutex_op_mode);

	ret = sprintf(buf, "%d\n", op_mode);

	return ret;
}

static ssize_t bmg_store_op_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_device_data *device_data = input_get_drvdata(input);

	long op_mode;

	err = kstrtoul(buf, 10, &op_mode);
	if (err)
		return err;

	mutex_lock(&device_data->mutex_op_mode);

	err = BMG_CALL_API(set_mode)(op_mode);

	mutex_unlock(&device_data->mutex_op_mode);

	if (err)
		return err;
	else
		return count;
}



static ssize_t bmg_show_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmg_device_data *device_data = input_get_drvdata(input);
	int count;

	struct bmg160_data_t value_data;
	BMG_CALL_API(get_dataXYZ)(&value_data);
	/*BMG160 sensor raw data remapping*/
	bmg160_remap_sensor_data(&value_data, device_data);

	count = sprintf(buf, "%hd %hd %hd\n",
				value_data.datax,
				value_data.datay,
				value_data.dataz);

	return count;
}

static ssize_t bmg_show_range(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char range = 0;
	BMG_CALL_API(get_range_reg)(&range);
	err = sprintf(buf, "%d\n", range);
	return err;
}

static ssize_t bmg_store_range(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long range;
	err = kstrtoul(buf, 10, &range);
	if (err)
		return err;
	BMG_CALL_API(set_range_reg)(range);
	return count;
}

static ssize_t bmg_show_bandwidth(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char bandwidth = 0;
	BMG_CALL_API(get_bw)(&bandwidth);
	err = sprintf(buf, "%d\n", bandwidth);
	return err;
}

static ssize_t bmg_store_bandwidth(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long bandwidth;
	err = kstrtoul(buf, 10, &bandwidth);
	if (err)
		return err;
	BMG_CALL_API(set_bw)(bandwidth);
	return count;
}


static ssize_t bmg_show_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmg_device_data *device_data = input_get_drvdata(input);
	int err;

	mutex_lock(&device_data->mutex_enable);
	err = sprintf(buf, "%d\n", device_data->enable);
	mutex_unlock(&device_data->mutex_enable);
	return err;
}

static ssize_t bmg_store_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_device_data *device_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;

	data = data ? 1 : 0;
	mutex_lock(&device_data->mutex_enable);
	if (data != device_data->enable) {
		if (data) {
			schedule_delayed_work(
					&device_data->work,
					msecs_to_jiffies(atomic_read(
							&device_data->delay)));
		} else {
			cancel_delayed_work_sync(&device_data->work);
		}

		device_data->enable = data;
	}
	mutex_unlock(&device_data->mutex_enable);

	return count;
}

static ssize_t bmg_show_delay(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmg_device_data *device_data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", atomic_read(&device_data->delay));

}

static ssize_t bmg_store_delay(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_device_data *device_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;

	if (data == 0) {
		err = -EINVAL;
		return err;
	}

	if (data < BMG_DELAY_MIN)
		data = BMG_DELAY_MIN;

	atomic_set(&device_data->delay, data);

	return count;
}


static ssize_t bmg_store_fastoffset_en(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long fastoffset_en;
	err = kstrtoul(buf, 10, &fastoffset_en);
	if (err)
		return err;
	if (fastoffset_en) {

		BMG_CALL_API(set_fast_offset_en_ch)(BMG160_X_AXIS, 1);
		BMG_CALL_API(set_fast_offset_en_ch)(BMG160_Y_AXIS, 1);
		BMG_CALL_API(set_fast_offset_en_ch)(BMG160_Z_AXIS, 1);
		BMG_CALL_API(enable_fast_offset)();
	}
	return count;
}

static ssize_t bmg_store_slowoffset_en(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long slowoffset_en;
	err = kstrtoul(buf, 10, &slowoffset_en);
	if (err)
		return err;
	if (slowoffset_en) {
		BMG_CALL_API(set_slow_offset_th)(3);
		BMG_CALL_API(set_slow_offset_dur)(0);
		BMG_CALL_API(set_slow_offset_en_ch)(BMG160_X_AXIS, 1);
		BMG_CALL_API(set_slow_offset_en_ch)(BMG160_Y_AXIS, 1);
		BMG_CALL_API(set_slow_offset_en_ch)(BMG160_Z_AXIS, 1);
	} else {
	BMG_CALL_API(set_slow_offset_en_ch)(BMG160_X_AXIS, 0);
	BMG_CALL_API(set_slow_offset_en_ch)(BMG160_Y_AXIS, 0);
	BMG_CALL_API(set_slow_offset_en_ch)(BMG160_Z_AXIS, 0);
	}

	return count;
}

static ssize_t bmg_show_selftest(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char selftest;
	BMG_CALL_API(selftest)(&selftest);
	err = sprintf(buf, "%d\n", selftest);
	return err;
}

static ssize_t bmg_show_sleepdur(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char sleepdur;
	BMG_CALL_API(get_sleepdur)(&sleepdur);
	err = sprintf(buf, "%d\n", sleepdur);
	return err;
}

static ssize_t bmg_store_sleepdur(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long sleepdur;
	err = kstrtoul(buf, 10, &sleepdur);
	if (err)
		return err;
	BMG_CALL_API(set_sleepdur)(sleepdur);
	return count;
}

static ssize_t bmg_show_autosleepdur(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char autosleepdur;
	BMG_CALL_API(get_autosleepdur)(&autosleepdur);
	err = sprintf(buf, "%d\n", autosleepdur);
	return err;
}

static ssize_t bmg_store_autosleepdur(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long autosleepdur;
	unsigned char bandwidth;
	err = kstrtoul(buf, 10, &autosleepdur);
	if (err)
		return err;
	BMG_CALL_API(get_bw)(&bandwidth);
	BMG_CALL_API(set_autosleepdur)(autosleepdur, bandwidth);
	return count;
}

static ssize_t bmg_show_fifo_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char fifo_mode;
	BMG_CALL_API(get_fifo_mode)(&fifo_mode);
	err = sprintf(buf, "%d\n", fifo_mode);
	return err;
}

static ssize_t bmg_store_fifo_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long fifo_mode;
	err = kstrtoul(buf, 10, &fifo_mode);
	if (err)
		return err;
	BMG_CALL_API(set_fifo_mode)(fifo_mode);
	return count;
}

static ssize_t bmg_show_fifo_framecount(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char fifo_framecount;
	BMG_CALL_API(get_fifo_framecount)(&fifo_framecount);
	err = sprintf(buf, "%d\n", fifo_framecount);
	return err;
}

static ssize_t bmg_store_fifo_framecount(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_device_data *device_data = input_get_drvdata(input);
	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	device_data->fifo_count = (unsigned int) data;

	return count;
}

static ssize_t bmg_show_fifo_overrun(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char fifo_overrun;
	BMG_CALL_API(get_fifo_overrun)(&fifo_overrun);
	err = sprintf(buf, "%d\n", fifo_overrun);
	return err;
}

/*!
 * brief: bmg single axis data remaping
 * @param[i] fifo_datasel   fifo axis data select setting
 * @param[i/o] remap_dir   remapping direction
 * @param[i] device_data   to transfer sensor place
 *
 * @return none
 */
static void bmg_single_axis_remaping(unsigned char fifo_datasel,
		unsigned char *remap_dir, struct bmg_device_data *device_data)
{
	if ((NULL == device_data->bst_pd) ||
			(BOSCH_SENSOR_PLACE_UNKNOWN
			 == device_data->bst_pd->place))
		return;
	else {
		signed char place = device_data->bst_pd->place;
		/* sensor with place 0 needs not to be remapped */
		if ((place <= 0) ||
			(place >= MAX_AXIS_REMAP_TAB_SZ))
			return;

		if (fifo_datasel < 1 || fifo_datasel > 3)
			return;
		else {
			switch (fifo_datasel) {
			/*P2, P3, P4, P5 X axis(andorid) need to reverse*/
			case BMG160_FIFO_DAT_SEL_X:
				if (-1 == bst_axis_remap_tab_dft[place].sign_x)
					*remap_dir = 1;
				else
					*remap_dir = 0;
				break;
			/*P1, P2, P5, P6 Y axis(andorid) need to reverse*/
			case BMG160_FIFO_DAT_SEL_Y:
				if (-1 == bst_axis_remap_tab_dft[place].sign_y)
					*remap_dir = 1;
				else
					*remap_dir = 0;
				break;
			case BMG160_FIFO_DAT_SEL_Z:
			/*P4, P5, P6, P7 Z axis(andorid) need to reverse*/
				if (-1 == bst_axis_remap_tab_dft[place].sign_z)
					*remap_dir = 1;
				else
					*remap_dir = 0;
				break;
			default:
				break;
			}
		}
	}

	return;
}

static ssize_t bmg_show_fifo_data_frame(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err, i, len;
	signed char fifo_data_out[MAX_FIFO_F_LEVEL * MAX_FIFO_F_BYTES] = {0};
	unsigned char f_len = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_device_data *device_data = input_get_drvdata(input);
	struct bmg160_data_t gyro_lsb;
	unsigned char axis_dir_remap = 0;
	s16 value;

	if (device_data->fifo_count == 0)
		return -ENOENT;

	if (device_data->fifo_datasel)
		/*Select one axis data output for every fifo frame*/
		f_len = 2;
	else
		/*Select X Y Z axis data output for every fifo frame*/
		f_len = 6;

	bmg_spi_read(device_data->spi, BMG160_FIFO_DATA_ADDR,
			fifo_data_out, device_data->fifo_count * f_len);
	err = 0;

	if (f_len == 6) {
		/* Select X Y Z axis data output for every frame */
		for (i = 0; i < device_data->fifo_count; i++) {
			gyro_lsb.datax =
			((unsigned char)fifo_data_out[i * f_len + 1] << 8
				| (unsigned char)fifo_data_out[i * f_len + 0]);

			gyro_lsb.datay =
			((unsigned char)fifo_data_out[i * f_len + 3] << 8
				| (unsigned char)fifo_data_out[i * f_len + 2]);

			gyro_lsb.dataz =
			((unsigned char)fifo_data_out[i * f_len + 5] << 8
				| (unsigned char)fifo_data_out[i * f_len + 4]);

			bmg160_remap_sensor_data(&gyro_lsb, device_data);
			len = sprintf(buf, "%d %d %d ",
				gyro_lsb.datax, gyro_lsb.datay, gyro_lsb.dataz);
			buf += len;
			err += len;
		}
	} else {
		/* single axis data output for every frame */
		bmg_single_axis_remaping(device_data->fifo_datasel,
				&axis_dir_remap, device_data);
		for (i = 0; i < device_data->fifo_count * f_len / 2; i++) {
			value = ((unsigned char)fifo_data_out[2 * i + 1] << 8 |
					(unsigned char)fifo_data_out[2 * i]);
			if (axis_dir_remap)
				value = 0 - value;
			len = sprintf(buf, "%d ", value);
			buf += len;
			err += len;
		}
	}

	return err;
}

/*!
 * @brief show fifo_data_sel axis definition(Android definition, not sensor HW reg).
 * 0--> x, y, z axis fifo data for every frame
 * 1--> only x axis fifo data for every frame
 * 2--> only y axis fifo data for every frame
 * 3--> only z axis fifo data for every frame
 */
static ssize_t bmg_show_fifo_data_sel(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char fifo_data_sel;
	struct spi_device *spi = to_spi_device(dev);
	struct bmg_device_data *device_data = spi_get_drvdata(spi);
	signed char place = BOSCH_SENSOR_PLACE_UNKNOWN;

	BMG_CALL_API(get_fifo_data_sel)(&fifo_data_sel);

	/*remapping fifo_dat_sel if define virtual place in BSP files*/
	if ((NULL != device_data->bst_pd) &&
		(BOSCH_SENSOR_PLACE_UNKNOWN != device_data->bst_pd->place)) {
		place = device_data->bst_pd->place;
		/* sensor with place 0 needs not to be remapped */
		if ((place > 0) && (place < MAX_AXIS_REMAP_TAB_SZ)) {
			if (BMG160_FIFO_DAT_SEL_X == fifo_data_sel)
				/* BMG160_FIFO_DAT_SEL_X: 1, Y:2, Z:3;
				*bst_axis_remap_tab_dft[i].src_x:0, y:1, z:2
				*so we need to +1*/
				fifo_data_sel =
					bst_axis_remap_tab_dft[place].src_x + 1;

			else if (BMG160_FIFO_DAT_SEL_Y == fifo_data_sel)
				fifo_data_sel =
					bst_axis_remap_tab_dft[place].src_y + 1;
		}

	}

	err = sprintf(buf, "%d\n", fifo_data_sel);
	return err;
}

/*!
 * @brief store fifo_data_sel axis definition(Android definition, not sensor HW reg).
 * 0--> x, y, z axis fifo data for every frame
 * 1--> only x axis fifo data for every frame
 * 2--> only y axis fifo data for every frame
 * 3--> only z axis fifo data for every frame
 */
static ssize_t bmg_store_fifo_data_sel(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)

{
	int err;
	unsigned long fifo_data_sel;

	struct input_dev *input = to_input_dev(dev);
	struct bmg_device_data *device_data = input_get_drvdata(input);
	signed char place;

	err = kstrtoul(buf, 10, &fifo_data_sel);
	if (err)
		return err;

	/*save fifo_data_sel(android axis definition)*/
	device_data->fifo_datasel = (unsigned char) fifo_data_sel;

	/*remaping fifo_dat_sel if define virtual place*/
	if ((NULL != device_data->bst_pd) &&
		(BOSCH_SENSOR_PLACE_UNKNOWN != device_data->bst_pd->place)) {
		place = device_data->bst_pd->place;
		/* sensor with place 0 needs not to be remapped */
		if ((place > 0) && (place < MAX_AXIS_REMAP_TAB_SZ)) {
			/*Need X Y axis revesal sensor place: P1, P3, P5, P7 */
			/* BMG160_FIFO_DAT_SEL_X: 1, Y:2, Z:3;
			  * but bst_axis_remap_tab_dft[i].src_x:0, y:1, z:2
			  * so we need to +1*/
			if (BMG160_FIFO_DAT_SEL_X == fifo_data_sel)
				fifo_data_sel =
					bst_axis_remap_tab_dft[place].src_x + 1;

			else if (BMG160_FIFO_DAT_SEL_Y == fifo_data_sel)
				fifo_data_sel =
					bst_axis_remap_tab_dft[place].src_y + 1;
		}
	}

	if (BMG_CALL_API(set_fifo_data_sel)(fifo_data_sel) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bmg_show_fifo_tag(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char fifo_tag;
	BMG_CALL_API(get_fifo_tag)(&fifo_tag);
	err = sprintf(buf, "%d\n", fifo_tag);
	return err;
}

static ssize_t bmg_store_fifo_tag(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)

{
	int err;
	unsigned long fifo_tag;
	err = kstrtoul(buf, 10, &fifo_tag);
	if (err)
		return err;
	BMG_CALL_API(set_fifo_tag)(fifo_tag);
	return count;
}

static DEVICE_ATTR(chip_id, S_IRUGO,
		bmg_show_chip_id, NULL);
static DEVICE_ATTR(op_mode, S_IRUGO|S_IWUSR,
		bmg_show_op_mode, bmg_store_op_mode);
static DEVICE_ATTR(value, S_IRUGO,
		bmg_show_value, NULL);
static DEVICE_ATTR(range, S_IRUGO|S_IWUSR,
		bmg_show_range, bmg_store_range);
static DEVICE_ATTR(bandwidth, S_IRUGO|S_IWUSR,
		bmg_show_bandwidth, bmg_store_bandwidth);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR,
		bmg_show_enable, bmg_store_enable);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR,
		bmg_show_delay, bmg_store_delay);
static DEVICE_ATTR(fastoffset_en, S_IRUGO|S_IWUSR,
		NULL, bmg_store_fastoffset_en);
static DEVICE_ATTR(slowoffset_en, S_IRUGO|S_IWUSR,
		NULL, bmg_store_slowoffset_en);
static DEVICE_ATTR(selftest, S_IRUGO,
		bmg_show_selftest, NULL);
static DEVICE_ATTR(sleepdur, S_IRUGO|S_IWUSR,
		bmg_show_sleepdur, bmg_store_sleepdur);
static DEVICE_ATTR(autosleepdur, S_IRUGO|S_IWUSR,
		bmg_show_autosleepdur, bmg_store_autosleepdur);
static DEVICE_ATTR(fifo_mode, S_IRUGO|S_IWUSR,
		bmg_show_fifo_mode, bmg_store_fifo_mode);
static DEVICE_ATTR(fifo_framecount, S_IRUGO|S_IWUSR,
		bmg_show_fifo_framecount, bmg_store_fifo_framecount);
static DEVICE_ATTR(fifo_overrun, S_IRUGO|S_IWUSR,
		bmg_show_fifo_overrun, NULL);
static DEVICE_ATTR(fifo_data_frame, S_IRUGO|S_IWUSR,
		bmg_show_fifo_data_frame, NULL);
static DEVICE_ATTR(fifo_data_sel, S_IRUGO|S_IWUSR,
		bmg_show_fifo_data_sel, bmg_store_fifo_data_sel);
static DEVICE_ATTR(fifo_tag, S_IRUGO|S_IWUSR,
		bmg_show_fifo_tag, bmg_store_fifo_tag);

static struct attribute *bmg_attributes[] = {
	&dev_attr_chip_id.attr,
	&dev_attr_op_mode.attr,
	&dev_attr_value.attr,
	&dev_attr_range.attr,
	&dev_attr_bandwidth.attr,
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_fastoffset_en.attr,
	&dev_attr_slowoffset_en.attr,
	&dev_attr_selftest.attr,
	&dev_attr_sleepdur.attr,
	&dev_attr_autosleepdur.attr,
	&dev_attr_fifo_mode.attr,
	&dev_attr_fifo_framecount.attr,
	&dev_attr_fifo_overrun.attr,
	&dev_attr_fifo_data_frame.attr,
	&dev_attr_fifo_data_sel.attr,
	&dev_attr_fifo_tag.attr,
	NULL
};

static struct attribute_group bmg_attribute_group = {
	.attrs = bmg_attributes
};


static int bmg_input_init(struct bmg_device_data *device_data)
{
	struct input_dev *dev;
	int err = 0;

	dev = input_allocate_device();
	if (NULL == dev)
		return -ENOMEM;

	dev->name = SENSOR_NAME;
	dev->id.bustype = BUS_SPI;

	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_abs_params(dev, ABS_X, BMG_VALUE_MIN, BMG_VALUE_MAX, 0, 0);
	input_set_abs_params(dev, ABS_Y, BMG_VALUE_MIN, BMG_VALUE_MAX, 0, 0);
	input_set_abs_params(dev, ABS_Z, BMG_VALUE_MIN, BMG_VALUE_MAX, 0, 0);
	input_set_drvdata(dev, device_data);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	device_data->input = dev;

	return 0;
}

static void bmg_input_destroy(struct bmg_device_data *device_data)
{
	struct input_dev *dev = device_data->input;

	input_unregister_device(dev);
	input_free_device(dev);
}

static int bmg_probe(struct spi_device *spi)
{
	int err = 0;
	struct bmg_device_data *device_data = NULL;

	bmg_device = spi;

	dev_info(&spi->dev, "function entrance");

	device_data = kzalloc(sizeof(struct bmg_device_data), GFP_KERNEL);
	if (!device_data) {
		dev_err(&spi->dev, "no memory available");
		err = -ENOMEM;
		goto exit_err_clean;
	}

	/* do soft reset */
	mdelay(5);

	err = bmg_set_soft_reset(spi);
	if (err < 0) {
		dev_err(&spi->dev, "erro soft reset!\n");
		err = -EINVAL;
		goto exit_err_clean;
	}
	mdelay(5);

	/* check chip id */
	err = bmg_check_chip_id(spi);
	if (!err) {
		dev_notice(&spi->dev,
			"Bosch Sensortec Device %s detected", SENSOR_NAME);
	} else {
		dev_err(&spi->dev,
			"Bosch Sensortec Device not found, chip id mismatch");
		err = -1;
		goto exit_err_clean;
	}


	spi_set_drvdata(spi, device_data);
	device_data->spi = spi;

	mutex_init(&device_data->mutex_op_mode);
	mutex_init(&device_data->mutex_enable);

	/* input device init */
	err = bmg_input_init(device_data);
	if (err < 0)
		goto exit_err_clean;

	/* sysfs node creation */
	err = sysfs_create_group(&device_data->input->dev.kobj,
			&bmg_attribute_group);

	if (err < 0)
		goto exit_err_sysfs;

	if (NULL != spi->dev.platform_data) {
		device_data->bst_pd = kzalloc(sizeof(*device_data->bst_pd),
				GFP_KERNEL);

		if (NULL != device_data->bst_pd) {
			memcpy(device_data->bst_pd, spi->dev.platform_data,
					sizeof(*device_data->bst_pd));
			dev_notice(&spi->dev, "%s sensor driver set place: p%d",
					SENSOR_NAME,
					device_data->bst_pd->place);
		}
	}

	/* workqueue init */
	INIT_DELAYED_WORK(&device_data->work, bmg_work_func);
	atomic_set(&device_data->delay, BMG_DELAY_DEFAULT);

	/* h/w init */
	device_data->device.bus_read = bmg_spi_read_wrapper;
	device_data->device.bus_write = bmg_spi_write_wrapper;
	device_data->device.delay_msec = bmg_spi_delay;
	BMG_CALL_API(init)(&device_data->device);

	bmg_dump_reg(spi);

	device_data->enable = 0;
	device_data->fifo_datasel = 0;
	device_data->fifo_count = 0;

	/* now it's power on which is considered as resuming from suspend */
	err = BMG_CALL_API(set_mode)(
			BMG_VAL_NAME(MODE_SUSPEND));

	if (err < 0)
		goto exit_err_sysfs;

	dev_notice(&spi->dev, "sensor %s probed successfully", SENSOR_NAME);

	dev_dbg(&spi->dev,
		"spi_device: %p device_data: %p spi_device: %p input: %p",
		spi, device_data, &spi->dev, device_data->input);

	return 0;

exit_err_sysfs:
	if (err)
		bmg_input_destroy(device_data);

exit_err_clean:
	if (err) {
		if (device_data != NULL) {
			kfree(device_data);
			device_data = NULL;
		}

		bmg_device = NULL;
	}

	return err;
}

void bmg_shutdown(struct spi_device *spi)
{
	struct bmg_device_data *device_data =
		(struct bmg_device_data *)spi_get_drvdata(spi);

	mutex_lock(&device_data->mutex_op_mode);
	BMG_CALL_API(set_mode)(
		BMG_VAL_NAME(MODE_DEEPSUSPEND));
	mutex_unlock(&device_data->mutex_op_mode);
}

static int bmg_remove(struct spi_device *spi)
{
	int err = 0;
	u8 op_mode;

	struct bmg_device_data *device_data =
		(struct bmg_device_data *)spi_get_drvdata(spi);

	if (NULL != device_data) {
		mutex_lock(&device_data->mutex_op_mode);
		BMG_CALL_API(get_mode)(&op_mode);
		if (BMG_VAL_NAME(MODE_NORMAL) == op_mode) {
			cancel_delayed_work_sync(&device_data->work);
			dev_info(&spi->dev, "cancel work");
		}
		mutex_unlock(&device_data->mutex_op_mode);

		err = BMG_CALL_API(set_mode)(
				BMG_VAL_NAME(MODE_SUSPEND));
		mdelay(BMG_SPI_WRITE_DELAY_TIME);

		sysfs_remove_group(&device_data->input->dev.kobj,
				&bmg_attribute_group);
		bmg_input_destroy(device_data);
		kfree(device_data);

		bmg_device = NULL;
	}

	return err;
}

static const struct spi_device_id bmg_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(spi, bmg_id);

static struct spi_driver bmg_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SENSOR_NAME,
	},
	.id_table = bmg_id,
	.probe = bmg_probe,
	.remove = bmg_remove,
	.shutdown = bmg_shutdown,
};

static int __init BMG_init(void)
{
	return spi_register_driver(&bmg_driver);
}

static void __exit BMG_exit(void)
{
	spi_unregister_driver(&bmg_driver);
}

MODULE_AUTHOR("contact@bosch-sensortec.com>");
MODULE_DESCRIPTION("BMG GYROSCOPE SENSOR DRIVER");
MODULE_LICENSE("GPL v2");

module_init(BMG_init);
module_exit(BMG_exit);
