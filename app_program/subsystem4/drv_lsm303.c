/*
 *	drv_lsm303.c
 *	Subsystem 4 -- LSM303DLHC driver over the kernel I2C driver.
 *
 *	Needs DEVCNF_USE_IIC = 1 in config/config_device.h and the I2C0 pins
 *	set in device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h (shared files:
 *	see the subsystem 4 README for the agreed change).
 *
 *	Register map: ST LSM303DLHC datasheet, DocID018771.
 */

#include "drv_lsm303.h"	/* app.h -> tk/tkernel.h first */
#include <tk/device.h>	/* tk_opn_dev, T_I2C_EXEC, TDN_I2C_EXEC */

#if !DEVCNF_USE_IIC
#error "Subsystem 4 needs the I2C driver: set DEVCNF_USE_IIC to 1 in config/config_device.h"
#endif

/* Accelerometer registers */
#define CTRL_REG1_A	0x20
#define CTRL_REG4_A	0x23
#define OUT_X_L_A	0x28
#define AUTO_INC	0x80	/* set MSB of sub-address to read a block */

/* Magnetometer registers */
#define CRA_REG_M	0x00
#define CRB_REG_M	0x01
#define MR_REG_M	0x02
#define OUT_X_H_M	0x03	/* order: X, Z, Y, big-endian */
#define IRA_REG_M	0x0A	/* reads 'H' (0x48) */

/* Settings */
#define ACC_ODR_100HZ_XYZ	0x57	/* 100 Hz, normal mode, X/Y/Z on */
#define ACC_FS2G_HR		0x08	/* +-2 g, high resolution, little-endian */
#define MAG_ODR_75HZ		0x18	/* 75 Hz output rate */
#define MAG_GAIN_1_3		0x20	/* +-1.3 gauss */
#define MAG_CONTINUOUS		0x00

LOCAL ID	dd_iic = -1;		/* device descriptor, < 0 = closed */

/* LSM303D (one address for accel + mag, 0x1D with SA0 high or open,
   0x1E with SA0 low).  Detected at init when the DLHC does not answer. */
#define D_WHO_AM_I	0x0F	/* reads 0x49 */
#define D_ID		0x49
#define D_OUT_X_L_M	0x08
#define D_CTRL1		0x20
#define D_CTRL2		0x21
#define D_CTRL5		0x24
#define D_CTRL6		0x25
#define D_CTRL7		0x26
#define D_ACC_100HZ_XYZ	0x57	/* 100 Hz, X/Y/Z on, BDU off */
#define D_ACC_FS2G	0x00	/* +-2 g: 0.061 mg per LSB */
#define D_MAG_50HZ_HR	0x70	/* high resolution, 50 Hz */
#define D_MAG_FS4G	0x20	/* +-4 gauss */
#define D_MAG_CONT	0x00	/* continuous conversion */

LOCAL UW	d_addr;			/* 0 = LSM303DLHC, else LSM303D address */

LOCAL ER write_reg(UW sadr, UB reg, UB val)
{
	UB	buf[2];
	SZ	asz;

	buf[0] = reg;
	buf[1] = val;
	/* For "iic" devices the start parameter is the slave address */
	return tk_swri_dev(dd_iic, (W)sadr, buf, 2, &asz);
}

LOCAL ER read_regs(UW sadr, UB reg, UB *buf, SZ len)
{
	T_I2C_EXEC	ex;
	SZ		asz;

	ex.sadr     = sadr;
	ex.snd_size = 1;
	ex.snd_data = &reg;
	ex.rcv_size = len;
	ex.rcv_data = buf;
	/* Write the register address, repeated start, read len bytes */
	return tk_swri_dev(dd_iic, TDN_I2C_EXEC, &ex, sizeof(ex), &asz);
}

EXPORT void lsm303_close(void)
{
	if (dd_iic >= E_OK) {
		tk_cls_dev(dd_iic, 0);
		dd_iic = -1;
	}
}

EXPORT ER lsm303_init(void)
{
	UB	id;
	ER	er;

	if (dd_iic < E_OK) {
		dd_iic = tk_opn_dev((UB*)"iica", TD_UPDATE);
		if (dd_iic < E_OK) {
			return dd_iic;
		}
	}

	d_addr = 0;
	er = write_reg(LSM303_ACC_ADDR, CTRL_REG1_A, ACC_ODR_100HZ_XYZ);
	if (er < E_OK) {
		/* No LSM303DLHC accelerometer: try an LSM303D */
		UW	cand[2] = { 0x1D, 0x1E };
		INT	i;

		for (i = 0; i < 2; i++) {
			id = 0;
			if (read_regs(cand[i], D_WHO_AM_I, &id, 1) >= E_OK
			    && id == D_ID) {
				d_addr = cand[i];
				break;
			}
		}
		if (d_addr == 0) return er;
		tm_printf((UB*)"imu: LSM303D found at 0x%x\n", (INT)d_addr);
		if ((er = write_reg(d_addr, D_CTRL1, D_ACC_100HZ_XYZ)) < E_OK) return er;
		if ((er = write_reg(d_addr, D_CTRL2, D_ACC_FS2G)) < E_OK) return er;
		if ((er = write_reg(d_addr, D_CTRL5, D_MAG_50HZ_HR)) < E_OK) return er;
		if ((er = write_reg(d_addr, D_CTRL6, D_MAG_FS4G)) < E_OK) return er;
		if ((er = write_reg(d_addr, D_CTRL7, D_MAG_CONT)) < E_OK) return er;
		er = read_regs(d_addr, D_CTRL1, &id, 1);
		if (er < E_OK) return er;
		return (id == D_ACC_100HZ_XYZ) ? E_OK : E_IO;
	}
	er = write_reg(LSM303_ACC_ADDR, CTRL_REG4_A, ACC_FS2G_HR);
	if (er < E_OK) return er;

	er = write_reg(LSM303_MAG_ADDR, CRA_REG_M, MAG_ODR_75HZ);
	if (er < E_OK) return er;
	er = write_reg(LSM303_MAG_ADDR, CRB_REG_M, MAG_GAIN_1_3);
	if (er < E_OK) return er;
	er = write_reg(LSM303_MAG_ADDR, MR_REG_M, MAG_CONTINUOUS);
	if (er < E_OK) return er;

	/* Identity check: wrong chip or wiring gives a clear error */
	er = read_regs(LSM303_MAG_ADDR, IRA_REG_M, &id, 1);
	if (er < E_OK) return er;
	if (id != 0x48) {
		return E_IO;
	}

	/* Accelerometer has no ID register: read CTRL_REG1_A back */
	er = read_regs(LSM303_ACC_ADDR, CTRL_REG1_A, &id, 1);
	if (er < E_OK) return er;
	return (id == ACC_ODR_100HZ_XYZ) ? E_OK : E_IO;
}

EXPORT ER lsm303_read(LSM303_RAW *raw)
{
	UB	a[6];
	UB	m[6];
	ER	er;

	if (dd_iic < E_OK) {
		return E_OBJ;
	}

	if (d_addr != 0) {
		er = read_regs(d_addr, OUT_X_L_A | AUTO_INC, a, 6);
		if (er < E_OK) return er;
		er = read_regs(d_addr, D_OUT_X_L_M | AUTO_INC, m, 6);
		if (er < E_OK) return er;
		/* Accel: 16-bit little-endian, 0.061 mg/LSB at +-2 g */
		raw->ax_mg = (H)(((W)(H)(a[0] | (a[1] << 8)) * 61) / 1000);
		raw->ay_mg = (H)(((W)(H)(a[2] | (a[3] << 8)) * 61) / 1000);
		raw->az_mg = (H)(((W)(H)(a[4] | (a[5] << 8)) * 61) / 1000);
		/* Mag: little-endian X, Y, Z.  Only the direction is used
		   (heading), so the scale does not matter; /4 keeps it near
		   the DLHC's range. */
		raw->mx = (H)((H)(m[0] | (m[1] << 8)) / 4);
		raw->my = (H)((H)(m[2] | (m[3] << 8)) / 4);
		raw->mz = (H)((H)(m[4] | (m[5] << 8)) / 4);
		return E_OK;
	}

	er = read_regs(LSM303_ACC_ADDR, OUT_X_L_A | AUTO_INC, a, 6);
	if (er < E_OK) return er;
	er = read_regs(LSM303_MAG_ADDR, OUT_X_H_M, m, 6);
	if (er < E_OK) return er;

	/* Accel: 12-bit left-justified, little-endian.  >> 4 gives 1 mg/LSB
	   at +-2 g high resolution. */
	raw->ax_mg = (H)((H)(a[0] | (a[1] << 8)) >> 4);
	raw->ay_mg = (H)((H)(a[2] | (a[3] << 8)) >> 4);
	raw->az_mg = (H)((H)(a[4] | (a[5] << 8)) >> 4);

	/* Mag: big-endian, register order X, Z, Y */
	raw->mx = (H)((m[0] << 8) | m[1]);
	raw->mz = (H)((m[2] << 8) | m[3]);
	raw->my = (H)((m[4] << 8) | m[5]);
	return E_OK;
}

/* Bring-up aid: try every 7-bit address on I2C0 and print the ones that
   answer, with their WHO_AM_I (0x0F) byte.  LSM303DLHC: accel 0x19,
   mag 0x1E.  LSM303AGR: 0x19 (id 0x33) + 0x1E (id 0x40 at 0x4F).
   LSM303D/C: 0x1D. */
EXPORT INT lsm303_scan(void)
{
	UW	a;
	UB	id;
	INT	found = 0;

	if (dd_iic < E_OK) {
		dd_iic = tk_opn_dev((UB*)"iica", TD_UPDATE);
		if (dd_iic < E_OK) {
			tm_printf((UB*)"imu: scan: cannot open iica (%d)\n", dd_iic);
			return dd_iic;
		}
	}
	tm_printf((UB*)"imu: scan I2C0 (SDA GP0, SCL GP1)...\n");
	for (a = 0x08; a <= 0x77; a++) {
		id = 0;
		if (read_regs(a, 0x0F, &id, 1) >= E_OK) {
			tm_printf((UB*)"imu: scan: device at 0x%x, reg 0x0F = 0x%x\n",
				  (INT)a, (INT)id);
			found++;
		}
	}
	tm_printf((UB*)"imu: scan: %d device(s) found%s\n", found,
		  found == 0 ? " -- check wiring, power and pull-ups" : "");
	return found;
}
