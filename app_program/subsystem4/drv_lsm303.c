/*
 *      drv_lsm303.c
 *      Subsystem 4 -- LSM303DLHC / LSM303D driver over the kernel I2C
 *      driver.
 *
 *      Needs DEVCNF_USE_IIC = 1 in config/config_device.h and the I2C0 pins
 *      set in device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h (shared files:
 *      see the subsystem 4 README for the agreed change).
 *
 *      Register map: ST LSM303DLHC datasheet, DocID018771.
 */

#include "drv_lsm303.h" /* app.h -> tk/tkernel.h first */
#include <tk/device.h>  /* tk_opn_dev, T_I2C_EXEC, TDN_I2C_EXEC */

#if !DEVCNF_USE_IIC
#error \
    "Subsystem 4 needs the I2C driver: set DEVCNF_USE_IIC to 1 in config/config_device.h"
#endif

/* Accelerometer registers */
#define CTRL_REG1_A 0x20
#define CTRL_REG4_A 0x23
#define OUT_X_L_A   0x28
#define AUTO_INC    0x80 /* set MSB of sub-address to read a block */

/* Magnetometer registers */
#define CRA_REG_M 0x00
#define CRB_REG_M 0x01
#define MR_REG_M  0x02
#define OUT_X_H_M 0x03 /* order: X, Z, Y, big-endian */
#define IRA_REG_M 0x0A /* reads 'H' (0x48) */
#define IRA_ID    0x48

/* Settings */
#define ACC_ODR_100HZ_XYZ 0x57 /* 100 Hz, normal mode, X/Y/Z on */
#define ACC_FS2G_HR       0x08 /* +-2 g, high resolution, little-endian */
#define MAG_ODR_75HZ      0x18 /* 75 Hz output rate */
#define MAG_GAIN_1_3      0x20 /* +-1.3 gauss */
#define MAG_CONTINUOUS    0x00

static ID g_dd_iic = -1; /* device descriptor, < 0 = closed */

/* LSM303D (one address for accel + mag, 0x1D with SA0 high or open,
   0x1E with SA0 low).  Detected at init when the DLHC does not answer. */
#define D_WHO_AM_I      0x0F /* reads 0x49 */
#define D_ID            0x49
#define D_OUT_X_L_M     0x08
#define D_CTRL1         0x20
#define D_CTRL2         0x21
#define D_CTRL5         0x24
#define D_CTRL6         0x25
#define D_CTRL7         0x26
#define D_ACC_100HZ_XYZ 0x57 /* 100 Hz, X/Y/Z on, BDU off */
#define D_ACC_FS2G      0x00 /* +-2 g: 0.061 mg per LSB */
#define D_MAG_50HZ_HR   0x70 /* high resolution, 50 Hz */
#define D_MAG_FS4G      0x20 /* +-4 gauss */
#define D_MAG_CONT      0x00 /* continuous conversion */
#define D_ADDR_SA0_HIGH 0x1DU
#define D_ADDR_SA0_LOW  0x1EU
#define D_ADDR_COUNT    2
#define D_ACC_UG_LSB    61 /* 0.061 mg = 61 ug per LSB at +-2 g */
#define D_UG_PER_MG     1000
#define D_MAG_DIV       4 /* /4 keeps the mag near the DLHC's range */

#define SAMPLE_BYTES 6     /* X, Y, Z as 16-bit values */
#define SCAN_FIRST   0x08U /* 7-bit addresses that are not reserved */
#define SCAN_LAST    0x77U

static uint32_t g_d_addr; /* 0 = LSM303DLHC, else LSM303D address */

static ER write_reg(uint32_t sadr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2];
    SZ      asz;

    buf[0] = reg;
    buf[1] = val;
    /* For "iic" devices the start parameter is the slave address */
    /* Cast: 7-bit addresses fit easily in the signed start number. */
    return tk_swri_dev(g_dd_iic, (int32_t)sadr, buf, 2, &asz);
}

static ER read_regs(uint32_t sadr, uint8_t reg, uint8_t * p_buf, SZ len)
{
    T_I2C_EXEC exec;
    SZ         asz;

    exec.sadr     = sadr;
    exec.snd_size = 1;
    exec.snd_data = &reg;
    exec.rcv_size = len;
    exec.rcv_data = p_buf;
    /* Write the register address, repeated start, read len bytes */
    return tk_swri_dev(g_dd_iic, TDN_I2C_EXEC, &exec, sizeof(exec), &asz);
}

void lsm303_close(void)
{
    if (g_dd_iic >= E_OK)
    {
        tk_cls_dev(g_dd_iic, 0);
        g_dd_iic = -1;
    }
}

/* Little-endian and big-endian 16-bit two's complement values.
   Cast: the unsigned bit pattern is reinterpreted as int16_t (GCC
   keeps the bits, two's complement). */
static int16_t le16(const uint8_t * p_buf)
{
    return (int16_t)(uint16_t)((uint16_t)p_buf[0] |
                               (uint16_t)((uint16_t)p_buf[1] << 8));
}

static int16_t be16(const uint8_t * p_buf)
{
    return (int16_t)(uint16_t)((uint16_t)((uint16_t)p_buf[0] << 8) |
                               (uint16_t)p_buf[1]);
}

/* LSM303D: find it at 0x1D or 0x1E, then configure accel + mag */
static ER init_lsm303d(void)
{
    static const uint32_t cand[D_ADDR_COUNT] = { D_ADDR_SA0_HIGH,
                                                 D_ADDR_SA0_LOW };
    static const uint8_t  setup[][2]         = {
        { D_CTRL1, D_ACC_100HZ_XYZ }, { D_CTRL2, D_ACC_FS2G },
        { D_CTRL5, D_MAG_50HZ_HR },   { D_CTRL6, D_MAG_FS4G },
        { D_CTRL7, D_MAG_CONT },
    };
    uint8_t  who_am_i;
    uint32_t idx;
    ER       ercd = E_IO;

    g_d_addr = 0U;
    for (idx = 0U; (idx < D_ADDR_COUNT) && (0U == g_d_addr); idx++)
    {
        who_am_i = 0U;
        if ((read_regs(cand[idx], D_WHO_AM_I, &who_am_i, 1) >= E_OK) &&
            (D_ID == who_am_i))
        {
            g_d_addr = cand[idx];
        }
    }

    if (0U != g_d_addr)
    {
        /* Cast: the kernel API takes uint8_t strings and int32_t args. */
        tm_printf((uint8_t *)"imu: LSM303D found at 0x%x\n", (int32_t)g_d_addr);
        ercd = E_OK;
        for (idx = 0U;
             (idx < (sizeof(setup) / sizeof(setup[0]))) && (ercd >= E_OK);
             idx++)
        {
            ercd = write_reg(g_d_addr, setup[idx][0], setup[idx][1]);
        }
        if (ercd >= E_OK)
        {
            ercd = read_regs(g_d_addr, D_CTRL1, &who_am_i, 1);
        }
        if (ercd >= E_OK)
        {
            ercd = (D_ACC_100HZ_XYZ == who_am_i) ? E_OK : E_IO;
        }
    }
    return ercd;
}

/* LSM303DLHC: the accelerometer is already on (CTRL_REG1_A written) */
static ER init_dlhc(void)
{
    uint8_t who_am_i = 0U;
    ER      ercd;

    ercd = write_reg(LSM303_ACC_ADDR, CTRL_REG4_A, ACC_FS2G_HR);
    if (ercd >= E_OK)
    {
        ercd = write_reg(LSM303_MAG_ADDR, CRA_REG_M, MAG_ODR_75HZ);
    }
    if (ercd >= E_OK)
    {
        ercd = write_reg(LSM303_MAG_ADDR, CRB_REG_M, MAG_GAIN_1_3);
    }
    if (ercd >= E_OK)
    {
        ercd = write_reg(LSM303_MAG_ADDR, MR_REG_M, MAG_CONTINUOUS);
    }

    /* Identity check: wrong chip or wiring gives a clear error */
    if (ercd >= E_OK)
    {
        ercd = read_regs(LSM303_MAG_ADDR, IRA_REG_M, &who_am_i, 1);
    }
    if ((ercd >= E_OK) && (IRA_ID != who_am_i))
    {
        ercd = E_IO;
    }

    /* Accelerometer has no ID register: read CTRL_REG1_A back */
    if (ercd >= E_OK)
    {
        ercd = read_regs(LSM303_ACC_ADDR, CTRL_REG1_A, &who_am_i, 1);
    }
    if (ercd >= E_OK)
    {
        ercd = (ACC_ODR_100HZ_XYZ == who_am_i) ? E_OK : E_IO;
    }
    return ercd;
}

ER lsm303_init(void)
{
    ER ercd = E_OK;
    ER ercd_d;

    if (g_dd_iic < E_OK)
    {
        /* Cast: the kernel API takes device names as uint8_t strings. */
        g_dd_iic = tk_opn_dev((uint8_t *)"iica", TD_UPDATE);
        if (g_dd_iic < E_OK)
        {
            ercd = g_dd_iic;
        }
    }

    if (ercd >= E_OK)
    {
        g_d_addr = 0U;
        ercd     = write_reg(LSM303_ACC_ADDR, CTRL_REG1_A, ACC_ODR_100HZ_XYZ);
        if (ercd < E_OK)
        {
            /* No LSM303DLHC accelerometer: try an LSM303D.  If none is
               found either, report the DLHC error. */
            ercd_d = init_lsm303d();
            ercd   = (0U == g_d_addr) ? ercd : ercd_d;
        }
        else
        {
            ercd = init_dlhc();
        }
    }
    return ercd;
}

/* LSM303D sample: accel 16-bit LE at 0.061 mg/LSB, mag LE X, Y, Z */
static void convert_lsm303d(const uint8_t * p_acc, const uint8_t * p_mag,
                            lsm303_raw_t * p_raw)
{
    /* Cast: the result is within +-2000 mg, so it fits int16_t. */
    p_raw->ax_mg =
        (int16_t)(((int32_t)le16(&p_acc[0]) * D_ACC_UG_LSB) / D_UG_PER_MG);
    p_raw->ay_mg =
        (int16_t)(((int32_t)le16(&p_acc[2]) * D_ACC_UG_LSB) / D_UG_PER_MG);
    p_raw->az_mg =
        (int16_t)(((int32_t)le16(&p_acc[4]) * D_ACC_UG_LSB) / D_UG_PER_MG);
    /* Mag: only the direction is used (heading), so the scale does not
       matter.  Cast: dividing an int16_t by 4 still fits int16_t. */
    p_raw->mx = (int16_t)(le16(&p_mag[0]) / D_MAG_DIV);
    p_raw->my = (int16_t)(le16(&p_mag[2]) / D_MAG_DIV);
    p_raw->mz = (int16_t)(le16(&p_mag[4]) / D_MAG_DIV);
}

/* LSM303DLHC sample: accel 12-bit left-justified LE, mag BE X, Z, Y */
static void convert_dlhc(const uint8_t * p_acc, const uint8_t * p_mag,
                         lsm303_raw_t * p_raw)
{
    /* >> 4 gives 1 mg/LSB at +-2 g high resolution.  Deviation from
       BARR-C 5.3: an arithmetic right shift of a signed value keeps
       the sign of the left-justified reading (GCC behaviour). */
    p_raw->ax_mg = (int16_t)(le16(&p_acc[0]) >> 4);
    p_raw->ay_mg = (int16_t)(le16(&p_acc[2]) >> 4);
    p_raw->az_mg = (int16_t)(le16(&p_acc[4]) >> 4);

    p_raw->mx = be16(&p_mag[0]);
    p_raw->mz = be16(&p_mag[2]);
    p_raw->my = be16(&p_mag[4]);
}

ER lsm303_read(lsm303_raw_t * p_raw)
{
    uint8_t acc_buf[SAMPLE_BYTES];
    uint8_t mag_buf[SAMPLE_BYTES];
    ER      ercd;

    if (g_dd_iic < E_OK)
    {
        ercd = E_OBJ;
    }
    else if (0U != g_d_addr)
    {
        ercd = read_regs(g_d_addr, OUT_X_L_A | AUTO_INC, acc_buf, SAMPLE_BYTES);
        if (ercd >= E_OK)
        {
            ercd = read_regs(g_d_addr, D_OUT_X_L_M | AUTO_INC, mag_buf,
                             SAMPLE_BYTES);
        }
        if (ercd >= E_OK)
        {
            convert_lsm303d(acc_buf, mag_buf, p_raw);
            ercd = E_OK;
        }
    }
    else
    {
        ercd = read_regs(LSM303_ACC_ADDR, OUT_X_L_A | AUTO_INC, acc_buf,
                         SAMPLE_BYTES);
        if (ercd >= E_OK)
        {
            ercd = read_regs(LSM303_MAG_ADDR, OUT_X_H_M, mag_buf, SAMPLE_BYTES);
        }
        if (ercd >= E_OK)
        {
            convert_dlhc(acc_buf, mag_buf, p_raw);
            ercd = E_OK;
        }
    }
    return ercd;
}

/* Bring-up aid: try every 7-bit address on I2C0 and print the ones that
   answer, with their WHO_AM_I (0x0F) byte.  LSM303DLHC: accel 0x19,
   mag 0x1E.  LSM303AGR: 0x19 (id 0x33) + 0x1E (id 0x40 at 0x4F).
   LSM303D/C: 0x1D. */
int32_t lsm303_scan(void)
{
    uint32_t addr;
    uint8_t  who_am_i;
    int32_t  found = 0;

    if (g_dd_iic < E_OK)
    {
        /* Cast: the kernel API takes uint8_t strings (also below). */
        g_dd_iic = tk_opn_dev((uint8_t *)"iica", TD_UPDATE);
        if (g_dd_iic < E_OK)
        {
            tm_printf((uint8_t *)"imu: scan: cannot open iica (%d)\n",
                      g_dd_iic);
            return g_dd_iic;
        }
    }
    tm_printf((uint8_t *)"imu: scan I2C0 (SDA GP0, SCL GP1)...\n");
    for (addr = SCAN_FIRST; addr <= SCAN_LAST; addr++)
    {
        who_am_i = 0U;
        if (read_regs(addr, D_WHO_AM_I, &who_am_i, 1) >= E_OK)
        {
            tm_printf((uint8_t *)"imu: scan: device at 0x%x, reg 0x0F = 0x%x\n",
                      (int32_t)addr, (int32_t)who_am_i);
            found++;
        }
    }
    tm_printf((uint8_t *)"imu: scan: %d device(s) found%s\n", found,
              (0 == found) ? " -- check wiring, power and pull-ups" : "");
    return found;
}

/*** end of file ***/
