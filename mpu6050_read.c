#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define MPU6050_DEFAULT_DEV "/dev/i2c-2"
#define MPU6050_DEFAULT_ADDR 0x68

#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75

static int write_reg(int fd, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    ssize_t written = write(fd, buf, sizeof(buf));
    if (written != (ssize_t)sizeof(buf)) {
        fprintf(stderr, "write reg 0x%02X failed: %s\n", reg, written < 0 ? strerror(errno) : "short write");
        return -1;
    }
    return 0;
}

static int read_regs(int fd, uint8_t reg, uint8_t *buf, size_t len)
{
    ssize_t written = write(fd, &reg, 1);
    if (written != 1) {
        fprintf(stderr, "select reg 0x%02X failed: %s\n", reg, written < 0 ? strerror(errno) : "short write");
        return -1;
    }

    ssize_t read_len = read(fd, buf, len);
    if (read_len != (ssize_t)len) {
        fprintf(stderr, "read reg 0x%02X len %zu failed: %s\n", reg, len, read_len < 0 ? strerror(errno) : "short read");
        return -1;
    }
    return 0;
}

static int read_reg(int fd, uint8_t reg, uint8_t *value)
{
    return read_regs(fd, reg, value, 1);
}

static int16_t be16_to_i16(const uint8_t *buf)
{
    return (int16_t)((uint16_t)buf[0] << 8 | (uint16_t)buf[1]);
}

static int parse_i2c_addr(const char *text, int *addr)
{
    char *end = NULL;
    errno = 0;
    unsigned long value = strtoul(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' || value > 0x7F) {
        fprintf(stderr, "invalid I2C address: %s\n", text);
        return -1;
    }
    *addr = (int)value;
    return 0;
}

int main(int argc, char **argv)
{
    const char *dev_path = MPU6050_DEFAULT_DEV;
    int i2c_addr = MPU6050_DEFAULT_ADDR;

    if (argc >= 2) {
        dev_path = argv[1];
    }
    if (argc >= 3 && parse_i2c_addr(argv[2], &i2c_addr) != 0) {
        return 1;
    }
    if (argc > 3) {
        fprintf(stderr, "usage: %s [/dev/i2c-2] [0x68]\n", argv[0]);
        return 1;
    }

    int fd = open(dev_path, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "open %s failed: %s\n", dev_path, strerror(errno));
        return 1;
    }

    if (ioctl(fd, I2C_SLAVE, i2c_addr) < 0) {
        fprintf(stderr, "ioctl I2C_SLAVE 0x%02X failed: %s\n", i2c_addr, strerror(errno));
        close(fd);
        return 1;
    }

    uint8_t who_am_i = 0;
    if (read_reg(fd, MPU6050_REG_WHO_AM_I, &who_am_i) != 0) {
        close(fd);
        return 1;
    }
    printf("WHO_AM_I = 0x%02X\n", who_am_i);
    if (who_am_i != 0x68) {
        fprintf(stderr, "unexpected WHO_AM_I: expected 0x68, got 0x%02X\n", who_am_i);
        close(fd);
        return 1;
    }

    if (write_reg(fd, MPU6050_REG_PWR_MGMT_1, 0x00) != 0) {
        close(fd);
        return 1;
    }
    usleep(100000);

    if (write_reg(fd, MPU6050_REG_SMPLRT_DIV, 0x07) != 0 ||
        write_reg(fd, MPU6050_REG_CONFIG, 0x03) != 0 ||
        write_reg(fd, MPU6050_REG_GYRO_CONFIG, 0x00) != 0 ||
        write_reg(fd, MPU6050_REG_ACCEL_CONFIG, 0x00) != 0) {
        close(fd);
        return 1;
    }

    printf("MPU6050 initialized on %s addr 0x%02X\n", dev_path, i2c_addr);
    printf("Press Ctrl+C to stop.\n\n");

    for (;;) {
        uint8_t raw[14];
        if (read_regs(fd, MPU6050_REG_ACCEL_XOUT_H, raw, sizeof(raw)) != 0) {
            close(fd);
            return 1;
        }

        int16_t ax = be16_to_i16(&raw[0]);
        int16_t ay = be16_to_i16(&raw[2]);
        int16_t az = be16_to_i16(&raw[4]);
        int16_t temp = be16_to_i16(&raw[6]);
        int16_t gx = be16_to_i16(&raw[8]);
        int16_t gy = be16_to_i16(&raw[10]);
        int16_t gz = be16_to_i16(&raw[12]);

        printf("ACC raw: %6d %6d %6d | g: %+7.3f %+7.3f %+7.3f | "
               "TEMP raw: %6d | C: %6.2f | "
               "GYRO raw: %6d %6d %6d | dps: %+8.3f %+8.3f %+8.3f\n",
               ax, ay, az,
               ax / 16384.0, ay / 16384.0, az / 16384.0,
               temp, temp / 340.0 + 36.53,
               gx, gy, gz,
               gx / 131.0, gy / 131.0, gz / 131.0);

        usleep(100000);
    }

    close(fd);
    return 0;
}
