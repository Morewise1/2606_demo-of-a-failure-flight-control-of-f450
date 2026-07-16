#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define HMC5883L_DEFAULT_DEV "/dev/i2c-2"
#define HMC5883L_DEFAULT_ADDR 0x1e

#define HMC5883L_REG_CONFIG_A 0x00
#define HMC5883L_REG_CONFIG_B 0x01
#define HMC5883L_REG_MODE     0x02
#define HMC5883L_REG_DATA     0x03
#define HMC5883L_REG_STATUS   0x09
#define HMC5883L_REG_ID_A     0x0A
#define HMC5883L_REG_ID_B     0x0B
#define HMC5883L_REG_ID_C     0x0C

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
    const char *dev_path = HMC5883L_DEFAULT_DEV;
    int i2c_addr = HMC5883L_DEFAULT_ADDR;

    if (argc >= 2) {
        dev_path = argv[1];
    }
    if (argc >= 3 && parse_i2c_addr(argv[2], &i2c_addr) != 0) {
        return 1;
    }
    if (argc > 3) {
        fprintf(stderr, "usage: %s [/dev/i2c-2] [0x1e]\n", argv[0]);
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

    uint8_t id[3] = {0};
    if (read_reg(fd, HMC5883L_REG_ID_A, &id[0]) != 0 ||
        read_reg(fd, HMC5883L_REG_ID_B, &id[1]) != 0 ||
        read_reg(fd, HMC5883L_REG_ID_C, &id[2]) != 0) {
        close(fd);
        return 1;
    }

    printf("ID: 0x%02X 0x%02X 0x%02X ('%c''%c''%c')\n",
           id[0], id[1], id[2], id[0], id[1], id[2]);
    if (id[0] != 0x48 || id[1] != 0x34 || id[2] != 0x33) {
        fprintf(stderr, "unexpected HMC5883L ID: expected 0x48 0x34 0x33 ('H''4''3')\n");
        close(fd);
        return 1;
    }

    if (write_reg(fd, HMC5883L_REG_CONFIG_A, 0x70) != 0 ||
        write_reg(fd, HMC5883L_REG_CONFIG_B, 0x20) != 0 ||
        write_reg(fd, HMC5883L_REG_MODE, 0x00) != 0) {
        close(fd);
        return 1;
    }

    printf("HMC5883L initialized on %s addr 0x%02X\n", dev_path, i2c_addr);
    printf("Press Ctrl+C to stop.\n\n");

    for (;;) {
        uint8_t status = 0;
        uint8_t data[6] = {0};

        if (read_reg(fd, HMC5883L_REG_STATUS, &status) != 0) {
            close(fd);
            return 1;
        }

        if (read_regs(fd, HMC5883L_REG_DATA, data, sizeof(data)) != 0) {
            close(fd);
            return 1;
        }

        int16_t x = be16_to_i16(&data[0]);
        int16_t z = be16_to_i16(&data[2]);
        int16_t y = be16_to_i16(&data[4]);

        printf("STATUS: 0x%02X DRDY=%u | MAG raw: x=%6d y=%6d z=%6d\n",
               status, status & 0x01, x, y, z);

        usleep(100000);
    }

    close(fd);
    return 0;
}
