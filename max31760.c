// SPDX-License-Identifier: GPL-2.0-only

/*
 * Analog Devices MAX31760 Fan Speed Controller
 *
 * Copyright (C) 2023 Ibrahim Tilki <Ibrahim.Tilki@analog.com>
 *
 * This driver has been ported to Linux kernel v3.18.24
 * Original driver from v6.14.
 * Porting Author: hs.choi hs.choi@piolink.com
 * Date: 2025-07-07
 *
 * Major changes for porting:
 *  - Replaced hwmon_ops with individual sysfs show/store functions.
 *  - Replaced devm_hwmon_device_register_with_info with
 *    devm_hwmon_device_register_with_groups.
 *  - Adapted to v3.18.24 regmap API (e.g., no regmap_set/clear_bits).
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/pm.h>
#include <linux/of.h>


#define REG_CR1                 0x00
#define CR1_HYST                BIT(5)
#define CR1_DRV                 (BIT(4) | BIT(3))
#define CR1_TEMP_SRC            (BIT(1) | BIT(0))
#define REG_CR2                 0x01
#define CR2_STBY                BIT(7)
#define CR2_ALERTS              BIT(6)
#define CR2_DFC                 BIT(0)
#define REG_CR3                 0x02
#define REG_PWMR                0x50
#define REG_PWMV                0x51
#define REG_STATUS              0x5A
#define STATUS_ALARM_CRIT(ch)   BIT(2 + 2 * (ch))
#define STATUS_ALARM_MAX(ch)    BIT(3 + 2 * (ch))
#define STATUS_RDFA             BIT(6)

#define REG_TACH(ch)            (0x52 + (ch) * 2)
#define REG_TEMP_INPUT(ch)      (0x56 + (ch) * 2)
#define REG_TEMP_MAX(ch)        (0x06 + (ch) * 2)
#define REG_TEMP_CRIT(ch)       (0x0A + (ch) * 2)

#define TEMP11_FROM_REG(reg)    ((reg) / 32 * 125)
#define TEMP11_TO_REG(val)      (DIV_ROUND_CLOSEST(clamp_val((val), -128000, 127875), 125) * 32)

#define LUT_SIZE                48

#define REG_LUT(index)          (0x20 + (index))

struct max31760_state {
    struct regmap *regmap;

    struct lut_attribute {
        char name[24];
        struct sensor_device_attribute sda;
    } lut[LUT_SIZE];

    struct attribute *attrs[LUT_SIZE + 2];
    struct attribute_group group;
    const struct attribute_group *groups[2];
};

static bool max31760_volatile_reg(struct device *dev, unsigned int reg)
{
    return reg > 0x50;
}

static const struct regmap_config regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,
    .max_register = 0x5B,
    .cache_type = REGCACHE_RBTREE,
    .volatile_reg = max31760_volatile_reg,
};

static int tach_to_rpm(u16 tach)
{
    if (tach == 0)
        tach = 1;

    return 60 * 100000 / tach / 2;
}

/* temp attr */
static ssize_t temp_input_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct sensor_device_attribute *sattr = to_sensor_dev_attr(attr);
    struct max31760_state *state = dev_get_drvdata(dev);
    s16 temp;
    u8 reg[2];
    int ret;

    ret = regmap_bulk_read(state->regmap, REG_TEMP_INPUT(sattr->index), reg, 2);
    if (ret)
        return ret;

    temp = (reg[0] << 8) | reg[1];
    return sprintf(buf, "%d\n", TEMP11_FROM_REG(temp));
}

static ssize_t temp_max_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct sensor_device_attribute *sattr = to_sensor_dev_attr(attr);
    struct max31760_state *state = dev_get_drvdata(dev);
    s16 temp;
    u8 reg[2];
    int ret;

    ret = regmap_bulk_read(state->regmap, REG_TEMP_MAX(sattr->index), reg, 2);
    if (ret)
        return ret;

    temp = (reg[0] << 8) | reg[1];
    return sprintf(buf, "%d\n", TEMP11_FROM_REG(temp));
}

static ssize_t temp_max_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct sensor_device_attribute *sattr = to_sensor_dev_attr(attr);
    struct max31760_state *state = dev_get_drvdata(dev);
    long val;
    int temp;
    u8 reg_val[2];
    int ret;

    ret = kstrtol(buf, 10, &val);
    if (ret)
        return ret;

    temp = TEMP11_TO_REG(val);
    reg_val[0] = temp >> 8;
    reg_val[1] = temp & 0xFF;

    ret = regmap_bulk_write(state->regmap, REG_TEMP_MAX(sattr->index), reg_val, 2);
    return ret ? ret : count;
}

static ssize_t temp_crit_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct sensor_device_attribute *sattr = to_sensor_dev_attr(attr);
    struct max31760_state *state = dev_get_drvdata(dev);
    s16 temp;
    u8 reg[2];
    int ret;

    ret = regmap_bulk_read(state->regmap, REG_TEMP_CRIT(sattr->index), reg, 2);
    if (ret)
        return ret;

    temp = (reg[0] << 8) | reg[1];
    return sprintf(buf, "%d\n", TEMP11_FROM_REG(temp));
}

static ssize_t temp_crit_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct sensor_device_attribute *sattr = to_sensor_dev_attr(attr);
    struct max31760_state *state = dev_get_drvdata(dev);
    long val;
    int temp;
    u8 reg_val[2];
    int ret;

    ret = kstrtol(buf, 10, &val);
    if (ret)
        return ret;

    temp = TEMP11_TO_REG(val);
    reg_val[0] = temp >> 8;
    reg_val[1] = temp & 0xFF;

    ret = regmap_bulk_write(state->regmap, REG_TEMP_CRIT(sattr->index), reg_val, 2);
    return ret ? ret : count;
}

static ssize_t temp_max_alarm_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct sensor_device_attribute *sattr = to_sensor_dev_attr(attr);
    struct max31760_state *state = dev_get_drvdata(dev);
    unsigned int regval;
    int ret;

    ret = regmap_read(state->regmap, REG_STATUS, &regval);
    if (ret)
        return ret;

    return sprintf(buf, "%d\n", (regval & STATUS_ALARM_MAX(sattr->index)) ? 1 : 0);
}

static ssize_t temp_crit_alarm_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct sensor_device_attribute *sattr = to_sensor_dev_attr(attr);
    struct max31760_state *state = dev_get_drvdata(dev);
    unsigned int regval;
    int ret;

    ret = regmap_read(state->regmap, REG_STATUS, &regval);
    if (ret)
        return ret;

    return sprintf(buf, "%d\n", (regval & STATUS_ALARM_CRIT(sattr->index)) ? 1 : 0);
}

static ssize_t temp_fault_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct max31760_state *state = dev_get_drvdata(dev);
    unsigned int regval;
    int ret;

    ret = regmap_read(state->regmap, REG_STATUS, &regval);
    if (ret)
        return ret;

    return sprintf(buf, "%d\n", (regval & STATUS_RDFA) ? 1 : 0);
}

static ssize_t temp_label_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct sensor_device_attribute *sattr = to_sensor_dev_attr(attr);
    return sprintf(buf, "%s\n", sattr->index ? "local" : "remote");
}

/* fan attr */
static ssize_t fan_input_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct sensor_device_attribute *sattr = to_sensor_dev_attr(attr);
    struct max31760_state *state = dev_get_drvdata(dev);
    u16 tach_val;
    u8 reg[2];
    int ret;

    ret = regmap_bulk_read(state->regmap, REG_TACH(sattr->index), reg, 2);
    if (ret)
        return ret;

    tach_val = (reg[0] << 8) | reg[1];
    return sprintf(buf, "%d\n", tach_to_rpm(tach_val));
}

static ssize_t fan_fault_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct sensor_device_attribute *sattr = to_sensor_dev_attr(attr);
    struct max31760_state *state = dev_get_drvdata(dev);
    unsigned int regval;
    int ret;

    ret = regmap_read(state->regmap, REG_STATUS, &regval);
    if (ret)
        return ret;

    return sprintf(buf, "%d\n", (regval & BIT(sattr->index)) ? 1 : 0);
}

static ssize_t fan_enable_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct sensor_device_attribute *sattr = to_sensor_dev_attr(attr);
    struct max31760_state *state = dev_get_drvdata(dev);
    unsigned int regval;
    int ret;

    ret = regmap_read(state->regmap, REG_CR3, &regval);
    if (ret)
        return ret;

    return sprintf(buf, "%d\n", (regval & BIT(sattr->index)) ? 1 : 0);
}

static ssize_t fan_enable_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct sensor_device_attribute *sattr = to_sensor_dev_attr(attr);
    struct max31760_state *state = dev_get_drvdata(dev);
    unsigned long val;
    int ret;

    ret = kstrtoul(buf, 10, &val);
    if (ret)
        return ret;

    if (val > 1)
        return -EINVAL;

    if (val)
        ret = regmap_update_bits(state->regmap, REG_CR3, BIT(sattr->index), BIT(sattr->index));
    else
        ret = regmap_update_bits(state->regmap, REG_CR3, BIT(sattr->index), 0);

    return ret ? ret : count;
}

/* pwm attr */
static ssize_t pwm1_input_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct max31760_state *state = dev_get_drvdata(dev);
    unsigned int regval;
    int ret;

    ret = regmap_read(state->regmap, REG_PWMV, &regval);
    if (ret)
        return ret;

    return sprintf(buf, "%u\n", regval);
}

static ssize_t pwm1_input_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct max31760_state *state = dev_get_drvdata(dev);
    unsigned long val;
    int ret;

    ret = kstrtoul(buf, 10, &val);
    if (ret)
        return ret;

    if (val > 255)
        return -EINVAL;

    ret = regmap_write(state->regmap, REG_PWMR, val);

    return ret ? ret : count;
}

static const int max31760_pwm_freq[] = {33, 150, 1500, 25000};

static ssize_t pwm1_freq_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct max31760_state *state = dev_get_drvdata(dev);
    unsigned int regval;
    int ret;

    ret = regmap_read(state->regmap, REG_CR1, &regval);
    if (ret)
        return ret;

    regval = (regval & CR1_DRV) >> __ffs(CR1_DRV);

    if (regval >= ARRAY_SIZE(max31760_pwm_freq))
        return -ENXIO;

    return sprintf(buf, "%d\n", max31760_pwm_freq[regval]);
}

static ssize_t pwm1_freq_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct max31760_state *state = dev_get_drvdata(dev);
    unsigned long val;
    unsigned int pwm_index;
    int i, ret;

    ret = kstrtoul(buf, 10, &val);
    if (ret)
        return ret;

    pwm_index = 0;

    for (i = 1; i < ARRAY_SIZE(max31760_pwm_freq); i++) {
        if (abs(val - max31760_pwm_freq[i]) < abs(val - max31760_pwm_freq[pwm_index])) {
            pwm_index = i;
        }
    }

    ret = regmap_update_bits(state->regmap, REG_CR1, CR1_DRV,
                (pwm_index << __ffs(CR1_DRV)) & CR1_DRV);

    return ret ? ret : count;
}

static ssize_t pwm1_enable_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct max31760_state *state = dev_get_drvdata(dev);
    unsigned int regval;
    long val;
    int ret;

    ret = regmap_read(state->regmap, REG_CR2, &regval);
    if (ret)
        return ret;

    val = 2 - ((regval & CR2_DFC) ? 1 : 0);

    return sprintf(buf, "%ld\n", val);
}

static ssize_t pwm1_enable_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct max31760_state *state = dev_get_drvdata(dev);
    unsigned long val;
    int ret;

    ret = kstrtoul(buf, 10, &val);
    if (ret)
        return ret;

    if (val == 1) { 
        ret = regmap_update_bits(state->regmap, REG_CR2, CR2_DFC, CR2_DFC);
    } else if (val == 2) {
        ret = regmap_update_bits(state->regmap, REG_CR2,
                CR2_DFC, 0);
    } else {
        return -EINVAL;
    }

    return ret ? ret : count;
}

static ssize_t pwm1_auto_channels_temp_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct max31760_state *state = dev_get_drvdata(dev);
    unsigned int regval;
    long val;
    int ret;

    ret = regmap_read(state->regmap, REG_CR1, &regval);
    if (ret)
        return ret;

    regval = (regval & CR1_TEMP_SRC) >> __ffs(CR1_TEMP_SRC);

    switch (regval) {
        case 0: // remote
            val = 2;
            break;
        case 1: // local
            val = 1;
            break;
        case 2:
        case 3: // max of
            val = 3;
            break;
        default:
            return -ENXIO;
    }

    return sprintf(buf, "%ld\n", val);
}

static ssize_t pwm1_auto_channels_temp_store(struct device *dev,
        struct device_attribute *attr,
        const char
        *buf, size_t
        count)
{
    struct max31760_state *state = dev_get_drvdata(dev);
    unsigned long val;
    unsigned int reg_val;
    int ret;

    ret = kstrtoul(buf, 10, &val);
    if (ret)
        return ret;

    switch (val) {
        case 1: // local -> 1
            reg_val = 1;
            break;
        case 2: // remote -> 0
            reg_val = 0;
            break;
        case 3: // max or avg -> 2
            reg_val = 2;
            break;
        default:
            return
                -EINVAL;
    }

    ret = regmap_update_bits(state->regmap, REG_CR1, CR1_TEMP_SRC, reg_val);

    return ret ? ret : count;
}

static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, temp_input_show, NULL, 0);
static SENSOR_DEVICE_ATTR(temp2_input, S_IRUGO, temp_input_show, NULL, 1);
static SENSOR_DEVICE_ATTR(temp1_max, S_IRUGO | S_IWUSR, temp_max_show, temp_max_store, 0);
static SENSOR_DEVICE_ATTR(temp2_max, S_IRUGO | S_IWUSR, temp_max_show, temp_max_store, 1);
static SENSOR_DEVICE_ATTR(temp1_crit, S_IRUGO | S_IWUSR, temp_crit_show, temp_crit_store, 0);
static SENSOR_DEVICE_ATTR(temp2_crit, S_IRUGO | S_IWUSR, temp_crit_show, temp_crit_store, 1);
static SENSOR_DEVICE_ATTR(temp1_max_alarm, S_IRUGO, temp_max_alarm_show, NULL, 0);
static SENSOR_DEVICE_ATTR(temp2_max_alarm, S_IRUGO, temp_max_alarm_show, NULL, 1);
static SENSOR_DEVICE_ATTR(temp1_crit_alarm, S_IRUGO, temp_crit_alarm_show, NULL, 0);
static SENSOR_DEVICE_ATTR(temp2_crit_alarm, S_IRUGO, temp_crit_alarm_show, NULL, 1);
static SENSOR_DEVICE_ATTR(temp1_fault, S_IRUGO, temp_fault_show, NULL, 0);
static SENSOR_DEVICE_ATTR(temp2_fault, S_IRUGO, temp_fault_show, NULL, 1);
static SENSOR_DEVICE_ATTR(temp1_label, S_IRUGO, temp_label_show, NULL, 0);
static SENSOR_DEVICE_ATTR(temp2_label, S_IRUGO, temp_label_show, NULL, 1);

static SENSOR_DEVICE_ATTR(fan1_input, S_IRUGO, fan_input_show, NULL, 0);
static SENSOR_DEVICE_ATTR(fan2_input, S_IRUGO, fan_input_show, NULL, 1);
static SENSOR_DEVICE_ATTR(fan1_fault, S_IRUGO, fan_fault_show, NULL, 0);
static SENSOR_DEVICE_ATTR(fan2_fault, S_IRUGO, fan_fault_show, NULL, 1);
static SENSOR_DEVICE_ATTR(fan1_enable, S_IRUGO | S_IWUSR, fan_enable_show, fan_enable_store, 0);
static SENSOR_DEVICE_ATTR(fan2_enable, S_IRUGO | S_IWUSR, fan_enable_show, fan_enable_store, 1);

static SENSOR_DEVICE_ATTR(pwm1_enable, S_IRUGO | S_IWUSR, pwm1_enable_show, pwm1_enable_store, 0);
static SENSOR_DEVICE_ATTR(pwm1_input, S_IRUGO | S_IWUSR, pwm1_input_show, pwm1_input_store, 0);
static SENSOR_DEVICE_ATTR(pwm1_freq, S_IRUGO | S_IWUSR, pwm1_freq_show, pwm1_freq_store, 0);
static SENSOR_DEVICE_ATTR(pwm1_auto_channels_temp, S_IRUGO | S_IWUSR, pwm1_auto_channels_temp_show, pwm1_auto_channels_temp_store, 0);

static struct attribute *max31760_hwmon_attrs[] = {
    /* temp attr */
    &sensor_dev_attr_temp1_input.dev_attr.attr,
    &sensor_dev_attr_temp2_input.dev_attr.attr,
    &sensor_dev_attr_temp1_max.dev_attr.attr,
    &sensor_dev_attr_temp2_max.dev_attr.attr,
    &sensor_dev_attr_temp1_crit.dev_attr.attr,
    &sensor_dev_attr_temp2_crit.dev_attr.attr,
    &sensor_dev_attr_temp1_max_alarm.dev_attr.attr,
    &sensor_dev_attr_temp2_max_alarm.dev_attr.attr,
    &sensor_dev_attr_temp1_crit_alarm.dev_attr.attr,
    &sensor_dev_attr_temp2_crit_alarm.dev_attr.attr,
    &sensor_dev_attr_temp1_label.dev_attr.attr,
    &sensor_dev_attr_temp2_label.dev_attr.attr,
    &sensor_dev_attr_temp1_fault.dev_attr.attr,
    &sensor_dev_attr_temp2_fault.dev_attr.attr,
    /* fan attr */
    &sensor_dev_attr_fan1_input.dev_attr.attr,
    &sensor_dev_attr_fan2_input.dev_attr.attr,
    &sensor_dev_attr_fan1_fault.dev_attr.attr,
    &sensor_dev_attr_fan2_fault.dev_attr.attr,
    &sensor_dev_attr_fan1_enable.dev_attr.attr,
    &sensor_dev_attr_fan2_enable.dev_attr.attr,
    /* pwm attr */
    &sensor_dev_attr_pwm1_enable.dev_attr.attr,
    &sensor_dev_attr_pwm1_input.dev_attr.attr,
    &sensor_dev_attr_pwm1_freq.dev_attr.attr,
    &sensor_dev_attr_pwm1_auto_channels_temp.dev_attr.attr,
    NULL
};

static const struct attribute_group max31760_hwmon_group = {
    .attrs = max31760_hwmon_attrs,
};

static ssize_t pwm1_auto_point_temp_hyst_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct max31760_state *state = dev_get_drvdata(dev);
    unsigned int regval;
    int hyst_val;
    int ret;

    ret = regmap_read(state->regmap, REG_CR1, &regval);
    if (ret)
        return ret;

    hyst_val = (1 + ((regval & CR1_HYST) ? 1 : 0)) * 2000;

    return sprintf(buf, "%d\n", hyst_val);
}

static ssize_t pwm1_auto_point_temp_hyst_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    struct max31760_state *state = dev_get_drvdata(dev);
    unsigned long hyst;
    int ret;

    ret = kstrtoul(buf, 10, &hyst);
    if (ret)
        return ret;

    if (hyst < 3000) {
        ret = regmap_update_bits(state->regmap, REG_CR1, CR1_HYST, 0);
    } else {
        ret = regmap_update_bits(state->regmap, REG_CR1, CR1_HYST, CR1_HYST);
    }

    if (ret)
        return ret;

    return count;
}

static DEVICE_ATTR_RW(pwm1_auto_point_temp_hyst);

static ssize_t lut_show(struct device *dev,
        struct device_attribute *devattr, char *buf)
{
    struct sensor_device_attribute *sda = to_sensor_dev_attr(devattr);
    struct max31760_state *state = dev_get_drvdata(dev);
    int ret;
    unsigned int regval;

    ret = regmap_read(state->regmap, REG_LUT(sda->index), &regval);
    if (ret)
        return ret;

    return sprintf(buf, "%d\n", regval);
}

static ssize_t lut_store(struct device *dev,
        struct device_attribute *devattr,
        const char *buf, size_t count)
{
    struct sensor_device_attribute *sda = to_sensor_dev_attr(devattr);
    struct max31760_state *state = dev_get_drvdata(dev);
    unsigned long val;
    int ret;

    ret = kstrtoul(buf, 10, &val);
    if (ret)
        return ret;

    if (val > 255)
        return -EINVAL;

    ret = regmap_write(state->regmap, REG_LUT(sda->index), val);
    if (ret)
        return ret;

    return count;
}

static void max31760_create_lut_nodes(struct max31760_state *state)
{
    int i;
    struct sensor_device_attribute *sda;
    struct lut_attribute *lut;

    for (i = 0; i < LUT_SIZE; ++i) {
        lut = &state->lut[i];
        sda = &lut->sda;

        snprintf(lut->name, sizeof(lut->name),
                "pwm1_auto_point%d_pwm", i + 1);

        sda->dev_attr.attr.mode = 0644;
        sda->index = i;
        sda->dev_attr.show = lut_show;
        sda->dev_attr.store = lut_store;
        sda->dev_attr.attr.name = lut->name;

        sysfs_attr_init(&sda->dev_attr.attr);

        state->attrs[i] = &sda->dev_attr.attr;
    }

    state->attrs[i] = &dev_attr_pwm1_auto_point_temp_hyst.attr;

    state->group.attrs = state->attrs;
    state->groups[0] = &state->group;
}

static const struct of_device_id max31760_of_match[] = {
    {.compatible = "adi,max31760"},
    { }
};
MODULE_DEVICE_TABLE(of, max31760_of_match);

static int max31760_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    struct max31760_state *state;
    struct device *hwmon_dev;
    int ret;

    state = devm_kzalloc(dev, sizeof(*state), GFP_KERNEL);
    if (!state)
        return -ENOMEM;

    state->regmap = devm_regmap_init_i2c(client, &regmap_config);
    if (IS_ERR(state->regmap)) {
        dev_err(dev, "regmap initialization failed\n");
        return PTR_ERR(state->regmap);
    }

    i2c_set_clientdata(client, state);

    /* Set alert output to comparator mode */
    ret = regmap_update_bits(state->regmap, REG_CR2, CR2_ALERTS, CR2_ALERTS);
    if (ret) {
        dev_err(dev, "cannot write register\n");
        return ret;
    }

    max31760_create_lut_nodes(state);

    state->groups[1] = &max31760_hwmon_group;

    hwmon_dev = devm_hwmon_device_register_with_groups(dev, client->name,
                                                       state,
                                                       state->groups);

    if (IS_ERR(hwmon_dev)) {
        dev_err(dev, "unable to register hwmon device\n");
        return PTR_ERR(hwmon_dev);
    }

    return 0;
}

static const struct i2c_device_id max31760_id[] = {
    {"max31760"},
    { }
};
MODULE_DEVICE_TABLE(i2c, max31760_id);

static int max31760_suspend(struct device *dev)
{
    struct max31760_state *state = dev_get_drvdata(dev);

    return regmap_update_bits(state->regmap, REG_CR2, CR2_STBY, CR2_STBY);
}

static int max31760_resume(struct device *dev)
{
    struct max31760_state *state = dev_get_drvdata(dev);

    return regmap_update_bits(state->regmap, REG_CR2, CR2_STBY, 0);
}

static const struct dev_pm_ops max31760_pm_ops = {
    .suspend    = max31760_suspend,
    .resume     = max31760_resume,
};

static int max31760_remove(struct i2c_client *client)
{
    return 0;
}

static struct i2c_driver max31760_driver = {
    .class = I2C_CLASS_HWMON,
    .driver = {
        .name   = "max31760",
        .of_match_table = max31760_of_match,
        .pm = &max31760_pm_ops,
    },
    .probe        = max31760_probe,
    .remove       = max31760_remove,
    .id_table     = max31760_id,
};
module_i2c_driver(max31760_driver);

MODULE_AUTHOR("Ibrahim Tilki <Ibrahim.Tilki@analog.com>");
MODULE_AUTHOR("hs.choi <hs.choi@piolink.com> (v3.18.24 port)");
MODULE_DESCRIPTION("Analog Devices MAX31760 Fan Speed Controller (ported to v3.18.24)");
MODULE_LICENSE("GPL");

