// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 NXP
 * This P3H2x4x driver file implements functions for Hub probe and DT parsing.
 */

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/mfd/p3h2840.h>

#include "p3h2840_i3c_hub.h"

/* LDO voltage DT settings */
#define P3H2x4x_DT_LDO_VOLT_1_0V		1000
#define P3H2x4x_DT_LDO_VOLT_1_1V		1100
#define P3H2x4x_DT_LDO_VOLT_1_2V		1200
#define P3H2x4x_DT_LDO_VOLT_1_8V		1800

/* target port pull-up settings */
#define P3H2x4x_DT_TP_PULLUP_250R		250
#define P3H2x4x_DT_TP_PULLUP_500R		500
#define P3H2x4x_DT_TP_PULLUP_1000R		1000
#define P3H2x4x_DT_TP_PULLUP_2000R		2000

/*  IO strenght settings */
#define P3H2x4x_DT_IO_STRENGTH_20_OHM		20
#define P3H2x4x_DT_IO_STRENGTH_30_OHM		30
#define P3H2x4x_DT_IO_STRENGTH_40_OHM		40
#define P3H2x4x_DT_IO_STRENGTH_50_OHM		50

/* target port mode settings */
static const struct p3h2x4x_setting tp_mode_settings[] = {
	{ "i3c",		P3H2x4x_TP_MODE_I3C },
	{ "smbus",		P3H2x4x_TP_MODE_SMBUS },
	{ "gpio",		P3H2x4x_TP_MODE_GPIO },
	{ "i2c",		P3H2x4x_TP_MODE_I2C },
};

static const struct i3c_ibi_setup p3h2x4x_ibireq = {
	.handler = p3h2x4x_ibi_handler,
	.max_payload_len = P3H2x4x_MAX_PAYLOAD_LEN,
	.num_slots = P3H2x4x_NUM_SLOTS,
};

static void p3h2x4x_of_get_dt_setting(struct device *dev,
				      const struct device_node *node,
				      const char *setting_name,
				      const struct p3h2x4x_setting settings[],
				      const int settings_count,
				      int *setting_value)
{
	const char *sval;
	int ret;
	int i;

	ret = of_property_read_string(node, setting_name, &sval);
	if (ret) {
		if (ret != -EINVAL)
			dev_warn(dev, "No setting or invalid setting for %s, err=%i\n",
				 setting_name, ret);
		return;
	}

	for (i = 0; i < settings_count; ++i) {
		const struct p3h2x4x_setting *const setting = &settings[i];

		if (!strcmp(setting->name, sval)) {
			*setting_value = setting->value;
			return;
		}
	}
	dev_warn(dev, "Unknown setting for %s\n", setting_name);
}

static u8 p3h2x4x_pullup_dt_to_reg(int dt_value)
{
	switch (dt_value) {
	case P3H2x4x_DT_TP_PULLUP_2000R:
		return P3H2x4x_TP_PULLUP_2000R;
	case P3H2x4x_DT_TP_PULLUP_1000R:
		return P3H2x4x_TP_PULLUP_1000R;
	case P3H2x4x_DT_TP_PULLUP_250R:
		return P3H2x4x_TP_PULLUP_250R;
	default:
		return P3H2x4x_TP_PULLUP_500R;
	}
}

static u8 p3h2x4x_io_strength_dt_to_reg(int dt_value)
{
	switch (dt_value) {
	case P3H2x4x_DT_IO_STRENGTH_50_OHM:
		return P3H2x4x_IO_STRENGTH_50_OHM;
	case P3H2x4x_DT_IO_STRENGTH_40_OHM:
		return P3H2x4x_IO_STRENGTH_40_OHM;
	case P3H2x4x_DT_IO_STRENGTH_30_OHM:
		return P3H2x4x_IO_STRENGTH_30_OHM;
	default:
		return P3H2x4x_IO_STRENGTH_20_OHM;
	}
}

static int p3h2x4x_configure_ldo(struct device *dev)
{
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub = dev_get_drvdata(dev);
	int ret;

	p3h2x4x_i3c_hub->rp3h2x4x.rcp0 = devm_regulator_get_optional(dev, "cp0");
	if (IS_ERR(p3h2x4x_i3c_hub->rp3h2x4x.rcp0)) {
		p3h2x4x_i3c_hub->rp3h2x4x.rcp0 = NULL;
		dev_dbg(dev, "cp0-supply not found\n");
	}

	p3h2x4x_i3c_hub->rp3h2x4x.rcp1 = devm_regulator_get_optional(dev, "cp1");
	if (IS_ERR(p3h2x4x_i3c_hub->rp3h2x4x.rcp1)) {
		p3h2x4x_i3c_hub->rp3h2x4x.rcp1 = NULL;
		dev_dbg(dev, "cp1-supply not found\n");
	}

	p3h2x4x_i3c_hub->rp3h2x4x.rtp0145 = devm_regulator_get_optional(dev, "tp0145");
	if (IS_ERR(p3h2x4x_i3c_hub->rp3h2x4x.rtp0145)) {
		p3h2x4x_i3c_hub->rp3h2x4x.rtp0145 = NULL;
		dev_dbg(dev, "tp0145-supply not found\n");
	}

	p3h2x4x_i3c_hub->rp3h2x4x.rtp2367 = devm_regulator_get_optional(dev, "tp2367");
	if (IS_ERR(p3h2x4x_i3c_hub->rp3h2x4x.rtp2367)) {
		p3h2x4x_i3c_hub->rp3h2x4x.rtp2367 = NULL;
		dev_dbg(dev, "tp2367-supply not found\n");
	}

	/* Set the regulators volatage */
	if (p3h2x4x_i3c_hub->settings.cp0_ldo_volt != P3H2x4x_DT_LDO_VOLT_NOT_SET &&
	    p3h2x4x_i3c_hub->rp3h2x4x.rcp0) {
		ret = regulator_set_voltage(p3h2x4x_i3c_hub->rp3h2x4x.rcp0,
					    p3h2x4x_i3c_hub->settings.cp0_ldo_volt,
					    p3h2x4x_i3c_hub->settings.cp0_ldo_volt);
		if (ret)
			dev_warn(dev, "Failed to set CP0 voltage (ignoring)\n");
	}

	/* Set the regulators volatage */
	if (p3h2x4x_i3c_hub->settings.cp1_ldo_volt != P3H2x4x_DT_LDO_VOLT_NOT_SET &&
	    p3h2x4x_i3c_hub->rp3h2x4x.rcp1) {
		ret = regulator_set_voltage(p3h2x4x_i3c_hub->rp3h2x4x.rcp1,
					    p3h2x4x_i3c_hub->settings.cp1_ldo_volt,
					    p3h2x4x_i3c_hub->settings.cp1_ldo_volt);
		if (ret)
			dev_warn(dev, "Failed to set CP1 voltage (ignoring)\n");
	}

	/* Set the regulators volatage */
	if (p3h2x4x_i3c_hub->settings.tp0145_ldo_volt != P3H2x4x_DT_LDO_VOLT_NOT_SET &&
	    p3h2x4x_i3c_hub->rp3h2x4x.rtp0145) {
		ret = regulator_set_voltage(p3h2x4x_i3c_hub->rp3h2x4x.rtp0145,
					    p3h2x4x_i3c_hub->settings.tp0145_ldo_volt,
					    p3h2x4x_i3c_hub->settings.tp0145_ldo_volt);
		if (ret)
			dev_warn(dev, "Failed to set TP0145 voltage (ignoring)\n");
	}

	/* Set the regulators volatage */
	if (p3h2x4x_i3c_hub->settings.tp2367_ldo_volt != P3H2x4x_DT_LDO_VOLT_NOT_SET &&
	    p3h2x4x_i3c_hub->rp3h2x4x.rtp2367) {
		ret = regulator_set_voltage(p3h2x4x_i3c_hub->rp3h2x4x.rtp2367,
					    p3h2x4x_i3c_hub->settings.tp2367_ldo_volt,
					    p3h2x4x_i3c_hub->settings.tp2367_ldo_volt);
		if (ret)
			dev_warn(dev, "Failed to set TP2367 voltage (ignoring)\n");
	}

	return 0;
}

static int p3h2x4x_configure_pullup(struct device *dev)
{
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub = dev_get_drvdata(dev);
	u8 mask_all = 0, val_all = 0;

	if (p3h2x4x_i3c_hub->settings.tp0145_pullup != P3H2x4x_TP_PULLUP_NOT_SET) {
		val_all |= P3H2x4x_TP0145_PULLUP_CONF(p3h2x4x_pullup_dt_to_reg
						    (p3h2x4x_i3c_hub->settings.tp0145_pullup));
		mask_all |= P3H2x4x_TP0145_PULLUP_CONF_MASK;
	}

	if (p3h2x4x_i3c_hub->settings.tp2367_pullup != P3H2x4x_TP_PULLUP_NOT_SET) {
		val_all |= P3H2x4x_TP2367_PULLUP_CONF(p3h2x4x_pullup_dt_to_reg
						    (p3h2x4x_i3c_hub->settings.tp2367_pullup));
		mask_all |= P3H2x4x_TP2367_PULLUP_CONF_MASK;
	}

	return regmap_update_bits(p3h2x4x_i3c_hub->regmap, P3H2x4x_LDO_AND_PULLUP_CONF,
				  mask_all, val_all);
}

static int p3h2x4x_configure_io_strength(struct device *dev)
{
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub = dev_get_drvdata(dev);
	u8 mask_all = 0, val_all = 0;

	if (p3h2x4x_i3c_hub->settings.cp0_io_strength != P3H2x4x_IO_STRENGTH_NOT_SET) {
		val_all |= P3H2x4x_CP0_IO_STRENGTH(p3h2x4x_io_strength_dt_to_reg
					      (p3h2x4x_i3c_hub->settings.cp0_io_strength));
		mask_all |= P3H2x4x_CP0_IO_STRENGTH_MASK;
	}

	if (p3h2x4x_i3c_hub->settings.cp1_io_strength != P3H2x4x_IO_STRENGTH_NOT_SET) {
		val_all |= P3H2x4x_CP1_IO_STRENGTH(p3h2x4x_io_strength_dt_to_reg
					      (p3h2x4x_i3c_hub->settings.cp1_io_strength));
		mask_all |= P3H2x4x_CP1_IO_STRENGTH_MASK;
	}

	if (p3h2x4x_i3c_hub->settings.tp0145_io_strength != P3H2x4x_IO_STRENGTH_NOT_SET) {
		val_all |= P3H2x4x_TP0145_IO_STRENGTH(p3h2x4x_io_strength_dt_to_reg
						 (p3h2x4x_i3c_hub->settings.tp0145_io_strength));
		mask_all |= P3H2x4x_TP0145_IO_STRENGTH_MASK;
	}

	if (p3h2x4x_i3c_hub->settings.tp2367_io_strength != P3H2x4x_IO_STRENGTH_NOT_SET) {
		val_all |= P3H2x4x_TP2367_IO_STRENGTH(p3h2x4x_io_strength_dt_to_reg
						 (p3h2x4x_i3c_hub->settings.tp2367_io_strength));
		mask_all |= P3H2x4x_TP2367_IO_STRENGTH_MASK;
	}

	return regmap_update_bits(p3h2x4x_i3c_hub->regmap, P3H2x4x_IO_STRENGTH, mask_all, val_all);
}

static int p3h2x4x_configure_tp(struct device *dev)
{
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub = dev_get_drvdata(dev);
	u8 pullup_val = 0, ibi_val = 0;
	u8 smbus_val = 0, i2c_val = 0;
	u8 gpio_val = 0, i3c_val = 0;
	u8 i, mode_mask = 0;
	int ret;

	/* TBD: Read type of HUB from register P3H2x4x_DEV_INFO_0 to learn target ports count. */
	for (i = 0; i < P3H2x4x_TP_MAX_COUNT; ++i) {
		if (p3h2x4x_i3c_hub->settings.tp[i].mode != P3H2x4x_TP_MODE_NOT_SET) {
			if (p3h2x4x_i3c_hub->settings.tp[i].mode == P3H2x4x_TP_MODE_I3C)
				i3c_val |= P3H2x4x_SET_BIT(i);
			else if (p3h2x4x_i3c_hub->settings.tp[i].mode == P3H2x4x_TP_MODE_SMBUS)
				smbus_val |= P3H2x4x_SET_BIT(i);
			else if (p3h2x4x_i3c_hub->settings.tp[i].mode == P3H2x4x_TP_MODE_GPIO)
				gpio_val |= P3H2x4x_SET_BIT(i);
			else if (p3h2x4x_i3c_hub->settings.tp[i].mode == P3H2x4x_TP_MODE_I2C)
				i2c_val |= P3H2x4x_SET_BIT(i);

			mode_mask |= P3H2x4x_SET_BIT(i);
		}
		if (p3h2x4x_i3c_hub->settings.tp[i].pullup_en)
			pullup_val |= P3H2x4x_SET_BIT(i);

		if (p3h2x4x_i3c_hub->settings.tp[i].ibi_en)
			ibi_val |= P3H2x4x_SET_BIT(i);
	}

	ret = regmap_update_bits(p3h2x4x_i3c_hub->regmap, P3H2x4x_TP_IO_MODE_CONF,
				 mode_mask, (smbus_val | i2c_val));
	if (ret)
		return ret;

	ret = regmap_update_bits(p3h2x4x_i3c_hub->regmap, P3H2x4x_TP_PULLUP_EN,
				 pullup_val, pullup_val);
	if (ret)
		return ret;

	ret = regmap_update_bits(p3h2x4x_i3c_hub->regmap, P3H2x4x_TP_SMBUS_AGNT_IBI_CONFIG,
				 ibi_val, ibi_val);
	if (ret)
		return ret;
	p3h2x4x_i3c_hub->tp_ibi_mask = ibi_val;

	ret = regmap_update_bits(p3h2x4x_i3c_hub->regmap, P3H2x4x_TP_SMBUS_AGNT_EN,
				 mode_mask, smbus_val);
	if (ret)
		return ret;

	ret = regmap_update_bits(p3h2x4x_i3c_hub->regmap, P3H2x4x_TP_GPIO_MODE_EN,
				 mode_mask, gpio_val);
	if (ret)
		return ret;

	/* Request for HUB Network connection in case any TP is configured in I3C mode */
	if ((i3c_val) || (i2c_val)) {
		ret = regmap_write(p3h2x4x_i3c_hub->regmap, P3H2x4x_CP_MUX_SET,
				   P3H2x4x_CONTROLLER_PORT_MUX_REQ);
		if (ret)
			return ret;
	}

	/* Enable TP here in case TP was configured */
	ret = regmap_update_bits(p3h2x4x_i3c_hub->regmap, P3H2x4x_TP_ENABLE,
				 mode_mask, (i3c_val | smbus_val | gpio_val | i2c_val));
	if (ret)
		return ret;

	return 0;
}

static int p3h2x4x_configure_smbus_local_device(struct device *dev)
{
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub = dev_get_drvdata(dev);
	u8 target_buffer_page, hub_tp;
	int ret = 0;

	for (hub_tp = 0; hub_tp < P3H2x4x_TP_MAX_COUNT; hub_tp++) {
		if (p3h2x4x_i3c_hub->settings.tp[hub_tp].mode == P3H2x4x_TP_MODE_SMBUS &&
		    p3h2x4x_i3c_hub->tp_bus[hub_tp].local_dev_count) {
			target_buffer_page = P3H2x4x_TARGET_AGENT_LOCAL_DEV + 4 * hub_tp;
			ret = regmap_write(p3h2x4x_i3c_hub->regmap, P3H2x4x_PAGE_PTR,
					   target_buffer_page);
			if (ret) {
				dev_err(dev, "Failed to configure local device settings\n");
				break;
			}

			ret = regmap_bulk_write(p3h2x4x_i3c_hub->regmap,
						P3H2x4x_CONTROLLER_AGENT_BUFF,
						p3h2x4x_i3c_hub->tp_bus[hub_tp].local_dev_list,
						p3h2x4x_i3c_hub->tp_bus[hub_tp].local_dev_count);
			if (ret) {
				dev_err(dev, "Failed to add local devices\n");
				break;
			}
		}
	}
	regmap_write(p3h2x4x_i3c_hub->regmap, P3H2x4x_PAGE_PTR, 0x00);
	return ret;
}

static int p3h2x4x_configure_hw(struct device *dev)
{
	int ret;

	ret = p3h2x4x_configure_ldo(dev);
	if (ret)
		return ret;

	ret = p3h2x4x_configure_io_strength(dev);
	if (ret)
		return ret;

	ret = p3h2x4x_configure_pullup(dev);
	if (ret)
		return ret;

	ret = p3h2x4x_configure_smbus_local_device(dev);
	if (ret)
		return ret;

	return p3h2x4x_configure_tp(dev);
}

static void p3h2x4x_of_get_tp_dt_conf(struct device *dev,
				      const struct device_node *node)
{
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub = dev_get_drvdata(dev);
	struct device_node *dev_node;
	u64 tp_port;

	for_each_available_child_of_node(node, dev_node) {
		if (of_property_read_reg(dev_node, 0, &tp_port, NULL))
			continue;

		if (tp_port < P3H2x4x_TP_MAX_COUNT) {
			p3h2x4x_i3c_hub->tp_bus[tp_port].of_node = dev_node;
			p3h2x4x_i3c_hub->tp_bus[tp_port].tp_mask = BIT(tp_port);
			p3h2x4x_i3c_hub->tp_bus[tp_port].p3h2x4x_i3c_hub = p3h2x4x_i3c_hub;
			p3h2x4x_i3c_hub->tp_bus[tp_port].tp_port = tp_port;
		}
	}
}

/* return true when backend node exist */
static bool p3h2x4x_is_backend_node_exist(int port,
					  struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub,
					  u32 addr)
{
	struct smbus_device *backend = NULL;

	list_for_each_entry(backend,
			    &p3h2x4x_i3c_hub->tp_bus[port].tp_device_entry, list) {
		if (backend->addr == addr)
			return true;
	}
	return false;
}

static int p3h2x4x_read_backend_from_dts(struct device_node *i3c_node_target,
					 struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub)
{
	struct device_node *tp_node;
	const char *compatible;
	u64 tp_port, addr_dts;
	int ret;

	struct smbus_device *backend;

	if (of_property_read_reg(i3c_node_target, 0, &tp_port, NULL))
		return -EINVAL;

	if (tp_port >= P3H2x4x_TP_MAX_COUNT || tp_port < 0)
		return -ERANGE;

	INIT_LIST_HEAD(&p3h2x4x_i3c_hub->tp_bus[tp_port].tp_device_entry);

	if (p3h2x4x_i3c_hub->settings.tp[tp_port].mode == P3H2x4x_TP_MODE_I3C)
		return 0;

	for_each_available_child_of_node(i3c_node_target, tp_node) {
		ret = of_property_read_reg(tp_node, 0, &addr_dts, NULL);
		if (ret)
			return ret;

		if (p3h2x4x_is_backend_node_exist(tp_port, p3h2x4x_i3c_hub, addr_dts))
			continue;

		ret = of_property_read_string(tp_node, "compatible", &compatible);
		if (ret)
			return ret;

		backend = kzalloc(sizeof(*backend), GFP_KERNEL);
		if (!backend)
			return -ENOMEM;

		backend->addr = addr_dts;
		backend->compatible = compatible;
		backend->tp_device_dt_node = tp_node;
		backend->client = NULL;

		list_add(&backend->list,
			 &p3h2x4x_i3c_hub->tp_bus[tp_port].tp_device_entry);
	}

	return 0;
}

static void p3h2x4x_parse_dt_tp(struct device *dev,
				const struct device_node *i3c_node_hub,
				struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub)
{
	struct device_node *i3c_node_target;
	int ret;

	for_each_available_child_of_node(i3c_node_hub, i3c_node_target) {
		ret = p3h2x4x_read_backend_from_dts(i3c_node_target, p3h2x4x_i3c_hub);
		if (ret)
			dev_warn(dev, "Skiping DTS node %s", i3c_node_target->name);
	}
}

static int p3h2x4x_get_tp_local_device_dt_setting(struct device *dev,
						  const struct device_node *node, u32 id)
{
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub = dev_get_drvdata(dev);
	int ret;

	ret = of_property_read_variable_u8_array(node, "local-dev",
						 p3h2x4x_i3c_hub->tp_bus[id].local_dev_list,
						 sizeof(u8), P3H2x4x_TP_LOCAL_DEV);
	if (ret > 0 && ret <= P3H2x4x_TP_LOCAL_DEV)
		p3h2x4x_i3c_hub->tp_bus[id].local_dev_count = ret;
	else if (ret == -EOVERFLOW)
		dev_warn(dev,
			 "local Devices list is out of range or invalid\n");

	return ret;
}

static void p3h2x4x_get_tp_of_get_setting(struct device *dev,
					  const struct device_node *node,
					  struct tp_setting tp_setting[])
{
	u64 id;

	for_each_available_child_of_node_scoped(node, tp_node) {
		if (of_property_read_reg(tp_node, 0, &id, NULL))
			continue;

		if (id >= P3H2x4x_TP_MAX_COUNT) {
			dev_warn(dev, "Invalid target port index found in DT: %lli\n", id);
			continue;
		}

		p3h2x4x_of_get_dt_setting(dev, tp_node, "mode", tp_mode_settings,
					  ARRAY_SIZE(tp_mode_settings),
					  &tp_setting[id].mode);

		tp_setting[id].pullup_en =
					of_property_read_bool(tp_node, "pullup-enable");
		tp_setting[id].ibi_en =
					of_property_read_bool(tp_node, "ibi-enable");
		tp_setting[id].always_enable =
					of_property_read_bool(tp_node, "hub-bridge-en");

		p3h2x4x_get_tp_local_device_dt_setting(dev, tp_node, id);
	}
}

static void p3h2x4x_of_get_p3h2x4x_conf(struct device *dev,
					const struct device_node *node)
{
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub = dev_get_drvdata(dev);

	of_property_read_u32(node, "cp0-ldo-microvolt",
			     &p3h2x4x_i3c_hub->settings.cp0_ldo_volt);
	of_property_read_u32(node, "cp1-ldo-microvolt",
			     &p3h2x4x_i3c_hub->settings.cp1_ldo_volt);
	of_property_read_u32(node, "tp0145-ldo-microvolt",
			     &p3h2x4x_i3c_hub->settings.tp0145_ldo_volt);
	of_property_read_u32(node, "tp2367-ldo-microvolt",
			     &p3h2x4x_i3c_hub->settings.tp2367_ldo_volt);
	of_property_read_u32(node, "tp0145-pullup-ohms",
			     &p3h2x4x_i3c_hub->settings.tp0145_pullup);
	of_property_read_u32(node, "tp2367-pullup-ohms",
			     &p3h2x4x_i3c_hub->settings.tp2367_pullup);
	of_property_read_u32(node, "cp0-io-strength-ohms",
			     &p3h2x4x_i3c_hub->settings.cp0_io_strength);
	of_property_read_u32(node, "cp1-io-strength-ohms",
			     &p3h2x4x_i3c_hub->settings.cp1_io_strength);
	of_property_read_u32(node, "tp0145-io-strength-ohms",
			     &p3h2x4x_i3c_hub->settings.tp0145_io_strength);
	of_property_read_u32(node, "tp2367-io-strength-ohms",
			     &p3h2x4x_i3c_hub->settings.tp2367_io_strength);

	p3h2x4x_get_tp_of_get_setting(dev, node, p3h2x4x_i3c_hub->settings.tp);
}

static void p3h2x4x_of_default_configuration(struct device *dev)
{
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub = dev_get_drvdata(dev);
	int tp_count;

	p3h2x4x_i3c_hub->settings.cp0_ldo_volt = P3H2x4x_DT_LDO_VOLT_NOT_SET;
	p3h2x4x_i3c_hub->settings.cp1_ldo_volt = P3H2x4x_DT_LDO_VOLT_NOT_SET;
	p3h2x4x_i3c_hub->settings.tp0145_ldo_volt = P3H2x4x_DT_LDO_VOLT_NOT_SET;
	p3h2x4x_i3c_hub->settings.tp2367_ldo_volt = P3H2x4x_DT_LDO_VOLT_NOT_SET;
	p3h2x4x_i3c_hub->settings.tp0145_pullup = P3H2x4x_TP_PULLUP_NOT_SET;
	p3h2x4x_i3c_hub->settings.tp2367_pullup = P3H2x4x_TP_PULLUP_NOT_SET;
	p3h2x4x_i3c_hub->settings.cp0_io_strength = P3H2x4x_IO_STRENGTH_NOT_SET;
	p3h2x4x_i3c_hub->settings.cp1_io_strength = P3H2x4x_IO_STRENGTH_NOT_SET;
	p3h2x4x_i3c_hub->settings.tp0145_io_strength = P3H2x4x_IO_STRENGTH_NOT_SET;
	p3h2x4x_i3c_hub->settings.tp2367_io_strength = P3H2x4x_IO_STRENGTH_NOT_SET;

	for (tp_count = 0; tp_count < P3H2x4x_TP_MAX_COUNT; ++tp_count)
		p3h2x4x_i3c_hub->settings.tp[tp_count].mode =  P3H2x4x_TP_MODE_NOT_SET;
}

static int p3h2x4x_i3c_hub_probe(struct platform_device *pdev)
{
	struct p3h2x4x_dev *p3h2x4x = dev_get_drvdata(pdev->dev.parent);
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	int ret, i;

	p3h2x4x_i3c_hub = devm_kzalloc(dev, sizeof(*p3h2x4x_i3c_hub), GFP_KERNEL);
	if (!p3h2x4x_i3c_hub)
		return -ENOMEM;

	platform_set_drvdata(pdev, p3h2x4x_i3c_hub);

	p3h2x4x_i3c_hub->regmap = p3h2x4x->regmap;

	if (p3h2x4x->is_p3h2x4x_in_i3c) {
		p3h2x4x_i3c_hub->i3cdev = p3h2x4x->i3cdev;
		p3h2x4x_i3c_hub->driving_master = i3c_dev_get_master(p3h2x4x->i3cdev->desc);
	}

	p3h2x4x_i3c_hub->is_p3h2x4x_in_i3c = p3h2x4x->is_p3h2x4x_in_i3c;
	p3h2x4x_i3c_hub->dev = dev;

	device_set_of_node_from_dev(dev, dev->parent);

	p3h2x4x_of_default_configuration(dev);
	mutex_init(&p3h2x4x_i3c_hub->etx_mutex);

	for (i = 0; i < P3H2x4x_TP_MAX_COUNT; i++)
		mutex_init(&p3h2x4x_i3c_hub->tp_bus[i].port_mutex);

	/* get hub node from DT */
	node =  dev->of_node;
	if (!node) {
		dev_dbg(dev, "No device tree entry found, using hardware defaults.\n");
	} else {
		p3h2x4x_of_get_p3h2x4x_conf(dev, node);
		p3h2x4x_of_get_tp_dt_conf(dev, node);
		/* Parse DTS to find backend device on the SMBus target mode */
		p3h2x4x_parse_dt_tp(dev, node, p3h2x4x_i3c_hub);
	}

	/* Unlock access to protected registers */
	ret = regmap_write(p3h2x4x_i3c_hub->regmap, P3H2x4x_DEV_REG_PROTECTION_CODE,
			   P3H2x4x_REGISTERS_UNLOCK_CODE);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to unlock HUB's protected registers\n");

	ret = p3h2x4x_configure_hw(dev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to configure the HUB\n");

	if (p3h2x4x->is_p3h2x4x_in_i3c) {
		/* Register logic for native vertual I3C ports */
		for (i = 0; i < P3H2x4x_TP_MAX_COUNT; i++) {
			if (p3h2x4x_i3c_hub->settings.tp[i].mode == P3H2x4x_TP_MODE_I3C &&
			    !p3h2x4x_i3c_hub->settings.tp[i].always_enable) {
				ret = p3h2x4x_tp_i3c_algo(p3h2x4x_i3c_hub, i);
				if (ret)
					return dev_err_probe(dev, ret,
							    "failed to register i3c bus tp %d\n",
							    i);
			}
		}
	}

	for (i = 0; i < P3H2x4x_TP_MAX_COUNT; ++i) {
		if (p3h2x4x_i3c_hub->settings.tp[i].always_enable)
			p3h2x4x_i3c_hub->tp_always_enable_mask =
					(p3h2x4x_i3c_hub->tp_always_enable_mask |  BIT(i));
	}

	ret = regmap_write(p3h2x4x_i3c_hub->regmap, P3H2x4x_TP_NET_CON_CONF,
			   p3h2x4x_i3c_hub->tp_always_enable_mask);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to open Target Port(s)\n");

	if (p3h2x4x->is_p3h2x4x_in_i3c) {
		ret = i3c_master_do_daa(p3h2x4x_i3c_hub->driving_master);
		if (ret)
			dev_warn(dev, "Failed to run DAA\n");

		/*
		 * holding SDA low when both SMBus Target Agent received data buffers are full.
		 * This feature can be used as a flow-control mechanism for MCTP applications to
		 * avoid MCTP transmitters on Target Ports time out when the SMBus agent buffers
		 * are not serviced in time by upstream controller and only receives write message
		 * from its downstream ports.
		 */
		ret = regmap_update_bits(p3h2x4x_i3c_hub->regmap,
					 P3H2x4x_ONCHIP_TD_AND_SMBUS_AGNT_CONF,
					 P3H2x4x_TARGET_AGENT_DFT_IBI_CONF_MASK,
					 P3H2x4x_TARGET_AGENT_DFT_IBI_CONF);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to data buffer full config\n");

		ret = i3c_device_request_ibi(p3h2x4x->i3cdev, &p3h2x4x_ibireq);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to request IBI\n");

		ret = i3c_device_enable_ibi(p3h2x4x->i3cdev);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to Enable IBI\n");
	}

	/* Register logic for native SMBus ports */
	for (i = 0; i < P3H2x4x_TP_MAX_COUNT; i++) {
		if (p3h2x4x_i3c_hub->settings.tp[i].mode == P3H2x4x_TP_MODE_SMBUS) {
			ret = p3h2x4x_tp_smbus_algo(p3h2x4x_i3c_hub, i);
			if (ret)
				return dev_err_probe(dev, ret,
						    "failed to add i2c adapter for tp %d\n", i);
		}
	}

	/* Lock access to protected registers */
	ret = regmap_write(p3h2x4x_i3c_hub->regmap, P3H2x4x_DEV_REG_PROTECTION_CODE,
			   P3H2x4x_REGISTERS_LOCK_CODE);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to lock HUB's protected registers\n");

	return 0;
}

static void p3h2x4x_i3c_hub_remove(struct platform_device *pdev)
{
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub = platform_get_drvdata(pdev);
	struct p3h2x4x_dev *p3h2x4x = dev_get_drvdata(pdev->dev.parent);
	struct i3c_master_controller *tp_controller;
	struct smbus_device *backend = NULL;
	struct i2c_adapter *tp_adap;
	u8 i;

	for (i = 0; i < P3H2x4x_TP_MAX_COUNT; i++) {
		tp_adap = p3h2x4x_i3c_hub->tp_bus[i].smbus_port_adapter;
		tp_controller = &p3h2x4x_i3c_hub->tp_bus[i].i3c_port_controller;

		if (p3h2x4x_i3c_hub->tp_bus[i].is_registered) {
			if (p3h2x4x_i3c_hub->settings.tp[i].mode == P3H2x4x_TP_MODE_SMBUS) {
				list_for_each_entry(backend,
						    &p3h2x4x_i3c_hub->tp_bus[i].tp_device_entry,
						    list) {
					i2c_unregister_device(backend->client);
					kfree(backend);
				}
				i2c_del_adapter(tp_adap);
			} else if (p3h2x4x_i3c_hub->settings.tp[i].mode == P3H2x4x_TP_MODE_I3C) {
				i3c_master_unregister(tp_controller);
			}
		}
	}
	if (p3h2x4x->is_p3h2x4x_in_i3c) {
		i3c_device_disable_ibi(p3h2x4x->i3cdev);
		i3c_device_free_ibi(p3h2x4x->i3cdev);
	}

	mutex_destroy(&p3h2x4x_i3c_hub->etx_mutex);
	for (i = 0; i < P3H2x4x_TP_MAX_COUNT; i++)
		mutex_destroy(&p3h2x4x_i3c_hub->tp_bus[i].port_mutex);

}

static struct platform_driver p3h2x4x_i3c_hub_driver = {
	.driver = {
		.name = "p3h2x4x-i3c-hub",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = p3h2x4x_i3c_hub_probe,
	.remove = p3h2x4x_i3c_hub_remove,
};
module_platform_driver(p3h2x4x_i3c_hub_driver);

MODULE_AUTHOR("Aman Kumar Pandey <aman.kumarpandey@nxp.com>");
MODULE_AUTHOR("vikash Bansal <vikash.bansal@nxp.com>");
MODULE_DESCRIPTION("P3H2x4x I3C HUB driver");
MODULE_LICENSE("GPL");
