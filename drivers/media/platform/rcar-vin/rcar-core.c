/*
 * Driver for Renesas R-Car VIN
 *
 * Copyright (C) 2016 Renesas Electronics Corp.
 * Copyright (C) 2011-2013 Renesas Solutions Corp.
 * Copyright (C) 2013 Cogent Embedded, Inc., <source@cogentembedded.com>
 * Copyright (C) 2008 Magnus Damm
 *
 * Based on the soc-camera rcar_vin driver
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include <media/v4l2-fwnode.h>

#include "rcar-vin.h"

/* -----------------------------------------------------------------------------
 * Media Controller link notification
 */

static unsigned int rvin_group_csi_pad_to_chan(unsigned int pad)
{
	/*
	 * The CSI2 driver is rcar-csi2 and we know it's pad layout are
	 * 0: Source 1-4: Sinks so if we remove one from the pad we
	 * get the rcar-vin internal CSI2 channel number
	 */
	return pad - 1;
}

/* group lock should be held when calling this function */
static int rvin_group_entity_to_vin_num(struct rvin_group *group,
					struct media_entity *entity)
{
	struct video_device *vdev;
	int i;

	if (!is_media_entity_v4l2_video_device(entity))
		return -ENODEV;

	vdev = media_entity_to_video_device(entity);

	for (i = 0; i < RCAR_VIN_NUM; i++) {
		if (!group->vin[i])
			continue;

		if (&group->vin[i]->vdev == vdev)
			return i;
	}

	return -ENODEV;
}

/* group lock should be held when calling this function */
static int rvin_group_entity_to_csi_num(struct rvin_group *group,
					struct media_entity *entity)
{
	struct v4l2_subdev *sd;
	int i;

	if (!is_media_entity_v4l2_subdev(entity))
		return -ENODEV;

	sd = media_entity_to_v4l2_subdev(entity);

	for (i = 0; i < RVIN_CSI_MAX; i++)
		if (group->csi[i].subdev == sd)
			return i;

	return -ENODEV;
}

/* group lock should be held when calling this function */
static void __rvin_group_build_link_list(struct rvin_group *group,
					 struct rvin_group_chsel *map,
					 int start, int len)
{
	struct media_pad *vin_pad, *remote_pad;
	unsigned int n;

	for (n = 0; n < len; n++) {
		map[n].csi = -1;
		map[n].chan = -1;

		if (!group->vin[start + n])
			continue;

		vin_pad = &group->vin[start + n]->vdev.entity.pads[0];

		remote_pad = media_entity_remote_pad(vin_pad);
		if (!remote_pad)
			continue;

		map[n].csi =
			rvin_group_entity_to_csi_num(group, remote_pad->entity);
		map[n].chan = rvin_group_csi_pad_to_chan(remote_pad->index);
	}
}

/* group lock should be held when calling this function */
static int __rvin_group_try_get_chsel(struct rvin_group *group,
				      struct rvin_group_chsel *map,
				      int start, int len)
{
	const struct rvin_group_chsel *sel;
	unsigned int i, n;
	int chsel;

	for (i = 0; i < group->vin[start]->info->num_chsels; i++) {
		chsel = i;
		for (n = 0; n < len; n++) {

			/* If the link is not active it's OK */
			if (map[n].csi == -1)
				continue;

			/* Check if chsel match requested link */
			sel = &group->vin[start]->info->chsels[start + n][i];
			if (map[n].csi != sel->csi ||
			    map[n].chan != sel->chan) {
				chsel = -1;
				break;
			}
		}

		/* A chsel which satisfy the links have been found */
		if (chsel != -1)
			return chsel;
	}

	/* No chsel can satisfy the requested links */
	return -1;
}

/* group lock should be held when calling this function */
static bool rvin_group_in_use(struct rvin_group *group)
{
	struct media_entity *entity;

	media_device_for_each_entity(entity, &group->mdev)
		if (entity->use_count)
			return true;

	return false;
}

static int rvin_group_link_notify(struct media_link *link, u32 flags,
				  unsigned int notification)
{
	struct rvin_group *group = container_of(link->graph_obj.mdev,
						struct rvin_group, mdev);
	struct rvin_group_chsel chsel_map[4];
	int vin_num, vin_master, csi_num, csi_chan;
	unsigned int chsel;

	mutex_lock(&group->lock);

	vin_num = rvin_group_entity_to_vin_num(group, link->sink->entity);
	csi_num = rvin_group_entity_to_csi_num(group, link->source->entity);
	csi_chan = rvin_group_csi_pad_to_chan(link->source->index);

	/*
	 * Figure out which VIN node is the subgroup master.
	 *
	 * VIN0-3 are controlled by VIN0
	 * VIN4-7 are controlled by VIN4
	 */
	vin_master = vin_num < 4 ? 0 : 4;

	/* If not all devices exists something is horribly wrong */
	if (vin_num < 0 || csi_num < 0 || !group->vin[vin_master])
		goto error;

	/* Special checking only needed for links which are to be enabled */
	if (notification != MEDIA_DEV_NOTIFY_PRE_LINK_CH ||
	    !(flags & MEDIA_LNK_FL_ENABLED))
		goto out;

	/* If any link in the group are in use, no new link can be enabled */
	if (rvin_group_in_use(group))
		goto error;

	/* If the VIN already have a active link it's busy */
	if (media_entity_remote_pad(&link->sink->entity->pads[0]))
		goto error;

	/* Build list of active links */
	__rvin_group_build_link_list(group, chsel_map, vin_master, 4);

	/* Add the new proposed link */
	chsel_map[vin_num - vin_master].csi = csi_num;
	chsel_map[vin_num - vin_master].chan = csi_chan;

	/* See if there is a chsel value which match our link selection */
	chsel = __rvin_group_try_get_chsel(group, chsel_map, vin_master, 4);

	/* No chsel can provide the request links */
	if (chsel == -1)
		goto error;

	/* Update chsel value at group master */
	rvin_set_chsel(group->vin[vin_master], chsel);

out:
	mutex_unlock(&group->lock);

	return v4l2_pipeline_link_notify(link, flags, notification);
error:
	mutex_unlock(&group->lock);

	return -EMLINK;
}

static const struct media_device_ops rvin_media_ops = {
	.link_notify = rvin_group_link_notify,
};

/* -----------------------------------------------------------------------------
 * Gen3 CSI2 Group Allocator
 */

static int rvin_group_read_id(struct rvin_dev *vin, struct device_node *np)
{
	u32 val;
	int ret;

	ret = of_property_read_u32(np, "renesas,id", &val);
	if (ret) {
		vin_err(vin, "%s: No renesas,id property found\n",
			of_node_full_name(np));
		return -EINVAL;
	}

	if (val >= RCAR_VIN_NUM) {
		vin_err(vin, "%s: Invalid renesas,id '%u'\n",
			of_node_full_name(np), val);
		return -EINVAL;
	}

	return val;
}

static DEFINE_MUTEX(rvin_group_lock);
static struct rvin_group *rvin_group_data;

static void rvin_group_release(struct kref *kref)
{
	struct rvin_group *group =
		container_of(kref, struct rvin_group, refcount);

	mutex_lock(&rvin_group_lock);

	media_device_unregister(&group->mdev);
	media_device_cleanup(&group->mdev);

	rvin_group_data = NULL;

	mutex_unlock(&rvin_group_lock);

	kfree(group);
}

static struct rvin_group *__rvin_group_allocate(struct rvin_dev *vin)
{
	struct rvin_group *group;

	if (rvin_group_data) {
		group = rvin_group_data;
		kref_get(&group->refcount);
		vin_dbg(vin, "%s: get group=%p\n", __func__, group);
		return group;
	}

	group = kzalloc(sizeof(*group), GFP_KERNEL);
	if (!group)
		return NULL;

	kref_init(&group->refcount);
	rvin_group_data = group;

	vin_dbg(vin, "%s: alloc group=%p\n", __func__, group);
	return group;
}

static int rvin_group_add_vin(struct rvin_dev *vin)
{
	int ret;

	ret = rvin_group_read_id(vin, vin->dev->of_node);
	if (ret < 0)
		return ret;

	mutex_lock(&vin->group->lock);

	if (vin->group->vin[ret]) {
		mutex_unlock(&vin->group->lock);
		vin_err(vin, "VIN number %d already occupied\n", ret);
		return -EINVAL;
	}

	vin->group->vin[ret] = vin;

	mutex_unlock(&vin->group->lock);

	vin_dbg(vin, "I'm VIN number %d", ret);

	return 0;
}

static int rvin_group_allocate(struct rvin_dev *vin)
{
	struct rvin_group *group;
	struct media_device *mdev;
	int ret;

	mutex_lock(&rvin_group_lock);

	group = __rvin_group_allocate(vin);
	if (!group) {
		mutex_unlock(&rvin_group_lock);
		return -ENOMEM;
	}

	/* Init group data if its not already initialized */
	mdev = &group->mdev;
	if (!mdev->dev) {
		mutex_init(&group->lock);
		mdev->dev = vin->dev;

		strlcpy(mdev->driver_name, "Renesas VIN",
			sizeof(mdev->driver_name));
		strlcpy(mdev->model, vin->dev->of_node->name,
			sizeof(mdev->model));
		strlcpy(mdev->bus_info, of_node_full_name(vin->dev->of_node),
			sizeof(mdev->bus_info));
		media_device_init(mdev);

		mdev->ops = &rvin_media_ops;

		ret = media_device_register(mdev);
		if (ret) {
			vin_err(vin, "Failed to register media device\n");
			kref_put(&group->refcount, rvin_group_release);
			mutex_unlock(&rvin_group_lock);
			return ret;
		}
	}

	vin->group = group;
	vin->v4l2_dev.mdev = mdev;

	ret = rvin_group_add_vin(vin);
	if (ret) {
		kref_put(&group->refcount, rvin_group_release);
		mutex_unlock(&rvin_group_lock);
		return ret;
	}

	mutex_unlock(&rvin_group_lock);

	return 0;
}

static void rvin_group_delete(struct rvin_dev *vin)
{
	unsigned int i;

	mutex_lock(&vin->group->lock);
	for (i = 0; i < RCAR_VIN_NUM; i++)
		if (vin->group->vin[i] == vin)
			vin->group->vin[i] = NULL;
	mutex_unlock(&vin->group->lock);

	vin_dbg(vin, "%s: group=%p\n", __func__, &vin->group);
	kref_put(&vin->group->refcount, rvin_group_release);
}

/* -----------------------------------------------------------------------------
 * Async notifier
 */

#define notifier_to_vin(n) container_of(n, struct rvin_dev, notifier)

static int rvin_find_pad(struct v4l2_subdev *sd, int direction)
{
	unsigned int pad;

	if (sd->entity.num_pads <= 1)
		return 0;

	for (pad = 0; pad < sd->entity.num_pads; pad++)
		if (sd->entity.pads[pad].flags & direction)
			return pad;

	return -EINVAL;
}

/* -----------------------------------------------------------------------------
 * Digital async notifier
 */

static bool rvin_mbus_supported(struct rvin_dev *vin)
{
	struct v4l2_subdev *sd = vin->digital.subdev;
	struct v4l2_subdev_mbus_code_enum code = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};

	code.index = 0;
	code.pad = vin->digital.source_pad;
	while (!v4l2_subdev_call(sd, pad, enum_mbus_code, NULL, &code)) {
		code.index++;
		switch (code.code) {
		case MEDIA_BUS_FMT_YUYV8_1X16:
		case MEDIA_BUS_FMT_UYVY8_2X8:
		case MEDIA_BUS_FMT_UYVY10_2X10:
		case MEDIA_BUS_FMT_RGB888_1X24:
			vin->code = code.code;
			return true;
		default:
			break;
		}
	}

	return false;
}

static int rvin_digital_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct rvin_dev *vin = notifier_to_vin(notifier);
	struct v4l2_subdev *sd = vin_to_source(vin);
	int ret;

	/* Verify subdevices mbus format */
	if (!rvin_mbus_supported(vin)) {
		vin_err(vin, "Unsupported media bus format for %s\n",
			vin->digital.subdev->name);
		return -EINVAL;
	}

	vin_dbg(vin, "Found media bus format for %s: %d\n",
		vin->digital.subdev->name, vin->code);

	ret = v4l2_device_register_subdev_nodes(&vin->v4l2_dev);
	if (ret < 0) {
		vin_err(vin, "Failed to register subdev nodes\n");
		return ret;
	}

	/* Add the controls */
	/*
	 * Currently the subdev with the largest number of controls (13) is
	 * ov6550. So let's pick 16 as a hint for the control handler. Note
	 * that this is a hint only: too large and you waste some memory, too
	 * small and there is a (very) small performance hit when looking up
	 * controls in the internal hash.
	 */
	ret = v4l2_ctrl_handler_init(&vin->ctrl_handler, 16);
	if (ret < 0)
		return ret;

	ret = v4l2_ctrl_add_handler(&vin->ctrl_handler, sd->ctrl_handler, NULL);
	if (ret < 0)
		return ret;

	ret = v4l2_subdev_call(sd, video, g_tvnorms, &vin->vdev.tvnorms);
	if (ret < 0 && ret != -ENOIOCTLCMD && ret != -ENODEV)
		return ret;

	if (vin->vdev.tvnorms == 0) {
		/* Disable the STD API if there are no tvnorms defined */
		v4l2_disable_ioctl(&vin->vdev, VIDIOC_G_STD);
		v4l2_disable_ioctl(&vin->vdev, VIDIOC_S_STD);
		v4l2_disable_ioctl(&vin->vdev, VIDIOC_QUERYSTD);
		v4l2_disable_ioctl(&vin->vdev, VIDIOC_ENUMSTD);
	}

	return rvin_reset_format(vin);
}

static void rvin_digital_notify_unbind(struct v4l2_async_notifier *notifier,
				       struct v4l2_subdev *subdev,
				       struct v4l2_async_subdev *asd)
{
	struct rvin_dev *vin = notifier_to_vin(notifier);

	vin_dbg(vin, "unbind digital subdev %s\n", subdev->name);
	v4l2_ctrl_handler_free(&vin->ctrl_handler);
	vin->digital.subdev = NULL;
}

static int rvin_digital_notify_bound(struct v4l2_async_notifier *notifier,
				     struct v4l2_subdev *subdev,
				     struct v4l2_async_subdev *asd)
{
	struct rvin_dev *vin = notifier_to_vin(notifier);
	int ret;

	v4l2_set_subdev_hostdata(subdev, vin);

	/* Find source and sink pad of remote subdevice */

	ret = rvin_find_pad(subdev, MEDIA_PAD_FL_SOURCE);
	if (ret < 0)
		return ret;
	vin->digital.source_pad = ret;

	ret = rvin_find_pad(subdev, MEDIA_PAD_FL_SINK);
	vin->digital.sink_pad = ret < 0 ? 0 : ret;

	vin->digital.subdev = subdev;

	vin_dbg(vin, "bound subdev %s source pad: %u sink pad: %u\n",
		subdev->name, vin->digital.source_pad,
		vin->digital.sink_pad);

	return 0;
}

static int rvin_digitial_parse_v4l2(struct rvin_dev *vin,
				    struct device_node *ep,
				    struct v4l2_mbus_config *mbus_cfg)
{
	struct v4l2_fwnode_endpoint v4l2_ep;
	int ret;

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(ep), &v4l2_ep);
	if (ret) {
		vin_err(vin, "Could not parse v4l2 endpoint\n");
		return -EINVAL;
	}

	mbus_cfg->type = v4l2_ep.bus_type;

	switch (mbus_cfg->type) {
	case V4L2_MBUS_PARALLEL:
		vin_dbg(vin, "Found PARALLEL media bus\n");
		mbus_cfg->flags = v4l2_ep.bus.parallel.flags;
		break;
	case V4L2_MBUS_BT656:
		vin_dbg(vin, "Found BT656 media bus\n");
		mbus_cfg->flags = 0;
		break;
	default:
		vin_err(vin, "Unknown media bus type\n");
		return -EINVAL;
	}

	return 0;
}

static int rvin_digital_graph_parse(struct rvin_dev *vin)
{
	struct device_node *ep, *np;
	int ret;

	vin->digital.asd.match.fwnode.fwnode = NULL;
	vin->digital.subdev = NULL;

	/*
	 * Port 0 id 0 is local digital input, try to get it.
	 * Not all instances can or will have this, that is OK
	 */
	ep = of_graph_get_endpoint_by_regs(vin->dev->of_node, 0, 0);
	if (!ep)
		return 0;

	np = of_graph_get_remote_port_parent(ep);
	if (!np) {
		vin_err(vin, "No remote parent for digital input\n");
		of_node_put(ep);
		return -EINVAL;
	}
	of_node_put(np);

	ret = rvin_digitial_parse_v4l2(vin, ep, &vin->mbus_cfg);
	of_node_put(ep);
	if (ret)
		return ret;

	vin->digital.asd.match.fwnode.fwnode = of_fwnode_handle(np);
	vin->digital.asd.match_type = V4L2_ASYNC_MATCH_FWNODE;

	return 0;
}

static int rvin_digital_graph_init(struct rvin_dev *vin)
{
	struct v4l2_async_subdev **subdevs = NULL;
	int ret;

	ret = rvin_digital_graph_parse(vin);
	if (ret)
		return ret;

	if (!vin->digital.asd.match.fwnode.fwnode) {
		vin_dbg(vin, "No digital subdevice found\n");
		return -ENODEV;
	}

	/* Register the subdevices notifier. */
	subdevs = devm_kzalloc(vin->dev, sizeof(*subdevs), GFP_KERNEL);
	if (subdevs == NULL)
		return -ENOMEM;

	subdevs[0] = &vin->digital.asd;

	vin_dbg(vin, "Found digital subdevice %pOF\n",
		to_of_node(subdevs[0]->match.fwnode.fwnode));

	vin->notifier.num_subdevs = 1;
	vin->notifier.subdevs = subdevs;
	vin->notifier.bound = rvin_digital_notify_bound;
	vin->notifier.unbind = rvin_digital_notify_unbind;
	vin->notifier.complete = rvin_digital_notify_complete;

	ret = rvin_v4l2_probe(vin);
	if (ret)
		return ret;

	ret = v4l2_async_notifier_register(&vin->v4l2_dev, &vin->notifier);
	if (ret < 0) {
		vin_err(vin, "Notifier registration failed\n");
		return ret;
	}

	return 0;
}

/* -----------------------------------------------------------------------------
 * Group async notifier
 */

/* group lock should be held when calling this function */
static int rvin_group_add_link(struct rvin_dev *vin,
			       struct media_entity *source,
			       unsigned int source_idx,
			       struct media_entity *sink,
			       unsigned int sink_idx,
			       u32 flags)
{
	struct media_pad *source_pad, *sink_pad;
	int ret = 0;

	source_pad = &source->pads[source_idx];
	sink_pad = &sink->pads[sink_idx];

	if (!media_entity_find_link(source_pad, sink_pad))
		ret = media_create_pad_link(source, source_idx,
					    sink, sink_idx, flags);

	if (ret)
		vin_err(vin, "Error adding link from %s to %s\n",
			source->name, sink->name);

	return ret;
}

static int rvin_group_update_links(struct rvin_dev *vin)
{
	struct media_entity *source, *sink;
	struct rvin_dev *master;
	unsigned int i, n, idx, chsel, csi;
	u32 flags;
	int ret;

	mutex_lock(&vin->group->lock);

	for (n = 0; n < RCAR_VIN_NUM; n++) {

		/* Check that VIN is part of the group */
		if (!vin->group->vin[n])
			continue;

		/* Check that subgroup master is part of the group */
		master = vin->group->vin[n < 4 ? 0 : 4];
		if (!master)
			continue;

		chsel = rvin_get_chsel(master);

		for (i = 0; i < vin->info->num_chsels; i++) {
			csi = vin->info->chsels[n][i].csi;

			/* If the CSI-2 is out of bounds it's a noop, skip */
			if (csi >= RVIN_CSI_MAX)
				continue;

			/* Check that CSI-2 are part of the group */
			if (!vin->group->csi[csi].subdev)
				continue;

			source = &vin->group->csi[csi].subdev->entity;
			sink = &vin->group->vin[n]->vdev.entity;
			idx = vin->info->chsels[n][i].chan + 1;
			flags = i == chsel ? MEDIA_LNK_FL_ENABLED : 0;

			ret = rvin_group_add_link(vin, source, idx, sink, 0,
						  flags);
			if (ret)
				goto out;
		}
	}
out:
	mutex_unlock(&vin->group->lock);

	return ret;
}

static int rvin_group_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct rvin_dev *vin = notifier_to_vin(notifier);
	int ret;

	ret = v4l2_device_register_subdev_nodes(&vin->v4l2_dev);
	if (ret) {
		vin_err(vin, "Failed to register subdev nodes\n");
		return ret;
	}

	return rvin_group_update_links(vin);
}

static void rvin_group_notify_unbind(struct v4l2_async_notifier *notifier,
				     struct v4l2_subdev *subdev,
				     struct v4l2_async_subdev *asd)
{
	struct rvin_dev *vin = notifier_to_vin(notifier);
	struct rvin_graph_entity *csi = to_rvin_graph_entity(asd);

	mutex_lock(&vin->group->lock);
	csi->subdev = NULL;
	mutex_unlock(&vin->group->lock);
}

static int rvin_group_notify_bound(struct v4l2_async_notifier *notifier,
				   struct v4l2_subdev *subdev,
				   struct v4l2_async_subdev *asd)
{
	struct rvin_dev *vin = notifier_to_vin(notifier);
	struct rvin_graph_entity *csi = to_rvin_graph_entity(asd);

	v4l2_set_subdev_hostdata(subdev, vin);

	mutex_lock(&vin->group->lock);
	vin_dbg(vin, "Bound CSI-2 %s\n", subdev->name);
	csi->subdev = subdev;
	mutex_unlock(&vin->group->lock);

	return 0;
}

static struct device_node *rvin_group_get_remote(struct rvin_dev *vin,
						 struct device_node *node)
{
	struct device_node *np;

	np = of_graph_get_remote_port_parent(node);
	if (!np) {
		vin_err(vin, "Remote not found %s\n", of_node_full_name(node));
		return NULL;
	}

	/* Not all remotes are available, this is OK */
	if (!of_device_is_available(np)) {
		vin_dbg(vin, "Remote %s is not available\n",
			of_node_full_name(np));
		of_node_put(np);
		return NULL;
	}

	return np;
}

/* group lock should be held when calling this function */
static int rvin_group_graph_parse(struct rvin_dev *vin, struct device_node *np)
{
	int i, id, ret;

	/* Read VIN id from DT */
	id = rvin_group_read_id(vin, np);
	if (id < 0)
		return id;

	/* Check if VIN is already handled */
	if (vin->group->mask & BIT(id))
		return 0;

	vin->group->mask |= BIT(id);

	vin_dbg(vin, "Handling VIN%d\n", id);

	/* Parse all enpoints for CSI-2 and VIN nodes */
	for (i = 0; i < RVIN_CSI_MAX; i++) {
		struct device_node *ep, *csi, *remote;

		/* Check if instance is connected to the CSI-2 */
		ep = of_graph_get_endpoint_by_regs(np, 1, i);
		if (!ep) {
			vin_dbg(vin, "VIN%d: ep %d not connected\n", id, i);
			continue;
		}

		if (vin->group->csi[i].asd.match.fwnode.fwnode) {
			of_node_put(ep);
			vin_dbg(vin, "VIN%d: ep %d already handled\n", id, i);
			continue;
		}

		csi = rvin_group_get_remote(vin, ep);
		of_node_put(ep);
		if (!csi)
			continue;

		vin->group->csi[i].asd.match.fwnode.fwnode =
			of_fwnode_handle(csi);
		vin->group->csi[i].asd.match_type = V4L2_ASYNC_MATCH_FWNODE;

		vin_dbg(vin, "VIN%d ep: %d handled CSI-2 %s\n", id, i,
			of_node_full_name(csi));

		/* Parse the CSI-2 for all VIN nodes connected to it */
		ep = NULL;
		while (1) {
			ep = of_graph_get_next_endpoint(csi, ep);
			if (!ep)
				break;

			remote = rvin_group_get_remote(vin, ep);
			if (!remote)
				continue;

			if (of_match_node(vin->dev->driver->of_match_table,
					  remote)) {
				ret = rvin_group_graph_parse(vin, remote);
				if (ret)
					return ret;

			}
		}
	}

	return 0;
}

static int rvin_group_graph_register(struct rvin_dev *vin)
{
	struct v4l2_async_subdev **subdevs = NULL;
	int i, n, ret, count = 0;

	mutex_lock(&vin->group->lock);

	/* Count how many CSI-2 nodes found */
	for (i = 0; i < RVIN_CSI_MAX; i++)
		if (vin->group->csi[i].asd.match.fwnode.fwnode)
			count++;

	if (!count) {
		mutex_unlock(&vin->group->lock);
		return 0;
	}

	/* Allocate and setup list of subdevices for the notifier */
	subdevs = devm_kzalloc(vin->dev, sizeof(*subdevs) * count, GFP_KERNEL);
	if (subdevs == NULL) {
		mutex_unlock(&vin->group->lock);
		return -ENOMEM;
	}

	n = 0;
	for (i = 0; i < RVIN_CSI_MAX; i++)
		if (vin->group->csi[i].asd.match.fwnode.fwnode)
			subdevs[n++] = &vin->group->csi[i].asd;

	vin_dbg(vin, "Claimed %d subdevices for group\n", count);

	vin->notifier.num_subdevs = count;
	vin->notifier.subdevs = subdevs;
	vin->notifier.bound = rvin_group_notify_bound;
	vin->notifier.unbind = rvin_group_notify_unbind;
	vin->notifier.complete = rvin_group_notify_complete;

	mutex_unlock(&vin->group->lock);

	ret = v4l2_async_notifier_register(&vin->v4l2_dev, &vin->notifier);
	if (ret < 0)
		vin_err(vin, "Notifier registration failed\n");

	return ret;
}

static int rvin_group_init(struct rvin_dev *vin)
{
	int i, ret, count_mask, count_vin = 0;

	ret = rvin_group_allocate(vin);
	if (ret)
		return ret;

	/* All our sources are CSI-2 */
	vin->mbus_cfg.type = V4L2_MBUS_CSI2;
	vin->mbus_cfg.flags = 0;

	vin->pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&vin->vdev.entity, 1, &vin->pad);
	if (ret)
		goto error_group;

	/*
	 * Check number of registered VIN in group against the group mask.
	 * If the mask is empty DT have not yet been parsed and if the
	 * count match all VINs are registered and it's safe to register
	 * the async notifier
	 */
	mutex_lock(&vin->group->lock);

	if (!vin->group->mask) {
		ret = rvin_group_graph_parse(vin, vin->dev->of_node);
		if (ret) {
			mutex_unlock(&vin->group->lock);
			goto error_group;
		}
	}

	for (i = 0; i < RCAR_VIN_NUM; i++)
		if (vin->group->vin[i])
			count_vin++;

	count_mask = hweight_long(vin->group->mask);

	mutex_unlock(&vin->group->lock);

	ret = rvin_v4l2_probe(vin);
	if (ret)
		goto error_group;

	if (count_vin == count_mask) {
		ret = rvin_group_graph_register(vin);
		if (ret)
			goto error_vdev;
	}

	return 0;

error_vdev:
	rvin_v4l2_remove(vin);
error_group:
	rvin_group_delete(vin);

	return ret;
}

/* -----------------------------------------------------------------------------
 * Platform Device Driver
 */

static const struct rvin_info rcar_info_h1 = {
	.chip = RCAR_H1,
	.use_mc = false,
	.max_width = 2048,
	.max_height = 2048,
};

static const struct rvin_info rcar_info_m1 = {
	.chip = RCAR_M1,
	.use_mc = false,
	.max_width = 2048,
	.max_height = 2048,
};

static const struct rvin_info rcar_info_gen2 = {
	.chip = RCAR_GEN2,
	.use_mc = false,
	.max_width = 2048,
	.max_height = 2048,
};

static const struct of_device_id rvin_of_id_table[] = {
	{
		.compatible = "renesas,vin-r8a7794",
		.data = &rcar_info_gen2,
	},
	{
		.compatible = "renesas,vin-r8a7793",
		.data = &rcar_info_gen2,
	},
	{
		.compatible = "renesas,vin-r8a7791",
		.data = &rcar_info_gen2,
	},
	{
		.compatible = "renesas,vin-r8a7790",
		.data = &rcar_info_gen2,
	},
	{
		.compatible = "renesas,vin-r8a7779",
		.data = &rcar_info_h1,
	},
	{
		.compatible = "renesas,vin-r8a7778",
		.data = &rcar_info_m1,
	},
	{
		.compatible = "renesas,rcar-gen2-vin",
		.data = &rcar_info_gen2,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, rvin_of_id_table);

static int rcar_vin_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct rvin_dev *vin;
	struct resource *mem;
	int irq, ret;

	vin = devm_kzalloc(&pdev->dev, sizeof(*vin), GFP_KERNEL);
	if (!vin)
		return -ENOMEM;

	match = of_match_device(of_match_ptr(rvin_of_id_table), &pdev->dev);
	if (!match)
		return -ENODEV;

	vin->dev = &pdev->dev;
	vin->info = match->data;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (mem == NULL)
		return -EINVAL;

	vin->base = devm_ioremap_resource(vin->dev, mem);
	if (IS_ERR(vin->base))
		return PTR_ERR(vin->base);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = rvin_dma_probe(vin, irq);
	if (ret)
		return ret;

	if (vin->info->use_mc)
		ret = rvin_group_init(vin);
	else
		ret = rvin_digital_graph_init(vin);
	if (ret < 0)
		goto error;

	pm_suspend_ignore_children(&pdev->dev, true);
	pm_runtime_enable(&pdev->dev);

	platform_set_drvdata(pdev, vin);

	return 0;
error:
	rvin_dma_remove(vin);

	return ret;
}

static int rcar_vin_remove(struct platform_device *pdev)
{
	struct rvin_dev *vin = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);

	v4l2_async_notifier_unregister(&vin->notifier);

	/* Checks internaly if handlers have been init or not */
	if (!vin->info->use_mc)
		v4l2_ctrl_handler_free(&vin->ctrl_handler);

	rvin_v4l2_remove(vin);

	if (vin->info->use_mc)
		rvin_group_delete(vin);

	rvin_dma_remove(vin);

	return 0;
}

static struct platform_driver rcar_vin_driver = {
	.driver = {
		.name = "rcar-vin",
		.of_match_table = rvin_of_id_table,
	},
	.probe = rcar_vin_probe,
	.remove = rcar_vin_remove,
};

module_platform_driver(rcar_vin_driver);

MODULE_AUTHOR("Niklas Söderlund <niklas.soderlund@ragnatech.se>");
MODULE_DESCRIPTION("Renesas R-Car VIN camera host driver");
MODULE_LICENSE("GPL v2");
