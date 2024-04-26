// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2015 - 2024 Intel Corporation

#include <linux/module.h>

#include "ipu.h"
#include "ipu-cpd.h"

/* $CPD */
#define CPD_HDR_MARK		0x44504324

/* Maximum size is 4K DWORDs or 16KB */
#define MAX_MANIFEST_SIZE	(4 * 1024 * sizeof(u32))

#define CPD_MANIFEST_IDX	0
#define CPD_BINARY_START_IDX	1
#define CPD_METADATA_START_IDX	2
#define CPD_BINARY_NUM		2 /* ISYS + PSYS */
/*
 * Entries include:
 * 1 manifest entry.
 * 1 metadata entry for each sub system(ISYS and PSYS).
 * 1 binary entry for each sub system(ISYS and PSYS).
 */
#define CPD_ENTRY_NUM		(CPD_BINARY_NUM * 2 + 1)

#define CPD_METADATA_ATTR	0xa
#define CPD_METADATA_IPL	0x1c
#define ONLINE_METADATA_SIZE	128
#define ONLINE_METADATA_LINES	6

static inline struct ipu_cpd_ent *ipu_cpd_get_entry(const void *cpd, int idx)
{
	const struct ipu_cpd_hdr *cpd_hdr = cpd;

	return ((struct ipu_cpd_ent *)((u8 *)cpd + cpd_hdr->hdr_len)) + idx;
}

#define ipu_cpd_get_manifest(cpd) ipu_cpd_get_entry(cpd, 0)

static struct ipu_cpd_metadata *ipu_cpd_get_metadata(const void *cpd, int idx)
{
	struct ipu_cpd_ent *cpd_ent =
		ipu_cpd_get_entry(cpd, CPD_METADATA_START_IDX + idx * 2);

	return (struct ipu_cpd_metadata *)((u8 *)cpd + cpd_ent->offset);
}

static int ipu_cpd_validate_cpd(struct ipu_device *isp,
				const void *cpd, unsigned long data_size)
{
	const struct ipu_cpd_hdr *cpd_hdr = cpd;
	struct ipu_cpd_ent *ent;
	unsigned int i;
	u8 len;

	len = cpd_hdr->hdr_len;

	/* Ensure cpd hdr is within moduledata */
	if (data_size < len) {
		dev_err(&isp->pdev->dev, "Invalid CPD moduledata size\n");
		return -EINVAL;
	}

	/* Check for CPD file marker */
	if (cpd_hdr->hdr_mark != CPD_HDR_MARK) {
		dev_err(&isp->pdev->dev, "Invalid CPD header marker\n");
		return -EINVAL;
	}

	/* Sanity check for CPD entry header */
	if (cpd_hdr->ent_cnt != CPD_ENTRY_NUM) {
		dev_err(&isp->pdev->dev, "Invalid CPD entry number %d\n",
			cpd_hdr->ent_cnt);
		return -EINVAL;
	}
	if ((data_size - len) / sizeof(*ent) < cpd_hdr->ent_cnt) {
		dev_err(&isp->pdev->dev, "Invalid CPD entry headers\n");
		return -EINVAL;
	}

	/* Ensure that all entries are within moduledata */
	ent = (struct ipu_cpd_ent *)(((u8 *)cpd_hdr) + len);
	for (i = 0; i < cpd_hdr->ent_cnt; i++, ent++) {
		if (data_size < ent->offset ||
		    data_size - ent->offset < ent->len) {
			dev_err(&isp->pdev->dev, "Invalid CPD entry %d\n", i);
			return -EINVAL;
		}
	}

	return 0;
}

static int ipu_cpd_validate_metadata(struct ipu_device *isp,
				     const void *cpd, int idx)
{
	const struct ipu_cpd_ent *cpd_ent =
		ipu_cpd_get_entry(cpd, CPD_METADATA_START_IDX + idx * 2);
	const struct ipu_cpd_metadata *metadata =
		ipu_cpd_get_metadata(cpd, idx);

	/* Sanity check for metadata size */
	if (cpd_ent->len != sizeof(struct ipu_cpd_metadata)) {
		dev_err(&isp->pdev->dev, "Invalid metadata size\n");
		return -EINVAL;
	}

	/* Validate type and length of metadata sections */
	if (metadata->attr.hdr.type != CPD_METADATA_ATTR) {
		dev_err(&isp->pdev->dev,
			"Invalid metadata attr type (%d)\n",
			metadata->attr.hdr.type);
		return -EINVAL;
	}
	if (metadata->attr.hdr.len != sizeof(struct ipu_cpd_metadata_attr)) {
		dev_err(&isp->pdev->dev,
			"Invalid metadata attr size (%d)\n",
			metadata->attr.hdr.len);
		return -EINVAL;
	}
	if (metadata->ipl.hdr.type != CPD_METADATA_IPL) {
		dev_err(&isp->pdev->dev,
			"Invalid metadata ipl type (%d)\n",
			metadata->ipl.hdr.type);
		return -EINVAL;
	}
	if (metadata->ipl.hdr.len != sizeof(struct ipu_cpd_metadata_ipl)) {
		dev_err(&isp->pdev->dev,
			"Invalid metadata ipl size (%d)\n",
			metadata->ipl.hdr.len);
		return -EINVAL;
	}

	return 0;
}

int ipu_cpd_validate_cpd_file(struct ipu_device *isp, const void *cpd_file,
			      unsigned long cpd_file_size)
{
	struct ipu_cpd_ent *ent;
	unsigned int i;
	int ret;
	char *buf;

	ret = ipu_cpd_validate_cpd(isp, cpd_file, cpd_file_size);
	if (ret) {
		dev_err(&isp->pdev->dev, "Invalid CPD in file\n");
		return -EINVAL;
	}

	/* Sanity check for manifest size */
	ent = ipu_cpd_get_manifest(cpd_file);
	if (ent->len > MAX_MANIFEST_SIZE) {
		dev_err(&isp->pdev->dev, "Invalid manifest size\n");
		return -EINVAL;
	}

	/* Validate metadata */
	for (i = 0; i < CPD_BINARY_NUM; i++) {
		ret = ipu_cpd_validate_metadata(isp, cpd_file, i);
		if (ret) {
			dev_err(&isp->pdev->dev, "Invalid metadata%d\n", i);
			return ret;
		}
	}

	/* Get fw binary version. */
	buf = kmalloc(ONLINE_METADATA_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	for (i = 0; i < CPD_BINARY_NUM; i++) {
		char *lines[ONLINE_METADATA_LINES];
		char *info = buf;
		unsigned int l;

		ent = ipu_cpd_get_entry(cpd_file,
					CPD_BINARY_START_IDX + i * 2);
		memcpy(info, (u8 *)cpd_file + ent->offset + ent->len -
		       ONLINE_METADATA_SIZE, ONLINE_METADATA_SIZE);
		for (l = 0; l < ONLINE_METADATA_LINES; l++) {
			lines[l] = strsep((char **)&info, "\n");
			if (!lines[l])
				break;
		}
		if (l < ONLINE_METADATA_LINES) {
			dev_err(&isp->pdev->dev,
				"Failed to parse fw binary%d info.\n", i);
			continue;
		}
		dev_info(&isp->pdev->dev, "FW binary%d info:\n", i);
		dev_info(&isp->pdev->dev, "Name: %s\n", lines[1]);
		dev_info(&isp->pdev->dev, "Version: %s\n", lines[2]);
		dev_info(&isp->pdev->dev, "Timestamp: %s\n", lines[3]);
		dev_info(&isp->pdev->dev, "Commit: %s\n", lines[4]);
	}
	kfree(buf);

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_cpd_validate_cpd_file);

int ipu_cpd_copy_binary(const void *cpd, const char *name,
			void *code_region, u32 *entry)
{
	unsigned int i;

	for (i = 0; i < CPD_BINARY_NUM; i++) {
		const struct ipu_cpd_ent *binary =
			ipu_cpd_get_entry(cpd, CPD_BINARY_START_IDX + i * 2);
		const struct ipu_cpd_metadata *metadata =
			ipu_cpd_get_metadata(cpd, i);

		if (!strncmp(binary->name, name, sizeof(binary->name))) {
			memcpy(code_region + metadata->ipl.param[0],
			       cpd + binary->offset, binary->len);
			*entry = metadata->ipl.param[2];
			return 0;
		}
	}
	return -ENOENT;
}
EXPORT_SYMBOL_GPL(ipu_cpd_copy_binary);
