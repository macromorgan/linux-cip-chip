#include <linux/slab.h>
#include <linux/crc32.h>
#include "ubi.h"

static void consolidation_unlock(struct ubi_device *ubi,
				 struct ubi_leb_desc *clebs)
{
	int i;

	for (i = 0; i < ubi->lebs_per_cpeb; i++)
		ubi_eba_leb_write_unlock(ubi, clebs[i].vol_id, clebs[i].lnum);
}

static int find_consolidable_lebs(struct ubi_device *ubi,
				  struct ubi_leb_desc *clebs,
				  struct ubi_volume **vols)
{
	struct ubi_full_leb *fleb;
	LIST_HEAD(found);
	int i, err = 0, max_retries = ubi->full_count;

	spin_lock(&ubi->full_lock);
	if (ubi->full_count < ubi->lebs_per_cpeb)
		err = -EAGAIN;
	spin_unlock(&ubi->full_lock);
	if (err)
		return err;

	for (i = 0; i < ubi->lebs_per_cpeb;) {
retry:
		spin_lock(&ubi->full_lock);
		fleb = list_first_entry_or_null(&ubi->full,
						struct ubi_full_leb, node);
		if (fleb)
			clebs[i] = fleb->desc;
		spin_unlock(&ubi->full_lock);

		if (!fleb) {
			err = -EAGAIN;
			goto err;
		}

		err = ubi_eba_leb_write_trylock(ubi, clebs[i].vol_id, clebs[i].lnum);
		if (err) {
			if (err == 1) {
				/* First list entry is already locked, move it to the end and retry. */
				spin_lock(&ubi->full_lock);
				fleb = list_first_entry_or_null(&ubi->full,
								struct ubi_full_leb, node);
				if (fleb && !memcmp(&fleb->desc, &clebs[i], sizeof(*clebs))) {
					list_del(&fleb->node);
					list_add_tail(&fleb->node, &ubi->full);
				}

				spin_unlock(&ubi->full_lock);

				if (--max_retries > 0)
					goto retry;

				err = -EAGAIN;
			}
			goto err;
		}

		spin_lock(&ubi->full_lock);
		fleb = list_first_entry_or_null(&ubi->full,
						struct ubi_full_leb, node);
		if (fleb && !memcmp(&fleb->desc, &clebs[i], sizeof(*clebs))) {
			list_del_init(&fleb->node);
			list_add_tail(&fleb->node, &found);
			ubi->full_count--;
		} else {
			/*
			 * The LEB has been unmapped while we were trying to
			 * acquire its lock: drop it.
			 */
			fleb = NULL;
		}
		spin_unlock(&ubi->full_lock);

		/*
		 * The full LEB we were tracking is no longer mapped, search
		 * for another one.
		 */
		if (!fleb) {
			ubi_eba_leb_write_unlock(ubi, clebs[i].vol_id, clebs[i].lnum);
			continue;
		}

		spin_lock(&ubi->volumes_lock);
		vols[i] = ubi->volumes[vol_id2idx(ubi, clebs[i].vol_id)];
		spin_unlock(&ubi->volumes_lock);
		/* volume vanished under us */
		//TODO clarify/document when/why this can happen
		if (!vols[i]) {
			ubi_eba_leb_write_unlock(ubi, clebs[i].vol_id, clebs[i].lnum);
			spin_lock(&ubi->full_lock);
			list_del_init(&fleb->node);
			kfree(fleb);
			spin_unlock(&ubi->full_lock);
			continue;
		}

		i++;
	}

	while(!list_empty(&found)) {
		fleb = list_first_entry(&found, struct ubi_full_leb, node);
		list_del(&fleb->node);
		kfree(fleb);
	}

	ubi_assert(i == ubi->lebs_per_cpeb);

	return 0;

err:
	while(!list_empty(&found)) {
		spin_lock(&ubi->full_lock);
		fleb = list_first_entry(&found, struct ubi_full_leb, node);
		list_del(&fleb->node);
		list_add_tail(&fleb->node, &ubi->full);
		ubi->full_count++;
		spin_unlock(&ubi->full_lock);
		ubi_eba_leb_write_unlock(ubi, fleb->desc.vol_id, fleb->desc.lnum);
	}

	return err;
}

static int leb_pos(struct ubi_volume *vol, int lnum)
{
	struct ubi_device *ubi = vol->ubi;
	int pnum = vol->eba_tbl[lnum];
	struct ubi_leb_desc *clebs;
	int lpos;

	ubi_assert(pnum >= 0);
	ubi_assert(ubi->consolidated);

	clebs = ubi->consolidated[pnum];
	if (!clebs)
		return 0;

	for (lpos = 0; lpos < ubi->lebs_per_cpeb; lpos++) {
		if (clebs[lpos].vol_id == vol->vol_id &&
		    clebs[lpos].lnum == lnum)
			break;
	}

	ubi_assert(lpos < ubi->lebs_per_cpeb);

	return lpos;
}

static int consolidate_lebs(struct ubi_device *ubi)
{
	int i, pnum, offset = ubi->leb_start, err = 0;
	struct ubi_vid_hdr *vid_hdrs;
	struct ubi_leb_desc *clebs = NULL, *new_clebs = NULL;
	struct ubi_volume **vols = NULL;
	int *opnums = NULL;

	if (!ubi_conso_consolidation_needed(ubi))
		return 0;

	vols = kzalloc(sizeof(*vols) * ubi->lebs_per_cpeb, GFP_KERNEL);
	if (!vols)
		return -ENOMEM;

	opnums = kzalloc(sizeof(*opnums) * ubi->lebs_per_cpeb, GFP_KERNEL);
	if (!opnums) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	clebs = kzalloc(sizeof(*clebs) * ubi->lebs_per_cpeb, GFP_KERNEL);
	if (!clebs) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	new_clebs = kzalloc(sizeof(*clebs) * ubi->lebs_per_cpeb, GFP_KERNEL);
	if (!new_clebs) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	err = find_consolidable_lebs(ubi, clebs, vols);
	if (err)
		goto err_free_mem;

	memcpy(new_clebs, clebs, sizeof(*clebs) * ubi->lebs_per_cpeb);

	mutex_lock(&ubi->buf_mutex);

	pnum = ubi_wl_get_peb(ubi, true, 0);
	if (pnum < 0) {
		err = pnum;
		//TODO cleanup exit path
		mutex_unlock(&ubi->buf_mutex);
		up_read(&ubi->fm_eba_sem);
		goto err_unlock_lebs;
	}

	memset(ubi->peb_buf, 0, ubi->peb_size);
	vid_hdrs = ubi->peb_buf + ubi->vid_hdr_aloffset + ubi->vid_hdr_shift;

	for (i = 0; i < ubi->lebs_per_cpeb; i++) {
		int vol_id = clebs[i].vol_id, lnum = clebs[i].lnum, lpos;
		void *buf = ubi->peb_buf + offset;
		struct ubi_volume *vol = vols[i];
		int spnum;
		int data_size;
		u32 crc;

		spnum = vol->eba_tbl[lnum];
		lpos = leb_pos(vol, lnum);

		/* we raced against leb unmap */
		if (spnum == UBI_LEB_UNMAPPED) {
			//TODO: should be fixed now and no longer trigger.
			ubi_assert(0);
			err = 0;
			goto err_unlock_fm_eba;
		}

		opnums[i] = spnum;

		ubi_assert(offset + ubi->leb_size < ubi->peb_size);
		ubi_assert(lpos >= 0 && lpos < ubi->lebs_per_cpeb);

		if (!ubi->consolidated[spnum])
			err = ubi_io_read(ubi, buf, spnum, ubi->leb_start,
					  ubi->leb_size);
		else
			err = ubi_io_raw_read(ubi, buf, spnum,
					      ubi->leb_start +
					      (lpos * ubi->leb_size),
					      ubi->leb_size);

		if (err && err != UBI_IO_BITFLIPS)
			goto err_unlock_fm_eba;

		if (vol->vol_type == UBI_DYNAMIC_VOLUME) {
			data_size = ubi->leb_size - vol->data_pad;
			vid_hdrs[i].vol_type = UBI_VID_DYNAMIC;
		} else {
			int nvidh = ubi->lebs_per_cpeb;
			struct ubi_vid_hdr *vh;

			vh = ubi_zalloc_vid_hdr(ubi, GFP_NOFS);
			if (!vh) {
				err = -ENOMEM;
				goto err_unlock_fm_eba;
			}

			err = ubi_io_read_vid_hdrs(ubi, spnum, vh, &nvidh, 0);
			if (err && err != UBI_IO_BITFLIPS) {
				ubi_free_vid_hdr(ubi, vh);
				goto err_unlock_fm_eba;
			}

			data_size = be32_to_cpu(vh[lpos].data_size);
			vid_hdrs[i].vol_type = UBI_VID_STATIC;
			vid_hdrs[i].used_ebs = cpu_to_be32(vol->used_ebs);
			ubi_free_vid_hdr(ubi, vh);
		}

		vid_hdrs[i].data_pad = cpu_to_be32(vol->data_pad);
		vid_hdrs[i].sqnum = cpu_to_be64(ubi_next_sqnum(ubi));
		vid_hdrs[i].vol_id = cpu_to_be32(vol_id);
		vid_hdrs[i].lnum = cpu_to_be32(lnum);
		vid_hdrs[i].compat = ubi_get_compat(ubi, vol_id);
		vid_hdrs[i].data_size = cpu_to_be32(data_size);
		vid_hdrs[i].copy_flag = 1;
		crc = crc32(UBI_CRC32_INIT, buf, data_size);
		vid_hdrs[i].data_crc = cpu_to_be32(crc);
		offset += ubi->leb_size;
	}

	/*
	 * Pad remaining pages with zeros to prevent problem on some MLC chip
	 * that expect the whole block to be programmed in order to work
	 * reliably (some Hynix chips are impacted).
	 */
	memset(ubi->peb_buf + offset, 0, ubi->peb_size - offset);

	err = ubi_io_write_vid_hdrs(ubi, pnum, vid_hdrs, ubi->lebs_per_cpeb);
	if (err) {
		ubi_warn(ubi, "failed to write VID headers to PEB %d",
			 pnum);
		goto err_unlock_lebs;
	}

	err = ubi_io_raw_write(ubi, ubi->peb_buf + ubi->leb_start,
			       pnum, ubi->leb_start,
			       ubi->peb_size - ubi->leb_start);
	if (err) {
		ubi_warn(ubi, "failed to write %d bytes of data to PEB %d",
			 ubi->peb_size - ubi->leb_start, pnum);
		goto err_unlock_fm_eba;
	}

	for (i = 0; i < ubi->lebs_per_cpeb; i++) {
		struct ubi_volume *vol = vols[i];
		int lnum = clebs[i].lnum;

		vol->eba_tbl[lnum] = pnum;

		/*
		 * The LEB is still referenced, set opnums[i] to -1 to prevent
		 * its erasure.
		 */
		if (!ubi_conso_invalidate_leb(ubi, opnums[i], clebs[i].vol_id,
					      lnum))
			opnums[i] = -1;
	}
	ubi->consolidated[pnum] = new_clebs;

	up_read(&ubi->fm_eba_sem);
	mutex_unlock(&ubi->buf_mutex);
	consolidation_unlock(ubi, clebs);

	for (i = 0; i < ubi->lebs_per_cpeb; i++) {
		//TODO set torture if needed
		/*
		 * Only release the PEB if it's not referenced by
		 * anyone else.
		 */
		if (opnums[i] >= 0)
			ubi_wl_put_peb(ubi, opnums[i], 0);
	}

	kfree(clebs);
	kfree(opnums);
	kfree(vols);

	return 0;

err_unlock_fm_eba:
	mutex_unlock(&ubi->buf_mutex);
	up_read(&ubi->fm_eba_sem);

	for (i = 0; i < ubi->lebs_per_cpeb; i++)
		ubi_conso_add_full_leb(ubi, clebs[i].vol_id, clebs[i].lnum);

	ubi_wl_put_peb(ubi, pnum, 0);
err_unlock_lebs:
	consolidation_unlock(ubi, clebs);
err_free_mem:
	kfree(new_clebs);
	kfree(clebs);
	kfree(opnums);
	kfree(vols);

	return err;
}

static int consolidation_worker(struct ubi_device *ubi,
				struct ubi_work *wrk,
				int shutdown)
{
	int ret;

	if (shutdown)
		return 0;

	ret = consolidate_lebs(ubi);
	if (ret == -EAGAIN)
		ret = 0;

	ubi->conso_scheduled = 0;
	smp_wmb();

	if (ubi_conso_consolidation_needed(ubi))
		ubi_conso_schedule(ubi);

	return ret;
}

static bool consolidation_possible(struct ubi_device *ubi)
{
	if (ubi->lebs_per_cpeb < 2)
		return false;

	if (ubi->full_count < ubi->lebs_per_cpeb)
		return false;

	/*
	 * Do not even try to consolidate if we don't have enough free PEBs.
	 */
	if (ubi->free_count < UBI_CONSO_RESERVED_PEBS)
		return false;

	return true;
}

bool ubi_conso_consolidation_needed(struct ubi_device *ubi)
{
	if (!consolidation_possible(ubi))
		return false;

	if (ubi_dbg_force_leb_consolidation(ubi))
		return true;

	return ubi->free_count - ubi->beb_rsvd_pebs <=
	       ubi->consolidation_threshold;
}

void ubi_conso_schedule(struct ubi_device *ubi)
{
	struct ubi_work *wrk;

	if (ubi->conso_scheduled)
		return;

	wrk = ubi_alloc_work(ubi);
	if (wrk) {
		ubi->conso_scheduled = 1;
		smp_wmb();

		wrk->func = &consolidation_worker;
		INIT_LIST_HEAD(&wrk->list);
		ubi_schedule_work(ubi, wrk);
	} else
		BUG();
}

int ubi_conso_sync(struct ubi_device *ubi)
{
	int ret = -ENOMEM;

	struct ubi_work *wrk = ubi_alloc_work(ubi);

	if (wrk) {
		wrk->func = &consolidation_worker;
		ret = ubi_schedule_work_sync(ubi, wrk);
	}

	return ret;
}

void ubi_eba_consolidate(struct ubi_device *ubi)
{
	if (consolidation_possible(ubi))
		ubi_conso_schedule(ubi);
}

bool ubi_conso_remove_full_leb(struct ubi_device *ubi, int vol_id, int lnum)
{
	struct ubi_full_leb *fleb;
	bool full = false;

	spin_lock(&ubi->full_lock);
	list_for_each_entry(fleb, &ubi->full, node) {
		if (fleb->desc.lnum == lnum && fleb->desc.vol_id == vol_id) {
			ubi->full_count--;
			list_del(&fleb->node);
			kfree(fleb);
			full = true;
			break;
		}
	}
	spin_unlock(&ubi->full_lock);

	return full;
}

struct ubi_leb_desc *
ubi_conso_get_consolidated(struct ubi_device *ubi, int pnum)
{
	if (ubi->consolidated)
		return ubi->consolidated[pnum];

	return NULL;
}

int ubi_conso_add_full_leb(struct ubi_device *ubi, int vol_id, int lnum)
{
	struct ubi_full_leb *fleb;

	/*
	 * We don't track full LEBs if we don't need to (which is the case
	 * when UBI does not need or does not support LEB consolidation).
	 */
	if (!ubi->consolidated)
		return 0;

	fleb = kzalloc(sizeof(*fleb), GFP_KERNEL);
	if (!fleb)
		return -ENOMEM;

	fleb->desc.vol_id = vol_id;
	fleb->desc.lnum = lnum;

	spin_lock(&ubi->full_lock);
	list_add_tail(&fleb->node, &ubi->full);
	ubi->full_count++;
	spin_unlock(&ubi->full_lock);

	return 0;
}

bool ubi_conso_invalidate_leb(struct ubi_device *ubi, int pnum, int vol_id,
			      int lnum)
{
	struct ubi_leb_desc *clebs = NULL;
	int i, pos = -1, remaining = 0;

	if (!ubi->consolidated)
		return true;

	clebs = ubi->consolidated[pnum];
	if (!clebs) {
		ubi_conso_remove_full_leb(ubi, vol_id, lnum);
		return true;
	}

	/*
	 * We need to take the lock to avoid doing the same operation twice.
	 */
	mutex_lock(&ubi->conso_lock);
	for (i = 0; i < ubi->lebs_per_cpeb; i++) {
		if (clebs[i].lnum == lnum && clebs[i].vol_id == vol_id) {
			clebs[i].lnum = -1;
			clebs[i].vol_id = -1;
			pos = i;
		} else if (clebs[i].lnum >= 0) {
			remaining++;
		}
	}

	ubi_assert(pos >= 0);

	if (remaining == ubi->lebs_per_cpeb - 1) {
		for (i = 0; i < ubi->lebs_per_cpeb; i++) {
			if (i == pos)
				continue;

			ubi_conso_add_full_leb(ubi, clebs[i].vol_id,
					       clebs[i].lnum);
		}
	} else {
		ubi_conso_remove_full_leb(ubi, vol_id, lnum);

		if (!remaining) {
			ubi->consolidated[pnum] = NULL;
			kfree(clebs);
		}
	}
	mutex_unlock(&ubi->conso_lock);

	return !remaining;
}

int ubi_conso_init(struct ubi_device *ubi)
{
	spin_lock_init(&ubi->full_lock);
	INIT_LIST_HEAD(&ubi->full);
	ubi->full_count = 0;
	ubi->consolidation_threshold = (ubi->avail_pebs + ubi->rsvd_pebs) / 3;
	mutex_init(&ubi->conso_lock);

	if (ubi->consolidation_threshold < ubi->lebs_per_cpeb)
		ubi->consolidation_threshold = ubi->lebs_per_cpeb;

	if (ubi->lebs_per_cpeb == 1)
		return 0;

	if (ubi->avail_pebs < UBI_CONSO_RESERVED_PEBS) {
		ubi_err(ubi, "no enough physical eraseblocks (%d, need %d)",
			ubi->avail_pebs, UBI_CONSO_RESERVED_PEBS);
		if (ubi->corr_peb_count)
			ubi_err(ubi, "%d PEBs are corrupted and not used",
				ubi->corr_peb_count);
		return -ENOSPC;
	}

	ubi->avail_pebs -= UBI_CONSO_RESERVED_PEBS;
	ubi->rsvd_pebs += UBI_CONSO_RESERVED_PEBS;

	return 0;
}

void ubi_conso_close(struct ubi_device *ubi)
{
	struct ubi_full_leb *fleb;

	while(!list_empty(&ubi->full)) {
		fleb = list_first_entry(&ubi->full, struct ubi_full_leb, node);
		list_del(&fleb->node);
		kfree(fleb);
		ubi->full_count--;
	}

	ubi_assert(ubi->full_count == 0);
}
