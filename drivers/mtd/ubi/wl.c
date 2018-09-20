/*
 * Copyright (c) International Business Machines Corp., 2006
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
 * the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Authors: Artem Bityutskiy (Битюцкий Артём), Thomas Gleixner
 */

/*
 * UBI wear-leveling sub-system.
 *
 * This sub-system is responsible for wear-leveling. It works in terms of
 * physical eraseblocks and erase counters and knows nothing about logical
 * eraseblocks, volumes, etc. From this sub-system's perspective all physical
 * eraseblocks are of two types - used and free. Used physical eraseblocks are
 * those that were "get" by the 'ubi_wl_get_peb()' function, and free physical
 * eraseblocks are those that were put by the 'ubi_wl_put_peb()' function.
 *
 * Physical eraseblocks returned by 'ubi_wl_get_peb()' have only erase counter
 * header. The rest of the physical eraseblock contains only %0xFF bytes.
 *
 * When physical eraseblocks are returned to the WL sub-system by means of the
 * 'ubi_wl_put_peb()' function, they are scheduled for erasure. The erasure is
 * done asynchronously in context of the per-UBI device background thread,
 * which is also managed by the WL sub-system.
 *
 * The wear-leveling is ensured by means of moving the contents of used
 * physical eraseblocks with low erase counter to free physical eraseblocks
 * with high erase counter.
 *
 * If the WL sub-system fails to erase a physical eraseblock, it marks it as
 * bad.
 *
 * This sub-system is also responsible for scrubbing. If a bit-flip is detected
 * in a physical eraseblock, it has to be moved. Technically this is the same
 * as moving it for wear-leveling reasons.
 *
 * As it was said, for the UBI sub-system all physical eraseblocks are either
 * "free" or "used". Free eraseblock are kept in the @wl->free RB-tree, while
 * used eraseblocks are kept in @wl->used, @wl->erroneous, or @wl->scrub
 * RB-trees, as well as (temporarily) in the @wl->pq queue.
 *
 * When the WL sub-system returns a physical eraseblock, the physical
 * eraseblock is protected from being moved for some "time". For this reason,
 * the physical eraseblock is not directly moved from the @wl->free tree to the
 * @wl->used tree. There is a protection queue in between where this
 * physical eraseblock is temporarily stored (@wl->pq).
 *
 * All this protection stuff is needed because:
 *  o we don't want to move physical eraseblocks just after we have given them
 *    to the user; instead, we first want to let users fill them up with data;
 *
 *  o there is a chance that the user will put the physical eraseblock very
 *    soon, so it makes sense not to move it for some time, but wait.
 *
 * Physical eraseblocks stay protected only for limited time. But the "time" is
 * measured in erase cycles in this case. This is implemented with help of the
 * protection queue. Eraseblocks are put to the tail of this queue when they
 * are returned by the 'ubi_wl_get_peb()', and eraseblocks are removed from the
 * head of the queue on each erase operation (for any eraseblock). So the
 * length of the queue defines how may (global) erase cycles PEBs are protected.
 *
 * To put it differently, each physical eraseblock has 2 main states: free and
 * used. The former state corresponds to the @wl->free tree. The latter state
 * is split up on several sub-states:
 * o the WL movement is allowed (@wl->used tree);
 * o the WL movement is disallowed (@wl->erroneous) because the PEB is
 *   erroneous - e.g., there was a read error;
 * o the WL movement is temporarily prohibited (@wl->pq queue);
 * o scrubbing is needed (@wl->scrub tree).
 *
 * Depending on the sub-state, wear-leveling entries of the used physical
 * eraseblocks may be kept in one of those structures.
 *
 * Note, in this implementation, we keep a small in-RAM object for each physical
 * eraseblock. This is surely not a scalable solution. But it appears to be good
 * enough for moderately large flashes and it is simple. In future, one may
 * re-work this sub-system and make it more scalable.
 *
 * At the moment this sub-system does not utilize the sequence number, which
 * was introduced relatively recently. But it would be wise to do this because
 * the sequence number of a logical eraseblock characterizes how old is it. For
 * example, when we move a PEB with low erase counter, and we need to pick the
 * target PEB, we pick a PEB with the highest EC if our PEB is "old" and we
 * pick target PEB with an average EC if our PEB is not very "old". This is a
 * room for future re-works of the WL sub-system.
 */

#include <linux/slab.h>
#include <linux/crc32.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include "ubi.h"
#include "wl.h"

/*
 * Maximum difference between two erase counters. If this threshold is
 * exceeded, the WL sub-system starts moving data from used physical
 * eraseblocks with low erase counter to free physical eraseblocks with high
 * erase counter.
 */
#define UBI_WL_THRESHOLD CONFIG_MTD_UBI_WL_THRESHOLD

/*
 * When a physical eraseblock is moved, the WL sub-system has to pick the target
 * physical eraseblock to move to. The simplest way would be just to pick the
 * one with the highest erase counter. But in certain workloads this could lead
 * to an unlimited wear of one or few physical eraseblock. Indeed, imagine a
 * situation when the picked physical eraseblock is constantly erased after the
 * data is written to it. So, we have a constant which limits the highest erase
 * counter of the free physical eraseblock to pick. Namely, the WL sub-system
 * does not pick eraseblocks with erase counter greater than the lowest erase
 * counter plus %WL_FREE_MAX_DIFF.
 */
#define WL_FREE_MAX_DIFF (2*UBI_WL_THRESHOLD)

static int self_check_ec(struct ubi_device *ubi, int pnum, int ec);
static int self_check_in_wl_tree(const struct ubi_device *ubi,
				 struct ubi_wl_entry *e, struct rb_root *root);
static int self_check_in_pq(const struct ubi_device *ubi,
			    struct ubi_wl_entry *e);

/**
 * wl_tree_add - add a wear-leveling entry to a WL RB-tree.
 * @e: the wear-leveling entry to add
 * @root: the root of the tree
 *
 * Note, we use (erase counter, physical eraseblock number) pairs as keys in
 * the @ubi->used and @ubi->free RB-trees.
 */
static void wl_tree_add(struct ubi_wl_entry *e, struct rb_root *root)
{
	struct rb_node **p, *parent = NULL;

	p = &root->rb_node;
	while (*p) {
		struct ubi_wl_entry *e1;

		parent = *p;
		e1 = rb_entry(parent, struct ubi_wl_entry, u.rb);

		if (e->ec < e1->ec)
			p = &(*p)->rb_left;
		else if (e->ec > e1->ec)
			p = &(*p)->rb_right;
		else {
			ubi_assert(e->pnum != e1->pnum);
			if (e->pnum < e1->pnum)
				p = &(*p)->rb_left;
			else
				p = &(*p)->rb_right;
		}
	}

	rb_link_node(&e->u.rb, parent, p);
	rb_insert_color(&e->u.rb, root);
}

/**
 * wl_tree_destroy - destroy a wear-leveling entry.
 * @ubi: UBI device description object
 * @e: the wear-leveling entry to add
 *
 * This function destroys a wear leveling entry and removes
 * the reference from the lookup table.
 */
static void wl_entry_destroy(struct ubi_device *ubi, struct ubi_wl_entry *e)
{
	ubi->lookuptbl[e->pnum] = NULL;
	kmem_cache_free(ubi_wl_entry_slab, e);
}

/**
 * in_wl_tree - check if wear-leveling entry is present in a WL RB-tree.
 * @e: the wear-leveling entry to check
 * @root: the root of the tree
 *
 * This function returns non-zero if @e is in the @root RB-tree and zero if it
 * is not.
 */
static int in_wl_tree(struct ubi_wl_entry *e, struct rb_root *root)
{
	struct rb_node *p;

	p = root->rb_node;
	while (p) {
		struct ubi_wl_entry *e1;

		e1 = rb_entry(p, struct ubi_wl_entry, u.rb);

		if (e->pnum == e1->pnum) {
			ubi_assert(e == e1);
			return 1;
		}

		if (e->ec < e1->ec)
			p = p->rb_left;
		else if (e->ec > e1->ec)
			p = p->rb_right;
		else {
			ubi_assert(e->pnum != e1->pnum);
			if (e->pnum < e1->pnum)
				p = p->rb_left;
			else
				p = p->rb_right;
		}
	}

	return 0;
}

/**
 * in_pq - check if a wear-leveling entry is present in the protection queue.
 * @ubi: UBI device description object
 * @e: the wear-leveling entry to check
 *
 * This function returns non-zero if @e is in the protection queue and zero
 * if it is not.
 */
static inline int in_pq(const struct ubi_device *ubi, struct ubi_wl_entry *e)
{
	struct ubi_wl_entry *p;
	int i;

	for (i = 0; i < UBI_PROT_QUEUE_LEN; ++i)
		list_for_each_entry(p, &ubi->pq[i], u.list)
			if (p == e)
				return 1;

	return 0;
}

/**
 * prot_queue_add - add physical eraseblock to the protection queue.
 * @ubi: UBI device description object
 * @e: the physical eraseblock to add
 *
 * This function adds @e to the tail of the protection queue @ubi->pq, where
 * @e will stay for %UBI_PROT_QUEUE_LEN erase operations and will be
 * temporarily protected from the wear-leveling worker. Note, @wl->lock has to
 * be locked.
 */
static void prot_queue_add(struct ubi_device *ubi, struct ubi_wl_entry *e)
{
	int pq_tail = ubi->pq_head - 1;

	if (pq_tail < 0)
		pq_tail = UBI_PROT_QUEUE_LEN - 1;
	ubi_assert(pq_tail >= 0 && pq_tail < UBI_PROT_QUEUE_LEN);
	list_add_tail(&e->u.list, &ubi->pq[pq_tail]);
	dbg_wl("added PEB %d EC %d to the protection queue", e->pnum, e->ec);
}

/**
 * find_wl_entry - find wear-leveling entry closest to certain erase counter.
 * @ubi: UBI device description object
 * @root: the RB-tree where to look for
 * @diff: maximum possible difference from the smallest erase counter
 *
 * This function looks for a wear leveling entry with erase counter closest to
 * min + @diff, where min is the smallest erase counter.
 */
static struct ubi_wl_entry *find_wl_entry(struct ubi_device *ubi,
					  struct rb_root *root, int diff)
{
	struct rb_node *p;
	struct ubi_wl_entry *e, *prev_e = NULL;
	int max;

	e = rb_entry(rb_first(root), struct ubi_wl_entry, u.rb);
	max = e->ec + diff;

	p = root->rb_node;
	while (p) {
		struct ubi_wl_entry *e1;

		e1 = rb_entry(p, struct ubi_wl_entry, u.rb);
		if (e1->ec >= max)
			p = p->rb_left;
		else {
			p = p->rb_right;
			prev_e = e;
			e = e1;
		}
	}

	/* If no fastmap has been written and this WL entry can be used
	 * as anchor PEB, hold it back and return the second best WL entry
	 * such that fastmap can use the anchor PEB later. */
	if (prev_e && !ubi->fm_disabled &&
	    !ubi->fm && e->pnum < UBI_FM_MAX_START)
		return prev_e;

	return e;
}

/**
 * find_mean_wl_entry - find wear-leveling entry with medium erase counter.
 * @ubi: UBI device description object
 * @root: the RB-tree where to look for
 *
 * This function looks for a wear leveling entry with medium erase counter,
 * but not greater or equivalent than the lowest erase counter plus
 * %WL_FREE_MAX_DIFF/2.
 */
static struct ubi_wl_entry *find_mean_wl_entry(struct ubi_device *ubi,
					       struct rb_root *root)
{
	struct ubi_wl_entry *e, *first, *last;

	ubi_assert(root->rb_node);

	if (!root->rb_node)
		return NULL;

	first = rb_entry(rb_first(root), struct ubi_wl_entry, u.rb);
	last = rb_entry(rb_last(root), struct ubi_wl_entry, u.rb);

	if (last->ec - first->ec < WL_FREE_MAX_DIFF) {
		e = rb_entry(root->rb_node, struct ubi_wl_entry, u.rb);

		/* If no fastmap has been written and this WL entry can be used
		 * as anchor PEB, hold it back and return the second best
		 * WL entry such that fastmap can use the anchor PEB later. */
		e = may_reserve_for_fm(ubi, e, root);
	} else
		e = find_wl_entry(ubi, root, WL_FREE_MAX_DIFF/2);

	return e;
}

/**
 * wl_get_wle - get a mean wl entry to be used by ubi_wl_get_peb() or
 * refill_wl_user_pool().
 * @ubi: UBI device description object
 *
 * This function returns a a wear leveling entry in case of success and
 * NULL in case of failure.
 */
static struct ubi_wl_entry *wl_get_wle(struct ubi_device *ubi)
{
	struct ubi_wl_entry *e;

	e = find_mean_wl_entry(ubi, &ubi->free);
	if (!e) {
		ubi_err(ubi, "no free eraseblocks");
		return NULL;
	}

	self_check_in_wl_tree(ubi, e, &ubi->free);

	/*
	 * Move the physical eraseblock to the protection queue where it will
	 * be protected from being moved for some time.
	 */
	rb_erase(&e->u.rb, &ubi->free);
	ubi->free_count--;
	dbg_wl("PEB %d EC %d", e->pnum, e->ec);

	return e;
}

/**
 * prot_queue_del - remove a physical eraseblock from the protection queue.
 * @ubi: UBI device description object
 * @pnum: the physical eraseblock to remove
 *
 * This function deletes PEB @pnum from the protection queue and returns zero
 * in case of success and %-ENODEV if the PEB was not found.
 */
static int prot_queue_del(struct ubi_device *ubi, int pnum)
{
	struct ubi_wl_entry *e;

	e = ubi->lookuptbl[pnum];
	if (!e)
		return -ENODEV;

	if (self_check_in_pq(ubi, e))
		return -ENODEV;

	list_del(&e->u.list);
	dbg_wl("deleted PEB %d from the protection queue", e->pnum);
	return 0;
}

void ubi_wl_update_rc(struct ubi_device *ubi, int pnum)
{
#ifdef CONFIG_MTD_UBI_READ_COUNTER
	struct ubi_wl_entry *e;

	/*
	 * WL not initialized yet.
	 */
	if (!ubi->lookuptbl)
		return;

	spin_lock(&ubi->wl_lock);
	e = ubi->lookuptbl[pnum];
	if (e)
		e->rc++;
	spin_unlock(&ubi->wl_lock);
#endif
}

static void ubi_wl_clear_rc(struct ubi_wl_entry *e)
{
#ifdef CONFIG_MTD_UBI_READ_COUNTER
	e->rc = 0;
#endif
}

static void ubi_wl_get_rc(struct ubi_wl_entry *e, struct ubi_stats_entry *se)
{
#ifdef CONFIG_MTD_UBI_READ_COUNTER
	se->rc = e->rc;
#else
	se->rc = -1;
#endif
}

static int ubi_wl_fill_stats_entry(struct ubi_device *ubi,
				   struct ubi_stats_entry *se, int pnum)
{
	struct ubi_wl_entry *e;

	spin_lock(&ubi->wl_lock);
	e = ubi->lookuptbl[pnum];
	if (e) {
		se->pnum = pnum;
		se->ec = e->ec;
		ubi_wl_get_rc(e, se);
	}
	spin_unlock(&ubi->wl_lock);

	return e ? 0 : -1;
}

int ubi_wl_report_stats(struct ubi_device *ubi, struct ubi_stats_req *req,
			struct ubi_stats_entry __user *se)
{
	int i, pnum, peb_end, peb_start;
	struct ubi_stats_entry tmp_se;
	size_t write_len;
	int n = 0;

	pnum = req->req_pnum;
	if (pnum != -1) {
		if (pnum < 0 || pnum >= ubi->peb_count)
			return -EINVAL;

		peb_start = pnum;
		peb_end = pnum + 1;
		write_len = sizeof(*se);
	} else {
		peb_start = 0;
		peb_end = ubi->peb_count;
		write_len = sizeof(*se) * ubi->good_peb_count;
	}

	if (write_len > req->req_len - sizeof(*req))
		return -EFAULT;

	if (!access_ok(VERIFY_WRITE, se, req->req_len - sizeof(*req) + write_len))
		return -EFAULT;

	for (i = peb_start; i < peb_end; i++) {
		if (ubi_wl_fill_stats_entry(ubi, &tmp_se, i) == 0) {
			if (__copy_to_user(se, &tmp_se, sizeof(tmp_se)))
				return -EFAULT;

			se++;
			n++;
		}
	}

	return n;
}

/**
 * sync_erase - synchronously erase a physical eraseblock.
 * @ubi: UBI device description object
 * @e: the the physical eraseblock to erase
 * @torture: if the physical eraseblock has to be tortured
 *
 * This function returns zero in case of success and a negative error code in
 * case of failure.
 */
static int sync_erase(struct ubi_device *ubi, struct ubi_wl_entry *e,
		      int torture)
{
	int err;
	struct ubi_ec_hdr *ec_hdr;
	unsigned long long ec = e->ec;

	dbg_wl("erase PEB %d, old EC %llu", e->pnum, ec);

	err = self_check_ec(ubi, e->pnum, e->ec);
	if (err)
		return -EINVAL;

	ec_hdr = kzalloc(ubi->ec_hdr_alsize, GFP_NOFS);
	if (!ec_hdr)
		return -ENOMEM;

	err = ubi_io_sync_erase(ubi, e->pnum, torture);
	if (err < 0)
		goto out_free;

	ubi_wl_clear_rc(e);

	ec += err;
	if (ec > UBI_MAX_ERASECOUNTER) {
		/*
		 * Erase counter overflow. Upgrade UBI and use 64-bit
		 * erase counters internally.
		 */
		ubi_err(ubi, "erase counter overflow at PEB %d, EC %llu",
			e->pnum, ec);
		err = -EINVAL;
		goto out_free;
	}

	dbg_wl("erased PEB %d, new EC %llu", e->pnum, ec);

	ec_hdr->ec = cpu_to_be64(ec);

	err = ubi_io_write_ec_hdr(ubi, e->pnum, ec_hdr);
	if (err)
		goto out_free;

	e->ec = ec;
	spin_lock(&ubi->wl_lock);
	if (e->ec > ubi->max_ec)
		ubi->max_ec = e->ec;
	spin_unlock(&ubi->wl_lock);

out_free:
	kfree(ec_hdr);
	return err;
}

/**
 * serve_prot_queue - check if it is time to stop protecting PEBs.
 * @ubi: UBI device description object
 *
 * This function is called after each erase operation and removes PEBs from the
 * tail of the protection queue. These PEBs have been protected for long enough
 * and should be moved to the used tree.
 */
static void serve_prot_queue(struct ubi_device *ubi)
{
	struct ubi_wl_entry *e, *tmp;
	int count;

	/*
	 * There may be several protected physical eraseblock to remove,
	 * process them all.
	 */
repeat:
	count = 0;
	spin_lock(&ubi->wl_lock);
	list_for_each_entry_safe(e, tmp, &ubi->pq[ubi->pq_head], u.list) {
		dbg_wl("PEB %d EC %d protection over, move to used tree",
			e->pnum, e->ec);

		list_del(&e->u.list);
		wl_tree_add(e, &ubi->used);
		if (count++ > 32) {
			/*
			 * Let's be nice and avoid holding the spinlock for
			 * too long.
			 */
			spin_unlock(&ubi->wl_lock);
			cond_resched();
			goto repeat;
		}
	}

	ubi->pq_head += 1;
	if (ubi->pq_head == UBI_PROT_QUEUE_LEN)
		ubi->pq_head = 0;
	ubi_assert(ubi->pq_head >= 0 && ubi->pq_head < UBI_PROT_QUEUE_LEN);
	spin_unlock(&ubi->wl_lock);
}

static int erase_worker(struct ubi_device *ubi, struct ubi_work *wl_wrk,
			int shutdown);

struct ubi_work *ubi_alloc_erase_work(struct ubi_device *ubi,
				      struct ubi_wl_entry *e, int torture)
{
	struct ubi_work *wl_wrk;

	ubi_assert(e);

	wl_wrk = ubi_alloc_work(ubi);
	if (!wl_wrk)
		return NULL;

	wl_wrk->func = &erase_worker;
	wl_wrk->e = e;
	wl_wrk->torture = torture;

	return wl_wrk;
}

/**
 * prepare_erase_work - prepare an erase work.
 * @ubi: UBI device description object
 * @e: the WL entry of the physical eraseblock to erase
 * @torture: if the physical eraseblock has to be tortured
 *
 * This function returns a struct ubi_work in case of success
 * and an ERR_PTR(%-ENOMEM) in case of failure.
 */
static struct ubi_work *prepare_erase_work(struct ubi_device *ubi,
					   struct ubi_wl_entry *e,
					   int torture)
{
	struct ubi_work *wl_wrk;

	ubi_assert(e);
	ubi_assert(!ubi->consolidated || !ubi->consolidated[e->pnum]);

	wl_wrk = ubi_alloc_erase_work(ubi, e, torture);
	if (!wl_wrk)
		return ERR_PTR(-ENOMEM);

	wl_wrk->func = &erase_worker;
	wl_wrk->e = e;
	wl_wrk->torture = torture;

	return wl_wrk;
}

/**
 * schedule_erase - schedule an erase work.
 * @ubi: UBI device description object
 * @e: the WL entry of the physical eraseblock to erase
 * @torture: if the physical eraseblock has to be tortured
 *
 * This function returns zero in case of success and a %-ENOMEM in case of
 * failure.
 */
static int schedule_erase(struct ubi_device *ubi, struct ubi_wl_entry *e,
			  int torture)
{
	struct ubi_work *wl_wrk;

	dbg_wl("schedule erasure of PEB %d, EC %d, torture %d",
	       e->pnum, e->ec, torture);

	wl_wrk = prepare_erase_work(ubi, e, torture);

	if (IS_ERR(wl_wrk))
		return PTR_ERR(wl_wrk);

	ubi_schedule_work(ubi, wl_wrk);
	return 0;
}

static int __erase_worker(struct ubi_device *ubi, struct ubi_work *wl_wrk);
/**
 * do_sync_erase - run the erase worker synchronously.
 * @ubi: UBI device description object
 * @e: the WL entry of the physical eraseblock to erase
 * @torture: if the physical eraseblock has to be tortured
 *
 */
static int do_sync_erase(struct ubi_device *ubi, struct ubi_wl_entry *e,
			 int torture)
{
	struct ubi_work wl_wrk;

	dbg_wl("sync erase of PEB %i", e->pnum);

	wl_wrk.e = e;
	wl_wrk.torture = torture;

	return __erase_worker(ubi, &wl_wrk);
}

static int ensure_wear_leveling(struct ubi_device *ubi);

/**
 * wear_leveling_worker - wear-leveling worker function.
 * @ubi: UBI device description object
 * @wrk: the work object
 * @shutdown: non-zero if the worker has to free memory and exit
 * because the WL-subsystem is shutting down
 *
 * This function copies a more worn out physical eraseblock to a less worn out
 * one. Returns zero in case of success and a negative error code in case of
 * failure.
 */
static int wear_leveling_worker(struct ubi_device *ubi, struct ubi_work *wrk,
				int shutdown)
{
	int err, scrubbing = 0, torture = 0, protect = 0, erroneous = 0;
#ifdef CONFIG_MTD_UBI_FASTMAP
	int anchor = wrk->anchor;
#endif
	struct ubi_wl_entry *e1, *e2;
	struct ubi_vid_hdr *vid_hdr;
	int dst_leb_clean = 0;
	int nvidh = ubi->lebs_per_cpeb;

	if (shutdown)
		return 0;

	vid_hdr = ubi_zalloc_vid_hdr(ubi, GFP_NOFS);
	if (!vid_hdr)
		return -ENOMEM;

	mutex_lock(&ubi->move_mutex);
	spin_lock(&ubi->wl_lock);
	ubi_assert(!ubi->move_from && !ubi->move_to);
	ubi_assert(!ubi->move_to_put);

	if (!ubi->free.rb_node ||
	    (!ubi->used.rb_node && !ubi->scrub.rb_node)) {
		/*
		 * No free physical eraseblocks? Well, they must be waiting in
		 * the queue to be erased. Cancel movement - it will be
		 * triggered again when a free physical eraseblock appears.
		 *
		 * No used physical eraseblocks? They must be temporarily
		 * protected from being moved. They will be moved to the
		 * @ubi->used tree later and the wear-leveling will be
		 * triggered again.
		 */
		dbg_wl("cancel WL, a list is empty: free %d, used %d",
		       !ubi->free.rb_node, !ubi->used.rb_node);
		goto out_cancel;
	}

#ifdef CONFIG_MTD_UBI_FASTMAP
	/* Check whether we need to produce an anchor PEB */
	if (!anchor)
		anchor = !anchor_pebs_avalible(&ubi->free);

	if (anchor) {
		e1 = find_anchor_wl_entry(&ubi->used);
		if (!e1)
			goto out_cancel;
		e2 = get_peb_for_wl(ubi);
		if (!e2)
			goto out_cancel;

		self_check_in_wl_tree(ubi, e1, &ubi->used);
		rb_erase(&e1->u.rb, &ubi->used);
		dbg_wl("anchor-move PEB %d to PEB %d", e1->pnum, e2->pnum);
	} else if (!ubi->scrub.rb_node) {
#else
	if (!ubi->scrub.rb_node) {
#endif
		/*
		 * Now pick the least worn-out used physical eraseblock and a
		 * highly worn-out free physical eraseblock. If the erase
		 * counters differ much enough, start wear-leveling.
		 */
		e1 = rb_entry(rb_first(&ubi->used), struct ubi_wl_entry, u.rb);
		e2 = get_peb_for_wl(ubi);
		if (!e2)
			goto out_cancel;

		if (!(e2->ec - e1->ec >= UBI_WL_THRESHOLD)) {
			dbg_wl("no WL needed: min used EC %d, max free EC %d",
			       e1->ec, e2->ec);

			/* Give the unused PEB back */
			wl_tree_add(e2, &ubi->free);
			ubi->free_count++;
			goto out_cancel;
		}
		self_check_in_wl_tree(ubi, e1, &ubi->used);
		rb_erase(&e1->u.rb, &ubi->used);
		dbg_wl("move PEB %d EC %d to PEB %d EC %d",
		       e1->pnum, e1->ec, e2->pnum, e2->ec);
	} else {
		/* Perform scrubbing */
		scrubbing = 1;
		e1 = rb_entry(rb_first(&ubi->scrub), struct ubi_wl_entry, u.rb);
		e2 = get_peb_for_wl(ubi);
		if (!e2)
			goto out_cancel;

		self_check_in_wl_tree(ubi, e1, &ubi->scrub);
		rb_erase(&e1->u.rb, &ubi->scrub);
		dbg_wl("scrub PEB %d to PEB %d", e1->pnum, e2->pnum);
	}

	ubi->move_from = e1;
	ubi->move_to = e2;
	spin_unlock(&ubi->wl_lock);

	/*
	 * Now we are going to copy physical eraseblock @e1->pnum to @e2->pnum.
	 * We so far do not know which logical eraseblock our physical
	 * eraseblock (@e1) belongs to. We have to read the volume identifier
	 * header first.
	 *
	 * Note, we are protected from this PEB being unmapped and erased. The
	 * 'ubi_wl_put_peb()' would wait for moving to be finished if the PEB
	 * which is being moved was unmapped.
	 */

	err = ubi_io_read_vid_hdrs(ubi, e1->pnum, vid_hdr, &nvidh, 0);
	if (err && err != UBI_IO_BITFLIPS) {
		if (err == UBI_IO_FF) {
			/*
			 * We are trying to move PEB without a VID header. UBI
			 * always write VID headers shortly after the PEB was
			 * given, so we have a situation when it has not yet
			 * had a chance to write it, because it was preempted.
			 * So add this PEB to the protection queue so far,
			 * because presumably more data will be written there
			 * (including the missing VID header), and then we'll
			 * move it.
			 */
			dbg_wl("PEB %d has no VID header", e1->pnum);
			protect = 1;
			goto out_not_moved;
		} else if (err == UBI_IO_FF_BITFLIPS) {
			/*
			 * The same situation as %UBI_IO_FF, but bit-flips were
			 * detected. It is better to schedule this PEB for
			 * scrubbing.
			 */
			dbg_wl("PEB %d has no VID header but has bit-flips",
			       e1->pnum);
			scrubbing = 1;
			goto out_not_moved;
		}

		ubi_err(ubi, "error %d while reading VID header from PEB %d",
			err, e1->pnum);
		goto out_error;
	}

	if (ubi->consolidated && ubi->consolidated[e1->pnum])
		err = ubi_eba_copy_lebs(ubi, e1->pnum, e2->pnum, vid_hdr, nvidh);
	else
		err = ubi_eba_copy_leb(ubi, e1->pnum, e2->pnum, vid_hdr);

	if (err) {
		if (err == MOVE_CANCEL_RACE) {
			/*
			 * The LEB has not been moved because the volume is
			 * being deleted or the PEB has been put meanwhile. We
			 * should prevent this PEB from being selected for
			 * wear-leveling movement again, so put it to the
			 * protection queue.
			 */
			protect = 1;
			goto out_not_moved;
		}
		if (err == MOVE_RETRY) {
			scrubbing = 1;
			goto out_not_moved;
		}
		if (err == MOVE_TARGET_BITFLIPS || err == MOVE_TARGET_WR_ERR ||
		    err == MOVE_TARGET_RD_ERR) {
			/*
			 * Target PEB had bit-flips or write error - torture it.
			 */
			torture = 1;
			goto out_not_moved;
		}

		if (err == MOVE_SOURCE_RD_ERR) {
			/*
			 * An error happened while reading the source PEB. Do
			 * not switch to R/O mode in this case, and give the
			 * upper layers a possibility to recover from this,
			 * e.g. by unmapping corresponding LEB. Instead, just
			 * put this PEB to the @ubi->erroneous list to prevent
			 * UBI from trying to move it over and over again.
			 */
			if (ubi->erroneous_peb_count > ubi->max_erroneous) {
				ubi_err(ubi, "too many erroneous eraseblocks (%d)",
					ubi->erroneous_peb_count);
				goto out_error;
			}
			erroneous = 1;
			goto out_not_moved;
		}

		if (err < 0)
			goto out_error;

		ubi_assert(0);
	}

	/* The PEB has been successfully moved */
	if (scrubbing)
		ubi_msg(ubi, "scrubbed PEB %d, data moved to PEB %d",
			e1->pnum, e2->pnum);
	ubi_free_vid_hdr(ubi, vid_hdr);

	spin_lock(&ubi->wl_lock);
	if (!ubi->move_to_put) {
		wl_tree_add(e2, &ubi->used);
		e2 = NULL;
	}
	ubi->move_from = ubi->move_to = NULL;
	ubi->move_to_put = ubi->wl_scheduled = 0;
	spin_unlock(&ubi->wl_lock);

	err = do_sync_erase(ubi, e1, 0);
	if (err) {
		if (e2)
			wl_entry_destroy(ubi, e2);
		goto out_ro;
	}

	if (e2) {
		/*
		 * Well, the target PEB was put meanwhile, schedule it for
		 * erasure.
		 */
		dbg_wl("PEB %d was put meanwhile, erase",
		       e2->pnum);
		err = do_sync_erase(ubi, e2, 0);
		if (err)
			goto out_ro;
	}

	dbg_wl("done");
	mutex_unlock(&ubi->move_mutex);
	return 0;

	/*
	 * For some reasons the LEB was not moved, might be an error, might be
	 * something else. @e1 was not changed, so return it back. @e2 might
	 * have been changed, schedule it for erasure.
	 */
out_not_moved:
	dbg_wl("cancel moving PEB %d to PEB %d (%d)", e1->pnum, e2->pnum, err);
	spin_lock(&ubi->wl_lock);
	if (protect)
		prot_queue_add(ubi, e1);
	else if (erroneous) {
		wl_tree_add(e1, &ubi->erroneous);
		ubi->erroneous_peb_count += 1;
	} else if (scrubbing)
		wl_tree_add(e1, &ubi->scrub);
	else
		wl_tree_add(e1, &ubi->used);
	ubi_assert(!ubi->move_to_put);
	ubi->move_from = ubi->move_to = NULL;
	ubi->wl_scheduled = 0;
	spin_unlock(&ubi->wl_lock);

	ubi_free_vid_hdr(ubi, vid_hdr);
	if (dst_leb_clean) {
		ensure_wear_leveling(ubi);
	} else {
		err = do_sync_erase(ubi, e2, torture);
		if (err)
			goto out_ro;
	}

	mutex_unlock(&ubi->move_mutex);
	return 0;

out_error:
	ubi_err(ubi, "error %d while moving PEB %d to PEB %d", err, e1->pnum,
		e2->pnum);
	spin_lock(&ubi->wl_lock);
	ubi->move_from = ubi->move_to = NULL;
	ubi->move_to_put = ubi->wl_scheduled = 0;
	spin_unlock(&ubi->wl_lock);

	ubi_free_vid_hdr(ubi, vid_hdr);
	wl_entry_destroy(ubi, e1);
	wl_entry_destroy(ubi, e2);

out_ro:
	ubi_ro_mode(ubi);
	mutex_unlock(&ubi->move_mutex);
	ubi_assert(err != 0);
	return err < 0 ? err : -EIO;

out_cancel:
	ubi->wl_scheduled = 0;
	spin_unlock(&ubi->wl_lock);
	mutex_unlock(&ubi->move_mutex);
	ubi_free_vid_hdr(ubi, vid_hdr);
	return 0;
}

/**
 * ensure_wear_leveling - schedule wear-leveling if it is needed.
 * @ubi: UBI device description object
 *
 * This function checks if it is time to start wear-leveling and schedules it
 * if yes. This function returns zero in case of success and a negative error
 * code in case of failure.
 */
static int ensure_wear_leveling(struct ubi_device *ubi)
{
	int err = 0;
	struct ubi_wl_entry *e1;
	struct ubi_wl_entry *e2;
	struct ubi_work *wrk;

	spin_lock(&ubi->wl_lock);
	if (ubi->wl_scheduled)
		/* Wear-leveling is already in the work queue */
		goto out_unlock;

	/*
	 * If the ubi->scrub tree is not empty, scrubbing is needed, and the
	 * the WL worker has to be scheduled anyway.
	 */
	if (!ubi->scrub.rb_node) {
		if (!ubi->used.rb_node || !ubi->free.rb_node)
			/* No physical eraseblocks - no deal */
			goto out_unlock;

		/*
		 * We schedule wear-leveling only if the difference between the
		 * lowest erase counter of used physical eraseblocks and a high
		 * erase counter of free physical eraseblocks is greater than
		 * %UBI_WL_THRESHOLD.
		 */
		e1 = rb_entry(rb_first(&ubi->used), struct ubi_wl_entry, u.rb);
		e2 = find_wl_entry(ubi, &ubi->free, WL_FREE_MAX_DIFF);

		if (!(e2->ec - e1->ec >= UBI_WL_THRESHOLD))
			goto out_unlock;
		dbg_wl("schedule wear-leveling");
	} else
		dbg_wl("schedule scrubbing");

	ubi->wl_scheduled = 1;
	spin_unlock(&ubi->wl_lock);

	wrk = ubi_alloc_work(ubi);
	if (!wrk) {
		err = -ENOMEM;
		goto out_cancel;
	}

	wrk->anchor = 0;
	wrk->func = &wear_leveling_worker;
	ubi_schedule_work(ubi, wrk);

	return err;

out_cancel:
	spin_lock(&ubi->wl_lock);
	ubi->wl_scheduled = 0;
out_unlock:
	spin_unlock(&ubi->wl_lock);
	return err;
}

/**
 * __erase_worker - physical eraseblock erase worker function.
 * @ubi: UBI device description object
 * @wl_wrk: the work object
 * @shutdown: non-zero if the worker has to free memory and exit
 * because the WL sub-system is shutting down
 *
 * This function erases a physical eraseblock and perform torture testing if
 * needed. It also takes care about marking the physical eraseblock bad if
 * needed. Returns zero in case of success and a negative error code in case of
 * failure.
 */
static int __erase_worker(struct ubi_device *ubi, struct ubi_work *wl_wrk)
{
	struct ubi_wl_entry *e = wl_wrk->e;
	int pnum = e->pnum;
	int err, available_consumed = 0;

	dbg_wl("erase PEB %d EC %d", pnum, e->ec);

	err = sync_erase(ubi, e, wl_wrk->torture);
	if (!err) {
		spin_lock(&ubi->wl_lock);
		wl_tree_add(e, &ubi->free);
		ubi->free_count++;
		spin_unlock(&ubi->wl_lock);

		/*
		 * One more erase operation has happened, take care about
		 * protected physical eraseblocks.
		 */
		serve_prot_queue(ubi);

		/* And take care about wear-leveling */
		err = ensure_wear_leveling(ubi);
		return err;
	}

	ubi_err(ubi, "failed to erase PEB %d, error %d", pnum, err);

	if (err == -EINTR || err == -ENOMEM || err == -EAGAIN ||
	    err == -EBUSY) {
		int err1;

		/* Re-schedule the LEB for erasure */
		err1 = schedule_erase(ubi, e, true);
		if (err1) {
			wl_entry_destroy(ubi, e);
			err = err1;
			goto out_ro;
		}
		return err;
	}

	wl_entry_destroy(ubi, e);
	if (err != -EIO)
		/*
		 * If this is not %-EIO, we have no idea what to do. Scheduling
		 * this physical eraseblock for erasure again would cause
		 * errors again and again. Well, lets switch to R/O mode.
		 */
		goto out_ro;

	/* It is %-EIO, the PEB went bad */

	if (!ubi->bad_allowed) {
		ubi_err(ubi, "bad physical eraseblock %d detected", pnum);
		goto out_ro;
	}

	spin_lock(&ubi->volumes_lock);
	if (ubi->beb_rsvd_pebs == 0) {
		if (ubi->avail_pebs == 0) {
			spin_unlock(&ubi->volumes_lock);
			ubi_err(ubi, "no reserved/available physical eraseblocks");
			goto out_ro;
		}
		ubi->avail_pebs -= 1;
		available_consumed = 1;
	}
	spin_unlock(&ubi->volumes_lock);

	ubi_msg(ubi, "mark PEB %d as bad", pnum);
	err = ubi_io_mark_bad(ubi, pnum);
	if (err)
		goto out_ro;

	spin_lock(&ubi->volumes_lock);
	if (ubi->beb_rsvd_pebs > 0) {
		if (available_consumed) {
			/*
			 * The amount of reserved PEBs increased since we last
			 * checked.
			 */
			ubi->avail_pebs += 1;
			available_consumed = 0;
		}
		ubi->beb_rsvd_pebs -= 1;
	}
	ubi->bad_peb_count += 1;
	ubi->good_peb_count -= 1;
	ubi_calculate_reserved(ubi);
	if (available_consumed)
		ubi_warn(ubi, "no PEBs in the reserved pool, used an available PEB");
	else if (ubi->beb_rsvd_pebs)
		ubi_msg(ubi, "%d PEBs left in the reserve",
			ubi->beb_rsvd_pebs);
	else
		ubi_warn(ubi, "last PEB from the reserve was used");
	spin_unlock(&ubi->volumes_lock);

	return err;

out_ro:
	if (available_consumed) {
		spin_lock(&ubi->volumes_lock);
		ubi->avail_pebs += 1;
		spin_unlock(&ubi->volumes_lock);
	}
	ubi_ro_mode(ubi);
	return err;
}

static int erase_worker(struct ubi_device *ubi, struct ubi_work *wl_wrk,
			  int shutdown)
{
	int ret;

	if (shutdown) {
		struct ubi_wl_entry *e = wl_wrk->e;

		dbg_wl("cancel erasure of PEB %d EC %d", e->pnum, e->ec);
		wl_entry_destroy(ubi, e);
		return 0;
	}

	ret = __erase_worker(ubi, wl_wrk);
	return ret;
}

/**
 * ubi_wl_put_peb - return a PEB to the wear-leveling sub-system.
 * @ubi: UBI device description object
 * @pnum: physical eraseblock to return
 * @torture: if this physical eraseblock has to be tortured
 *
 * This function is called to return physical eraseblock @pnum to the pool of
 * free physical eraseblocks. The @torture flag has to be set if an I/O error
 * occurred to this @pnum and it has to be tested. This function returns zero
 * in case of success, and a negative error code in case of failure.
 */
int ubi_wl_put_peb(struct ubi_device *ubi, int pnum, int torture)
{
	int err;
	struct ubi_wl_entry *e;
	struct ubi_work *wrk;

	dbg_wl("PEB %d", pnum);
	ubi_assert(pnum >= 0);
	ubi_assert(pnum < ubi->peb_count);
	ubi_assert(!ubi->consolidated || !ubi->consolidated[pnum]);

	down_read(&ubi->fm_protect);

retry:
	spin_lock(&ubi->wl_lock);
	e = ubi->lookuptbl[pnum];
	if (e == ubi->move_from) {
		/*
		 * User is putting the physical eraseblock which was selected to
		 * be moved. It will be scheduled for erasure in the
		 * wear-leveling worker.
		 */
		dbg_wl("PEB %d is being moved, wait", pnum);
		spin_unlock(&ubi->wl_lock);

		/* Wait for the WL worker by taking the @ubi->move_mutex */
		mutex_lock(&ubi->move_mutex);
		mutex_unlock(&ubi->move_mutex);
		goto retry;
	} else if (e == ubi->move_to) {
		/*
		 * User is putting the physical eraseblock which was selected
		 * as the target the data is moved to. It may happen if the EBA
		 * sub-system already re-mapped the LEB in 'ubi_eba_copy_leb()'
		 * but the WL sub-system has not put the PEB to the "used" tree
		 * yet, but it is about to do this. So we just set a flag which
		 * will tell the WL worker that the PEB is not needed anymore
		 * and should be scheduled for erasure.
		 */
		dbg_wl("PEB %d is the target of data moving", pnum);
		ubi_assert(!ubi->move_to_put);
		ubi->move_to_put = 1;
		spin_unlock(&ubi->wl_lock);
		up_read(&ubi->fm_protect);
		return 0;
	} else {
		if (in_wl_tree(e, &ubi->used)) {
			self_check_in_wl_tree(ubi, e, &ubi->used);
			rb_erase(&e->u.rb, &ubi->used);
		} else if (in_wl_tree(e, &ubi->scrub)) {
			self_check_in_wl_tree(ubi, e, &ubi->scrub);
			rb_erase(&e->u.rb, &ubi->scrub);
		} else if (in_wl_tree(e, &ubi->erroneous)) {
			self_check_in_wl_tree(ubi, e, &ubi->erroneous);
			rb_erase(&e->u.rb, &ubi->erroneous);
			ubi->erroneous_peb_count -= 1;
			ubi_assert(ubi->erroneous_peb_count >= 0);
			/* Erroneous PEBs should be tortured */
			torture = 1;
		} else {
			err = prot_queue_del(ubi, e->pnum);
			if (err) {
				ubi_err(ubi, "PEB %d not found", pnum);
				ubi_ro_mode(ubi);
				spin_unlock(&ubi->wl_lock);
				up_read(&ubi->fm_protect);
				return err;
			}
		}
	}
	spin_unlock(&ubi->wl_lock);

	wrk = ubi_alloc_erase_work(ubi, e, torture);
	if (!wrk) {
		spin_lock(&ubi->wl_lock);
		wl_tree_add(e, &ubi->used);
		spin_unlock(&ubi->wl_lock);
	}
	up_read(&ubi->fm_protect);

	if (!wrk)
		return -ENOMEM;

	ubi_schedule_work(ubi, wrk);

	return 0;
}

/**
 * ubi_wl_scrub_peb - schedule a physical eraseblock for scrubbing.
 * @ubi: UBI device description object
 * @pnum: the physical eraseblock to schedule
 *
 * If a bit-flip in a physical eraseblock is detected, this physical eraseblock
 * needs scrubbing. This function schedules a physical eraseblock for
 * scrubbing which is done in background. This function returns zero in case of
 * success and a negative error code in case of failure.
 */
int ubi_wl_scrub_peb(struct ubi_device *ubi, int pnum)
{
	struct ubi_wl_entry *e;

	ubi_msg(ubi, "schedule PEB %d for scrubbing", pnum);

retry:
	spin_lock(&ubi->wl_lock);
	e = ubi->lookuptbl[pnum];
	if (e == ubi->move_from || in_wl_tree(e, &ubi->scrub) ||
				   in_wl_tree(e, &ubi->erroneous)) {
		spin_unlock(&ubi->wl_lock);
		return 0;
	}

	if (e == ubi->move_to) {
		/*
		 * This physical eraseblock was used to move data to. The data
		 * was moved but the PEB was not yet inserted to the proper
		 * tree. We should just wait a little and let the WL worker
		 * proceed.
		 */
		spin_unlock(&ubi->wl_lock);
		dbg_wl("the PEB %d is not in proper tree, retry", pnum);
		yield();
		goto retry;
	}

	if (in_wl_tree(e, &ubi->used)) {
		self_check_in_wl_tree(ubi, e, &ubi->used);
		rb_erase(&e->u.rb, &ubi->used);
	} else {
		int err;

		err = prot_queue_del(ubi, e->pnum);
		if (err) {
			ubi_err(ubi, "PEB %d not found", pnum);
			ubi_ro_mode(ubi);
			spin_unlock(&ubi->wl_lock);
			return err;
		}
	}

	wl_tree_add(e, &ubi->scrub);
	spin_unlock(&ubi->wl_lock);

	/*
	 * Technically scrubbing is the same as wear-leveling, so it is done
	 * by the WL worker.
	 */
	return ensure_wear_leveling(ubi);
}

static int scrub_possible(struct ubi_device *ubi, struct ubi_wl_entry *e)
{
	if (in_wl_tree(e, &ubi->scrub))
		return -EBUSY;
	else if (in_wl_tree(e, &ubi->erroneous))
		return -EBUSY;
	else if (ubi->move_from == e)
		return -EBUSY;
	else if (ubi->move_to == e)
		return -EBUSY;

	return 0;
}

/**
 * ubi_bitflip_check - Check an eraseblock for bitflips and scrub it if needed.
 * @ubi: UBI device description object
 * @pnum: the physical eraseblock to schedule
 * @force_scrub: force scrubbing if non-zero, schedule erase otherwise
 *
 * This function reads the given eraseblock and checks if bitflips occured.
 * In case of bitflips, the eraseblock is scheduled for scrubbing.
 * If scrubbing is forced with @force_scrub, the eraseblock is not read,
 * but scheduled for scrubbing right away.
 *
 * Returns:
 * %EINVAL, PEB is out of range
 * %ENOENT, PEB is no longer used by UBI
 * %EBUSY, PEB cannot be checked now or a check is currently running on it
 * %EAGAIN, bit flips happened but scrubbing is currently not possible
 * %EUCLEAN, bit flips happened and PEB is scheduled for scrubbing
 * %0, no bit flips detected
 */
int ubi_bitflip_check(struct ubi_device *ubi, int pnum, int force_scrub)
{
	int err;
	struct ubi_wl_entry *e;

	if (pnum < 0 || pnum >= ubi->peb_count) {
		err = -EINVAL;
		goto out;
	}

	/*
	 * Pause all parallel work, otherwise it can happen that the
	 * erase worker frees a wl entry under us.
	 */
	ubi_work_suspend(ubi);

	/*
	 * Make sure that the wl entry does not change state while
	 * inspecting it.
	 */
	spin_lock(&ubi->wl_lock);
	e = ubi->lookuptbl[pnum];
	if (!e) {
		spin_unlock(&ubi->wl_lock);
		err = -ENOENT;
		goto out_resume;
	}
	/*
	 * Does it make sense to check this PEB?
	 * Maybe UBI is already inspecing it...
	 */
	err = scrub_possible(ubi, e);
	spin_unlock(&ubi->wl_lock);
	if (err)
		goto out_resume;

	if (!force_scrub) {
		mutex_lock(&ubi->buf_mutex);
		err = ubi_io_raw_read(ubi, ubi->peb_buf, pnum, 0, ubi->peb_size);
		mutex_unlock(&ubi->buf_mutex);
	}

	if (err == UBI_IO_BITFLIPS || force_scrub) {
		/*
		 * Okay, bit flip happened, let's figure out what we can do.
		 */
		spin_lock(&ubi->wl_lock);

		/*
		 * Need to re-check state
		 */
		err = scrub_possible(ubi, e);
		if (err) {
			spin_unlock(&ubi->wl_lock);
			goto out_resume;
		}

		if (in_pq(ubi, e)) {
			prot_queue_del(ubi, e->pnum);
			wl_tree_add(e, &ubi->scrub);
			spin_unlock(&ubi->wl_lock);

			err = ensure_wear_leveling(ubi);
		} else if (in_wl_tree(e, &ubi->used)) {
			rb_erase(&e->u.rb, &ubi->used);
			wl_tree_add(e, &ubi->scrub);
			spin_unlock(&ubi->wl_lock);

			err = ensure_wear_leveling(ubi);
		} else if (in_wl_tree(e, &ubi->free)) {
			rb_erase(&e->u.rb, &ubi->free);
			ubi->free_count--;
			spin_unlock(&ubi->wl_lock);

			/*
			 * This PEB is empty we can schedule it for
			 * erasure right away. No wear leveling needed.
			 */
			err = schedule_erase(ubi, e, force_scrub ? 0 : 1);
		} else {
			spin_unlock(&ubi->wl_lock);
			ubi_work_resume(ubi);
			/*
			 * e is owned by fastmap. We are not allowed to
			 * move it as the on-flash fastmap data structure refers to it.
			 * Let's schedule a new fastmap write
			 * such that the said PEB can get released.
			 */
			ubi_update_fastmap(ubi);
			err = -EAGAIN;
			goto out;
		}

		if (!err && !force_scrub)
			err = -EUCLEAN;
	} else {
		err = 0;
	}

out_resume:
	ubi_work_resume(ubi);
out:
	return err;
}

/**
 * tree_destroy - destroy an RB-tree.
 * @ubi: UBI device description object
 * @root: the root of the tree to destroy
 */
static void tree_destroy(struct ubi_device *ubi, struct rb_root *root)
{
	struct rb_node *rb;
	struct ubi_wl_entry *e;

	rb = root->rb_node;
	while (rb) {
		if (rb->rb_left)
			rb = rb->rb_left;
		else if (rb->rb_right)
			rb = rb->rb_right;
		else {
			e = rb_entry(rb, struct ubi_wl_entry, u.rb);

			rb = rb_parent(rb);
			if (rb) {
				if (rb->rb_left == &e->u.rb)
					rb->rb_left = NULL;
				else
					rb->rb_right = NULL;
			}

			wl_entry_destroy(ubi, e);
		}
	}
}

/**
 * ubi_wl_init - initialize the WL sub-system using attaching information.
 * @ubi: UBI device description object
 * @ai: attaching information
 *
 * This function returns zero in case of success, and a negative error code in
 * case of failure.
 */
int ubi_wl_init(struct ubi_device *ubi, struct ubi_attach_info *ai)
{
	int err, i, reserved_pebs, found_pebs = 0;
	struct rb_node *rb1, *rb2;
	struct ubi_ainf_volume *av;
	struct ubi_ainf_leb *leb;
	struct ubi_ainf_peb *peb, *tmp;
	struct ubi_wl_entry *e;
	struct ubi_leb_desc *clebs;

	ubi->used = ubi->erroneous = ubi->free = ubi->scrub = RB_ROOT;
	spin_lock_init(&ubi->wl_lock);
	mutex_init(&ubi->move_mutex);
	mutex_init(&ubi->work_mutex);
	ubi->max_ec = ai->max_ec;
	INIT_LIST_HEAD(&ubi->works);

	sprintf(ubi->bgt_name, UBI_BGT_NAME_PATTERN, ubi->ubi_num);

	err = -ENOMEM;
	ubi->lookuptbl = kzalloc(ubi->peb_count * sizeof(void *), GFP_KERNEL);
	if (!ubi->lookuptbl)
		return err;

	if (ubi->lebs_per_cpeb > 1) {
		ubi->consolidated = kzalloc(ubi->peb_count * sizeof(void *),
					    GFP_KERNEL);
		if (!ubi->consolidated) {
			kfree(ubi->lookuptbl);
			return err;
		}
	}

	for (i = 0; i < UBI_PROT_QUEUE_LEN; i++)
		INIT_LIST_HEAD(&ubi->pq[i]);
	ubi->pq_head = 0;

	ubi->free_count = 0;
	list_for_each_entry_safe(peb, tmp, &ai->erase, list) {
		cond_resched();

		if (ubi->lookuptbl[peb->pnum])
			continue;

		e = kmem_cache_alloc(ubi_wl_entry_slab, GFP_KERNEL);
		if (!e)
			goto out_free;

		e->pnum = peb->pnum;
		e->ec = peb->ec;
		ubi->lookuptbl[e->pnum] = e;
		if (schedule_erase(ubi, e, false)) {
			wl_entry_destroy(ubi, e);
			goto out_free;
		}

		found_pebs++;
	}

	list_for_each_entry(peb, &ai->free, list) {
		cond_resched();

		if (ubi->lookuptbl[peb->pnum])
			continue;

		e = kmem_cache_alloc(ubi_wl_entry_slab, GFP_KERNEL);
		if (!e)
			goto out_free;

		e->pnum = peb->pnum;
		e->ec = peb->ec;
		ubi_assert(e->ec >= 0);

		wl_tree_add(e, &ubi->free);
		ubi->free_count++;

		ubi->lookuptbl[e->pnum] = e;

		found_pebs++;
	}

	list_for_each_entry(peb, &ai->used, list) {
		e = kmem_cache_alloc(ubi_wl_entry_slab, GFP_KERNEL);
		if (!e)
			goto out_free;

		e->pnum = peb->pnum;
		e->ec = peb->ec;
		ubi->lookuptbl[e->pnum] = e;

		if (!peb->scrub) {
			dbg_wl("add PEB %d EC %d to the used tree",
			       e->pnum, e->ec);
			wl_tree_add(e, &ubi->used);
		} else {
			dbg_wl("add PEB %d EC %d to the scrub tree",
			       e->pnum, e->ec);
			wl_tree_add(e, &ubi->scrub);
		}

		if (peb->consolidated) {
			int i;

			clebs = kmalloc(sizeof(*clebs) *
					ubi->lebs_per_cpeb,
					GFP_KERNEL);
			if (!clebs)
				goto out_free;

			for (i = 0; i < ubi->lebs_per_cpeb; i++) {
				clebs[i].lnum = -1;
				clebs[i].vol_id = -1;
			}

			ubi->consolidated[peb->pnum] = clebs;
		}

		found_pebs++;
	}

	ubi_rb_for_each_entry(rb1, av, &ai->volumes, rb) {
		ubi_rb_for_each_entry(rb2, leb, &av->root, rb) {
			cond_resched();

			if (ubi->lebs_per_cpeb > 1) {
				clebs = ubi->consolidated[leb->peb->pnum];
				if (clebs)
					clebs[leb->peb_pos] = leb->desc;
			}
		}
	}

	dbg_wl("found %i PEBs", found_pebs);

	if (ubi->fm) {
		ubi_assert(ubi->good_peb_count ==
			   found_pebs + ubi->fm->used_blocks);

		for (i = 0; i < ubi->fm->used_blocks; i++) {
			e = ubi->fm->e[i];
			ubi->lookuptbl[e->pnum] = e;
		}
	}
	else
		ubi_assert(ubi->good_peb_count == found_pebs);

	reserved_pebs = UBI_WL_RESERVED_PEBS;
	ubi_fastmap_init(ubi, &reserved_pebs);

	if (ubi->avail_pebs < reserved_pebs) {
		ubi_err(ubi, "no enough physical eraseblocks (%d, need %d)",
			ubi->avail_pebs, reserved_pebs);
		if (ubi->corr_peb_count)
			ubi_err(ubi, "%d PEBs are corrupted and not used",
				ubi->corr_peb_count);
		err = -ENOSPC;
		goto out_free;
	}
	ubi->avail_pebs -= reserved_pebs;
	ubi->rsvd_pebs += reserved_pebs;

	/* Schedule wear-leveling if needed */
	err = ensure_wear_leveling(ubi);
	if (err)
		goto out_free;

	return 0;

out_free:
	ubi_work_close(ubi, err);
	tree_destroy(ubi, &ubi->used);
	tree_destroy(ubi, &ubi->free);
	tree_destroy(ubi, &ubi->scrub);
	kfree(ubi->consolidated);
	kfree(ubi->lookuptbl);
	return err;
}

/**
 * protection_queue_destroy - destroy the protection queue.
 * @ubi: UBI device description object
 */
static void protection_queue_destroy(struct ubi_device *ubi)
{
	int i;
	struct ubi_wl_entry *e, *tmp;

	for (i = 0; i < UBI_PROT_QUEUE_LEN; ++i) {
		list_for_each_entry_safe(e, tmp, &ubi->pq[i], u.list) {
			list_del(&e->u.list);
			wl_entry_destroy(ubi, e);
		}
	}
}

/**
 * ubi_wl_close - close the wear-leveling sub-system.
 * @ubi: UBI device description object
 */
void ubi_wl_close(struct ubi_device *ubi)
{
	dbg_wl("close the WL sub-system");
	ubi_fastmap_close(ubi);
	ubi_work_close(ubi, 0);
	protection_queue_destroy(ubi);
	tree_destroy(ubi, &ubi->used);
	tree_destroy(ubi, &ubi->erroneous);
	tree_destroy(ubi, &ubi->free);
	tree_destroy(ubi, &ubi->scrub);
	kfree(ubi->lookuptbl);
	kfree(ubi->consolidated);
}

/**
 * self_check_ec - make sure that the erase counter of a PEB is correct.
 * @ubi: UBI device description object
 * @pnum: the physical eraseblock number to check
 * @ec: the erase counter to check
 *
 * This function returns zero if the erase counter of physical eraseblock @pnum
 * is equivalent to @ec, and a negative error code if not or if an error
 * occurred.
 */
static int self_check_ec(struct ubi_device *ubi, int pnum, int ec)
{
	int err;
	long long read_ec;
	struct ubi_ec_hdr *ec_hdr;

	if (!ubi_dbg_chk_gen(ubi))
		return 0;

	ec_hdr = kzalloc(ubi->ec_hdr_alsize, GFP_NOFS);
	if (!ec_hdr)
		return -ENOMEM;

	err = ubi_io_read_ec_hdr(ubi, pnum, ec_hdr, 0);
	if (err && err != UBI_IO_BITFLIPS) {
		/* The header does not have to exist */
		err = 0;
		goto out_free;
	}

	read_ec = be64_to_cpu(ec_hdr->ec);
	if (ec != read_ec && read_ec - ec > 1) {
		ubi_err(ubi, "self-check failed for PEB %d", pnum);
		ubi_err(ubi, "read EC is %lld, should be %d", read_ec, ec);
		dump_stack();
		err = 1;
	} else
		err = 0;

out_free:
	kfree(ec_hdr);
	return err;
}

/**
 * self_check_in_wl_tree - check that wear-leveling entry is in WL RB-tree.
 * @ubi: UBI device description object
 * @e: the wear-leveling entry to check
 * @root: the root of the tree
 *
 * This function returns zero if @e is in the @root RB-tree and %-EINVAL if it
 * is not.
 */
static int self_check_in_wl_tree(const struct ubi_device *ubi,
				 struct ubi_wl_entry *e, struct rb_root *root)
{
	if (!ubi_dbg_chk_gen(ubi))
		return 0;

	if (in_wl_tree(e, root))
		return 0;

	ubi_err(ubi, "self-check failed for PEB %d, EC %d, RB-tree %p ",
		e->pnum, e->ec, root);
	dump_stack();
	return -EINVAL;
}

/**
 * self_check_in_pq - check if wear-leveling entry is in the protection
 *                        queue.
 * @ubi: UBI device description object
 * @e: the wear-leveling entry to check
 *
 * This function returns zero if @e is in @ubi->pq and %-EINVAL if it is not.
 */
static int self_check_in_pq(const struct ubi_device *ubi,
			    struct ubi_wl_entry *e)
{
	if (!ubi_dbg_chk_gen(ubi))
		return 0;

	if (in_pq(ubi, e))
		return 0;

	ubi_err(ubi, "self-check failed for PEB %d, EC %d, Protect queue",
		e->pnum, e->ec);
	dump_stack();
	return -EINVAL;
}

static bool enough_free_pebs(struct ubi_device *ubi, int min_limit)
{
	return ubi->free_count > min_limit;
}

#ifndef CONFIG_MTD_UBI_FASTMAP
static struct ubi_wl_entry *get_peb_for_wl(struct ubi_device *ubi)
{
	struct ubi_wl_entry *e;

	/*
	 * Hold back one PEB for the producing case,
	 * currently only for consolidation.
	 */
	if (!enough_free_pebs(ubi, UBI_CONSO_RESERVED_PEBS))
		return NULL;

	e = find_wl_entry(ubi, &ubi->free, WL_FREE_MAX_DIFF);
	self_check_in_wl_tree(ubi, e, &ubi->free);
	ubi->free_count--;
	ubi_assert(ubi->free_count >= 0);
	rb_erase(&e->u.rb, &ubi->free);

	return e;
}

/**
 * produce_free_peb - produce a free physical eraseblock.
 * @ubi: UBI device description object
 *
 * This function tries to make a free PEB by means of synchronous execution of
 * pending works. This may be needed if, for example the background thread is
 * disabled. Returns zero in case of success and a negative error code in case
 * of failure.
 */
static int produce_free_peb(struct ubi_device *ubi, int min_limit)
{
	ubi_assert(spin_is_locked(&ubi->wl_lock));

	while (!enough_free_pebs(ubi, min_limit)) {
		spin_unlock(&ubi->wl_lock);

		ubi_eba_consolidate(ubi);

		dbg_wl("do one work synchronously");
		if (!ubi_work_join_one(ubi)) {
			spin_lock(&ubi->wl_lock);

			/* Work can finish before we tried to join. */
			if (enough_free_pebs(ubi, min_limit))
				break;

			/* Nothing to do. We have to give up. */
			return -ENOSPC;
		}

		spin_lock(&ubi->wl_lock);
	}

	return 0;
}

/**
 * ubi_wl_get_peb - get a physical eraseblock.
 * @ubi: UBI device description object
 * @producing: true if this function is being called from a context
 * which is trying to produce more free PEBs but needs a new one to
 * achieve that. i.e. consolidatation work.
 *
 * This function returns a physical eraseblock in case of success and a
 * negative error code in case of failure.
 * Returns with ubi->fm_eba_sem held in read mode!
 */
int ubi_wl_get_peb(struct ubi_device *ubi, bool producing, int min_limit)
{
	int err = 0;
	struct ubi_wl_entry *e;

retry:
	down_read(&ubi->fm_eba_sem);
	spin_lock(&ubi->wl_lock);

	if (!enough_free_pebs(ubi, min_limit) && !producing) {
		err = produce_free_peb(ubi, min_limit);
		if (err < 0) {
			ubi_err(ubi, "unable to produce free eraseblocks: %i", err);
			spin_unlock(&ubi->wl_lock);
			return err;
		}
		spin_unlock(&ubi->wl_lock);
		up_read(&ubi->fm_eba_sem);
		goto retry;
	} else if (!enough_free_pebs(ubi, min_limit) && producing) {
		ubi_err(ubi, "no free eraseblocks in producing case");
		ubi_assert(0);
		spin_unlock(&ubi->wl_lock);
		return -ENOSPC;
	}

	e = wl_get_wle(ubi);
	if (e)
		prot_queue_add(ubi, e);
	else
		err = -ENOSPC;
	spin_unlock(&ubi->wl_lock);

	if (err)
		return err;

	err = ubi_self_check_all_ff(ubi, e->pnum, ubi->vid_hdr_aloffset,
				    ubi->peb_size - ubi->vid_hdr_aloffset);
	if (err) {
		ubi_err(ubi, "new PEB %d does not contain all 0xFF bytes", e->pnum);
		return err;
	}

	return e->pnum;
}
#else
#include "fastmap-wl.c"
#endif
