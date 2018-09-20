#include <linux/slab.h>
#include <linux/crc32.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include "ubi.h"

/*
 * Maximum number of consecutive background thread failures which is enough to
 * switch to read-only mode.
 */
#define WORK_MAX_FAILURES 32

/**
 * work_suspended - Check whether UBI work is suspended.
 */
static bool work_suspended(struct ubi_device *ubi)
{
	return ubi->thread_suspended || !ubi->thread_enabled;
}

/**
 * destroy_work - destroy an UBI work.
 * @ref: kref object
 *
 * This function is called by kref upon the last reference is gone.
 */
static void destroy_work(struct kref *ref)
{
	struct ubi_work *wrk = container_of(ref, struct ubi_work, ref);

	kfree(wrk);
}

/**
 * ubi_schedule_work - schedule a work.
 * @ubi: UBI device description object
 * @wrk: the work to schedule
 *
 * This function adds a work defined by @wrk to the tail of the pending works
 * list.
 */
void ubi_schedule_work(struct ubi_device *ubi, struct ubi_work *wrk)
{
	ubi_assert(ubi->thread_enabled);

	mutex_lock(&ubi->work_mutex);
	spin_lock(&ubi->wl_lock);
	list_add_tail(&wrk->list, &ubi->works);
	ubi_assert(ubi->works_count >= 0);
	ubi->works_count += 1;
	if (!work_suspended(ubi))
		wake_up_process(ubi->bgt_thread);
	spin_unlock(&ubi->wl_lock);
	mutex_unlock(&ubi->work_mutex);
}

int ubi_schedule_work_sync(struct ubi_device *ubi, struct ubi_work *wrk)
{
	int ret;

	kref_get(&wrk->ref);

	ubi_schedule_work(ubi, wrk);
	wait_for_completion(&wrk->comp);

	spin_lock(&ubi->wl_lock);
	ret = wrk->ret;
	kref_put(&wrk->ref, destroy_work);
	spin_unlock(&ubi->wl_lock);

	return ret;
}

struct ubi_work *ubi_alloc_work(struct ubi_device *ubi)
{
	struct ubi_work *wrk;

	wrk = kzalloc(sizeof(*wrk), GFP_NOFS);
	if (!wrk)
		return NULL;

	INIT_LIST_HEAD(&wrk->list);
	kref_init(&wrk->ref);
	init_completion(&wrk->comp);

	return wrk;
}

static void shutdown_work(struct ubi_device *ubi, int error)
{
	struct ubi_work *wrk;

	while (!list_empty(&ubi->works)) {
		wrk = list_entry(ubi->works.next, struct ubi_work, list);
		list_del_init(&wrk->list);
		wrk->func(ubi, wrk, 1);
		wrk->ret = error;
		complete_all(&wrk->comp);
		spin_lock(&ubi->wl_lock);
		kref_put(&wrk->ref, destroy_work);
		spin_unlock(&ubi->wl_lock);
		ubi->works_count -= 1;
		ubi_assert(ubi->works_count >= 0);
	}
}

/**
 * ubi_work_close - shutdown all pending works.
 * @ubi: UBI device description object
 */
void ubi_work_close(struct ubi_device *ubi, int error)
{
#ifdef CONFIG_MTD_UBI_FASTMAP
	flush_work(&ubi->fm_work);
#endif
	shutdown_work(ubi, error);
}

/**
 * do_work - do one pending work.
 * @ubi: UBI device description object
 *
 * This function returns zero in case of success and a negative error code in
 * case of failure.
 */
static int do_work(struct ubi_device *ubi)
{
	int err;
	struct ubi_work *wrk;

	cond_resched();

	mutex_lock(&ubi->work_mutex);
	spin_lock(&ubi->wl_lock);
	ubi_assert(!ubi->cur_work);
	if (list_empty(&ubi->works) || work_suspended(ubi)) {
		spin_unlock(&ubi->wl_lock);
		mutex_unlock(&ubi->work_mutex);
		return 0;
	}

	wrk = list_entry(ubi->works.next, struct ubi_work, list);
	list_del_init(&wrk->list);
	ubi->works_count -= 1;
	ubi_assert(ubi->works_count >= 0);
	ubi->cur_work = wrk;
	spin_unlock(&ubi->wl_lock);
	mutex_unlock(&ubi->work_mutex);

	/*
	 * Call the worker function. Do not touch the work structure
	 * after this call as it will have been freed or reused by that
	 * time by the worker function.
	 */
	err = wrk->func(ubi, wrk, 0);
	wrk->ret = err;
	if (err)
		ubi_err(ubi, "work failed with error code %d", err);

	spin_lock(&ubi->wl_lock);
	ubi->cur_work = NULL;
	spin_unlock(&ubi->wl_lock);

	complete_all(&wrk->comp);

	spin_lock(&ubi->wl_lock);
	kref_put(&wrk->ref, destroy_work);
	spin_unlock(&ubi->wl_lock);

	return err;
}

void ubi_work_suspend(struct ubi_device *ubi)
{
	struct ubi_work *wrk = NULL;

	mutex_lock(&ubi->work_mutex);
	spin_lock(&ubi->wl_lock);

	wrk = ubi->cur_work;
	if (wrk)
		kref_get(&wrk->ref);

	ubi->thread_suspended = 1;

	spin_unlock(&ubi->wl_lock);
	mutex_unlock(&ubi->work_mutex);

	if (wrk) {
		wait_for_completion(&wrk->comp);
		spin_lock(&ubi->wl_lock);
		kref_put(&wrk->ref, destroy_work);
		spin_unlock(&ubi->wl_lock);
	}
}

void ubi_work_resume(struct ubi_device *ubi)
{
	ubi->thread_suspended = 0;
	wake_up_process(ubi->bgt_thread);
}

/**
 * ubi_work_join_one - Run one work in sync.
 * @ubi: UBI device description object
 *
 * This function joins one work and waits for it.
 * Call it when you run out of free LEBs need to wait for one.
 * It returns false if no pending work was found to join, true otherwise.
 */
bool ubi_work_join_one(struct ubi_device *ubi)
{
	struct ubi_work *wrk;
	bool success = false;

	mutex_lock(&ubi->work_mutex);
	spin_lock(&ubi->wl_lock);
	if (ubi->cur_work)
		wrk = ubi->cur_work;
	else
		wrk = list_first_entry_or_null(&ubi->works,
			struct ubi_work, list);

	if (wrk)
		kref_get(&wrk->ref);
	spin_unlock(&ubi->wl_lock);
	mutex_unlock(&ubi->work_mutex);

	if (wrk) {
		wait_for_completion(&wrk->comp);
		if (wrk->ret == 0)
			success = true;

		spin_lock(&ubi->wl_lock);
		kref_put(&wrk->ref, destroy_work);
		spin_unlock(&ubi->wl_lock);
	}

	return success;
}

/**
 * ubi_work_flush - flush all pending works.
 * @ubi: UBI device description object
 *
 */
int ubi_work_flush(struct ubi_device *ubi)
{
	int ret = 0;
	struct ubi_work *wrk = NULL;

	dbg_wl("flush (%d pending works)", ubi->works_count);

	/* Find the last entry in the work list and wait for it. */
	mutex_lock(&ubi->work_mutex);
	spin_lock(&ubi->wl_lock);
	if (!list_empty(&ubi->works)) {
		wrk = list_last_entry(&ubi->works, struct ubi_work, list);
		kref_get(&wrk->ref);
	}
	spin_unlock(&ubi->wl_lock);
	mutex_unlock(&ubi->work_mutex);

	if (wrk) {
		wait_for_completion(&wrk->comp);
		ret = wrk->ret;
		spin_lock(&ubi->wl_lock);
		kref_put(&wrk->ref, destroy_work);
		spin_unlock(&ubi->wl_lock);
	}

	return ret;
}

/**
 * ubi_thread - UBI background thread.
 * @u: the UBI device description object pointer
 */
int ubi_thread(void *u)
{
	int failures = 0;
	struct ubi_device *ubi = u;

	ubi_msg(ubi, "background thread \"%s\" started, PID %d",
		ubi->bgt_name, task_pid_nr(current));

	set_freezable();
	for (;;) {
		int err;

		if (kthread_should_stop())
			break;

		if (try_to_freeze())
			continue;

		spin_lock(&ubi->wl_lock);
		if (list_empty(&ubi->works) || ubi->ro_mode ||
		    work_suspended(ubi)) {
			set_current_state(TASK_INTERRUPTIBLE);
			spin_unlock(&ubi->wl_lock);
			schedule();
			continue;
		}
		spin_unlock(&ubi->wl_lock);

		err = do_work(ubi);
		if (err) {
			ubi_err(ubi, "%s: work failed with error code %d",
				ubi->bgt_name, err);
			if (failures++ > WORK_MAX_FAILURES) {
				/*
				 * Too many failures, disable the thread and
				 * switch to read-only mode.
				 */
				ubi_err(ubi, "%s: %d consecutive failures",
					ubi->bgt_name, WORK_MAX_FAILURES);
				shutdown_work(ubi, -EROFS);
				ubi_ro_mode(ubi);
				ubi->thread_enabled = 0;
				continue;
			}
		} else
			failures = 0;

		cond_resched();
	}

	dbg_wl("background thread \"%s\" is killed", ubi->bgt_name);
	return 0;
}

