/* eAVSyncCore — AMLogic kernel tsync engine coordinator.
 *
 * Owns /dev/tsync + wraps the four ioctls + /proc/stb/pcr_offset writes.
 * Backing kernel patch: meta-dream/recipes-linux/linux-meson64/
 *   aml_tsync_chrdev_ioctl.patch (LineageOS aml-4.9 backport, plus
 *   new_arch=true for G12A/G12B). */

#include <lib/dvb/avsync_core.h>
#include <lib/base/eerror.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>

/* ioctl encodings — must match kernel patch in
 * include/linux/amlogic/media/frame_sync/tsync.h.
 *
 * Local copies kept here to avoid pulling kernel headers into userspace. */
#ifndef TSYNC_IOC_MAGIC
#define TSYNC_IOC_MAGIC 'T'

struct dmx_info {
	int demux_device_id;
	int index;
	int vpid;
	int apid;
	int pcrpid;
};

#define TSYNC_IOC_SET_DEMUX_INFO   _IOW(TSYNC_IOC_MAGIC, 0x06, struct dmx_info)
#define TSYNC_IOC_STOP_TSYNC_PCR   _IO ( TSYNC_IOC_MAGIC, 0x08)
#endif

eAVSyncCore *eAVSyncCore::s_instance = NULL;
pthread_mutex_t eAVSyncCore::s_instance_mutex = PTHREAD_MUTEX_INITIALIZER;

eAVSyncCore *eAVSyncCore::getInstance()
{
	pthread_mutex_lock(&s_instance_mutex);
	if (!s_instance)
		s_instance = new eAVSyncCore();
	pthread_mutex_unlock(&s_instance_mutex);
	return s_instance;
}

eAVSyncCore::eAVSyncCore()
	: m_tsync_fd(-1),
	  m_kernel_sync_active(false),
	  m_pcrpid_absent(false)
{
	pthread_mutex_init(&m_tsync_mutex, NULL);
	eDebug("[eAVSyncCore] init");
}

eAVSyncCore::~eAVSyncCore()
{
	if (m_tsync_fd >= 0) {
		close(m_tsync_fd);
		m_tsync_fd = -1;
	}
	pthread_mutex_destroy(&m_tsync_mutex);
}

int eAVSyncCore::openTsync()
{
	/* Caller holds m_tsync_mutex. */
	if (m_tsync_fd >= 0)
		return m_tsync_fd;

	m_tsync_fd = open("/dev/tsync", O_RDWR | O_CLOEXEC);
	if (m_tsync_fd < 0) {
		eDebug("[eAVSyncCore] open /dev/tsync failed (%m) — kernel patch missing?");
		return -1;
	}
	return m_tsync_fd;
}

int eAVSyncCore::writeNode(const char *path, const char *value)
{
	int fd = open(path, O_WRONLY | O_CLOEXEC);
	if (fd < 0) {
		eDebug("[eAVSyncCore] open %s for write failed (%m)", path);
		return -1;
	}
	ssize_t n = write(fd, value, strlen(value));
	int saved_errno = errno;
	close(fd);
	if (n < 0) {
		errno = saved_errno;
		eDebug("[eAVSyncCore] write %s failed (%m)", path);
		return -1;
	}
	return 0;
}

int eAVSyncCore::writeHex(const char *path, unsigned int value)
{
	char buf[16];
	int n = snprintf(buf, sizeof(buf), "%x", value);
	if (n < 0 || n >= (int)sizeof(buf))
		return -1;
	return writeNode(path, buf);
}

int64_t eAVSyncCore::readHex(const char *path)
{
	FILE *fp = fopen(path, "r");
	if (!fp)
		return -1;
	char buf[32] = {0};
	size_t n = fread(buf, 1, sizeof(buf) - 1, fp);
	fclose(fp);
	if (n == 0)
		return -1;
	char *end = NULL;
	int64_t v = strtoll(buf, &end, 0);
	if (end == buf)
		return -1;
	return v;
}

int eAVSyncCore::enableKernelSync()
{
	/* Drop any stale pts_audio/pts_video left by a foreign owner
	 * (e.g. exteplayer3/gstplayer2 in HLS mode). pts_audio in particular
	 * is never updated by Live-TV's SW-decode path before our drift
	 * loop runs, so a stale value from exteplayer3 keeps pcrmaster from
	 * converging — kernel sees audio "way behind" and drops video frames
	 * to compensate. */
	writeNode("/sys/class/tsync/pts_audio", "0");
	writeNode("/sys/class/tsync/pts_video", "0");
	writeNode("/sys/class/tsync/discontinue", "1");
	int r1 = writeNode("/sys/class/tsync/mode", "2");
	int r2 = writeNode("/sys/class/tsync/enable", "1");
	if (r1 == 0 && r2 == 0) {
		m_kernel_sync_active = true;
		eDebug("[eAVSyncCore] kernel tsync enabled (mode=pcrmaster)");
		return 0;
	}
	eDebug("[eAVSyncCore] enableKernelSync: mode=%d enable=%d", r1, r2);
	return -1;
}

/* Ephemeral /dev/tsync open+ioctl+close — no fd held across streams. */
int eAVSyncCore::setDemuxInfo(int demux_device_id, int index,
							  int vpid, int apid, int pcrpid)
{
	struct dmx_info info;
	memset(&info, 0, sizeof(info));
	info.demux_device_id = demux_device_id;
	info.index           = index;
	info.vpid            = vpid;
	info.apid            = apid;
	info.pcrpid          = pcrpid;

	int fd = open("/dev/tsync", O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		eDebug("[eAVSyncCore] open /dev/tsync failed (%m) — kernel patch missing?");
		return -1;
	}
	int r = ioctl(fd, TSYNC_IOC_SET_DEMUX_INFO, &info);
	int saved_errno = errno;
	close(fd);
	if (r < 0) {
		errno = saved_errno;
		eDebug("[eAVSyncCore] TSYNC_IOC_SET_DEMUX_INFO failed (%m) "
			   "dmx=%d idx=%d v=0x%x a=0x%x p=0x%x",
			   demux_device_id, index, vpid, apid, pcrpid);
	} else {
		m_pcrpid_absent = (pcrpid == 0x1FFF);
		eDebug("[eAVSyncCore] setDemuxInfo dmx=%d v=0x%x a=0x%x p=0x%x %s",
			   demux_device_id, vpid, apid, pcrpid,
			   m_pcrpid_absent ? "(sw-descramble)" : "(fta)");
	}
	return r;
}

int eAVSyncCore::stopPCRSync()
{
	int fd = open("/dev/tsync", O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		eDebug("[eAVSyncCore] open /dev/tsync failed (%m)");
		return -1;
	}
	int r = ioctl(fd, TSYNC_IOC_STOP_TSYNC_PCR);
	int saved_errno = errno;
	close(fd);
	if (r < 0) {
		errno = saved_errno;
		/* EBUSY = "nothing to stop" — expected on the first setState after
		 * boot (no prior pts_start) or after a previous stop. We call this
		 * defensively on every setState transition; the kernel returns
		 * EBUSY when pts_start(VIDEO) wasn't active. Silent on EBUSY,
		 * log other errors. */
		if (saved_errno != EBUSY)
			eDebug("[eAVSyncCore] TSYNC_IOC_STOP_TSYNC_PCR failed (%m)");
	} else {
		eDebug("[eAVSyncCore] stopPCRSync");
	}
	return r;
}

int eAVSyncCore::setPCROffset(int offset_90khz)
{
	int r = writeHex("/proc/stb/pcr_offset", (unsigned int)offset_90khz);
	if (r == 0)
		eDebug("[eAVSyncCore] pcr_offset = 0x%x (%dms)",
			   offset_90khz, offset_90khz / 90);
	return r;
}

int eAVSyncCore::setAutoPCROffset(int offset_90khz)
{
	int r = writeHex("/proc/stb/auto_pcr_offset", (unsigned int)offset_90khz);
	if (r == 0)
		eDebug("[eAVSyncCore] auto_pcr_offset = 0x%x", offset_90khz);
	return r;
}

int64_t eAVSyncCore::readPtsPcrscr()
{
	return readHex("/sys/class/tsync/pts_pcrscr");
}

int64_t eAVSyncCore::readPtsVideo()
{
	return readHex("/sys/class/tsync/pts_video");
}
