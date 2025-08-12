#include <obs-module.h>
#include <plugin-support.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <pthread.h>
#include <stdint.h>
#include <stdbool.h>

OBS_DECLARE_MODULE()
OBS_MODULE_USE_DEFAULT_LOCALE(PLUGIN_NAME, "en-US")
MODULE_EXPORT const char *obs_module_description(void)
{
    return "V4L2 mplane NV12 camera capture plugin";
}

#define DEV_PATH "/dev/video11"
#define WIDTH 1920
#define HEIGHT 1080
#define BUFFER_COUNT 4

struct buffer {
    void *start[VIDEO_MAX_PLANES];
    size_t length[VIDEO_MAX_PLANES];
};

struct v4l2_mplane_source {
    obs_source_t *source;
    int fd;
    struct buffer buffers[BUFFER_COUNT];
    gs_texture_t *texture;
    uint8_t *rgb_front;
    uint8_t *rgb_back;
    bool new_frame;
    pthread_mutex_t frame_lock;
};


static const char *mplane_get_name(void *unused)
{
    (void)unused;
    return "V4L2 mplane Camera";
}

static uint32_t mplane_width(void *data) { (void)data; return WIDTH; }
static uint32_t mplane_height(void *data) { (void)data; return HEIGHT; }

static void free_mapped_buffers(struct v4l2_mplane_source *s)
{
    if (!s) return;
    for (int i = 0; i < BUFFER_COUNT; i++) {
        for (int p = 0; p < VIDEO_MAX_PLANES; p++) {
            if (s->buffers[i].start[p] && s->buffers[i].length[p] > 0) {
   		// TODO:
            }
        }
    }

    for (int i = 0; i < BUFFER_COUNT; i++) {
        void *mapped0 = s->buffers[i].start[0];
        size_t len0 = s->buffers[i].length[0];
        if (mapped0 && len0 > 0) {
            munmap(mapped0, len0);
            s->buffers[i].start[0] = NULL;
            s->buffers[i].length[0] = 0;
        }

        for (int p = 1; p < VIDEO_MAX_PLANES; p++) {
            void *sp = s->buffers[i].start[p];
            size_t lp = s->buffers[i].length[p];
            if (sp && sp != mapped0 && lp > 0) {
                munmap(sp, lp);
            }
            s->buffers[i].start[p] = NULL;
            s->buffers[i].length[p] = 0;
        }
    }
}

static void nv12_to_bgra(uint8_t *dst, uint8_t *y_plane, uint8_t *uv_plane)
{
    for (int j = 0; j < HEIGHT; j++) {
        for (int i = 0; i < WIDTH; i++) {
            int y = y_plane[j * WIDTH + i];
            int uv_index = (j / 2) * WIDTH + (i & ~1);
            int u = uv_plane[uv_index] - 128;
            int v = uv_plane[uv_index + 1] - 128;

            int c = y - 16;
            int d = u;
            int e = v;

            int r = (298 * c + 409 * e + 128) >> 8;
            int g = (298 * c - 100 * d - 208 * e + 128) >> 8;
            int b = (298 * c + 516 * d + 128) >> 8;

            if (r < 0) r = 0; if (r > 255) r = 255;
            if (g < 0) g = 0; if (g > 255) g = 255;
            if (b < 0) b = 0; if (b > 255) b = 255;

            int idx = (j * WIDTH + i) * 4;
            dst[idx + 0] = (uint8_t)b;
            dst[idx + 1] = (uint8_t)g;
            dst[idx + 2] = (uint8_t)r;
            dst[idx + 3] = 255;
        }
    }
}

static void *mplane_create(obs_data_t *settings, obs_source_t *source)
{
    (void)settings;
    struct v4l2_mplane_source *s = (v4l2_mplane_source *)bzalloc(sizeof(*s));
    if (!s) return NULL;

    s->source = source;
    s->fd = -1;
    s->texture = NULL;
    s->rgb_front = NULL;
    s->rgb_back = NULL;
    s->new_frame = false;
    pthread_mutex_init(&s->frame_lock, NULL);

    s->fd = open(DEV_PATH, O_RDWR);
    if (s->fd < 0) {
        blog(LOG_ERROR, "Failed to open %s: %s", DEV_PATH, strerror(errno));
        bfree(s);
        return NULL;
    }

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.width = WIDTH;
    fmt.fmt.pix_mp.height = HEIGHT;
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
    if (ioctl(s->fd, VIDIOC_S_FMT, &fmt) < 0) {
        blog(LOG_WARNING, "VIDIOC_S_FMT failed: %s", strerror(errno));
    }

    // blog(LOG_INFO, "Negotiated format: pixelformat=0x%08x width=%u height=%u num_planes=%u",
    //   fmt.fmt.pix_mp.pixelformat, fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height,
    //   fmt.fmt.pix_mp.num_planes);

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(s->fd, VIDIOC_REQBUFS, &req) < 0) {
        blog(LOG_ERROR, "VIDIOC_REQBUFS failed: %s", strerror(errno));
        close(s->fd);
        bfree(s);
        return NULL;
    }

    for (int i = 0; i < BUFFER_COUNT; i++) {
        struct v4l2_buffer buf;
        struct v4l2_plane planes[VIDEO_MAX_PLANES];
        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.length = VIDEO_MAX_PLANES;
        buf.m.planes = planes;

        if (ioctl(s->fd, VIDIOC_QUERYBUF, &buf) < 0) {
            blog(LOG_ERROR, "VIDIOC_QUERYBUF failed: %s", strerror(errno));
            free_mapped_buffers(s);
            close(s->fd);
            bfree(s);
            return NULL;
        }

        int planes_count = buf.length;
        if (planes_count <= 0) {
            blog(LOG_ERROR, "Driver returned zero planes for buffer %d", i);
            free_mapped_buffers(s);
            close(s->fd);
            bfree(s);
            return NULL;
        }

        if (planes_count >= 2) {
            for (int p = 0; p < planes_count && p < VIDEO_MAX_PLANES; p++) {
                size_t plen = buf.m.planes[p].length;
                off_t off = buf.m.planes[p].m.mem_offset;
                void *mapped = mmap(NULL, plen, PROT_READ | PROT_WRITE, MAP_SHARED, s->fd, off);
                if (mapped == MAP_FAILED) {
                    blog(LOG_ERROR, "mmap failed for buffer %d plane %d: %s", i, p, strerror(errno));
                    s->buffers[i].start[p] = NULL;
                    s->buffers[i].length[p] = 0;
                    free_mapped_buffers(s);
                    close(s->fd);
                    bfree(s);
                    return NULL;
                }
                s->buffers[i].start[p] = mapped;
                s->buffers[i].length[p] = plen;
            }
        } else {
            size_t total_len = buf.m.planes[0].length;
            off_t off = buf.m.planes[0].m.mem_offset;
            void *mapped = mmap(NULL, total_len, PROT_READ | PROT_WRITE, MAP_SHARED, s->fd, off);
            if (mapped == MAP_FAILED) {
                blog(LOG_ERROR, "mmap failed for buffer %d (single-plane): %s", i, strerror(errno));
                free_mapped_buffers(s);
                close(s->fd);
                bfree(s);
                return NULL;
            }

            s->buffers[i].start[0] = mapped;
            s->buffers[i].length[0] = total_len;

            size_t y_size = (size_t)WIDTH * (size_t)HEIGHT;
            if (fmt.fmt.pix_mp.plane_fmt[0].bytesperline > 0) {
                size_t stride = fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
                size_t computed = stride * (size_t)HEIGHT;
                if (computed <= total_len) {
                    y_size = computed;
                }
            }

            size_t uv_offset = y_size;
            if (uv_offset >= total_len) {
                blog(LOG_ERROR, "Single-plane buffer too small for NV12 layout (total=%zu, y=%zu)", total_len, y_size);
                munmap(mapped, total_len);
                free_mapped_buffers(s);
                close(s->fd);
                bfree(s);
                return NULL;
            }

            s->buffers[i].start[1] = (uint8_t *)mapped + uv_offset;
            s->buffers[i].length[1] = total_len - uv_offset;
        }

        struct v4l2_buffer qbuf;
        struct v4l2_plane qplanes[VIDEO_MAX_PLANES];
        memset(&qbuf, 0, sizeof(qbuf));
        memset(qplanes, 0, sizeof(qplanes));
        qbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        qbuf.memory = V4L2_MEMORY_MMAP;
        qbuf.index = i;
        qbuf.m.planes = qplanes;
        qbuf.length = planes_count;
        if (ioctl(s->fd, VIDIOC_QBUF, &qbuf) < 0) {
            blog(LOG_ERROR, "VIDIOC_QBUF failed for buffer %d: %s", i, strerror(errno));
            free_mapped_buffers(s);
            close(s->fd);
            bfree(s);
            return NULL;
        }
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(s->fd, VIDIOC_STREAMON, &type) < 0) {
        blog(LOG_ERROR, "VIDIOC_STREAMON failed: %s", strerror(errno));
        free_mapped_buffers(s);
        close(s->fd);
        bfree(s);
        return NULL;
    }

    size_t rgb_size = (size_t)WIDTH * (size_t)HEIGHT * 4;
    s->rgb_front = (uint8_t *)bzalloc(rgb_size);
    s->rgb_back = (uint8_t *)bzalloc(rgb_size);
    if (!s->rgb_front || !s->rgb_back) {
        blog(LOG_ERROR, "Failed to allocate rgb buffers");
        if (s->rgb_front) bfree(s->rgb_front);
        if (s->rgb_back) bfree(s->rgb_back);
        free_mapped_buffers(s);
        close(s->fd);
        bfree(s);
        return NULL;
    }
    s->new_frame = false;

    obs_enter_graphics();
    const uint8_t *init_data[1] = { (const uint8_t *)s->rgb_front };
    s->texture = gs_texture_create(WIDTH, HEIGHT, GS_BGRA, 1, init_data, GS_DYNAMIC);
    obs_leave_graphics();

    if (!s->texture) {
        blog(LOG_ERROR, "gs_texture_create failed");
        bfree(s->rgb_front);
        bfree(s->rgb_back);
        free_mapped_buffers(s);
        close(s->fd);
        bfree(s);
        return NULL;
    }

    (void)source;
    return s;
}

static void mplane_tick(void *data, float seconds)
{
    (void)seconds;
    struct v4l2_mplane_source *s = (struct v4l2_mplane_source *)data;
    if (!s || s->fd < 0) return;

    struct v4l2_buffer buf;
    struct v4l2_plane planes[VIDEO_MAX_PLANES];
    memset(&buf, 0, sizeof(buf));
    memset(planes, 0, sizeof(planes));

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.m.planes = planes;
    buf.length = VIDEO_MAX_PLANES;

    if (ioctl(s->fd, VIDIOC_DQBUF, &buf) == 0) {
        int idx = buf.index;
        if (idx < 0 || idx >= BUFFER_COUNT) {
            blog(LOG_ERROR, "VIDIOC_DQBUF returned invalid index %d", idx);
        } else {
            uint8_t *y_plane = (uint8_t *)s->buffers[idx].start[0];
            uint8_t *uv_plane = NULL;

            if (s->buffers[idx].start[1]) {
                uv_plane = (uint8_t *)s->buffers[idx].start[1];
            } else {
                uv_plane = (uint8_t *)s->buffers[idx].start[0] + (WIDTH * HEIGHT);
            }

            if (y_plane && uv_plane && s->rgb_back) {
                nv12_to_bgra(s->rgb_back, y_plane, uv_plane);
                pthread_mutex_lock(&s->frame_lock);
                uint8_t *tmp = s->rgb_front;
                s->rgb_front = s->rgb_back;
                s->rgb_back = tmp;
                s->new_frame = true;
                pthread_mutex_unlock(&s->frame_lock);
            } else {
                blog(LOG_WARNING, "Missing planes or back buffer; skipping conversion");
            }
        }

        struct v4l2_buffer qbuf;
        struct v4l2_plane qplanes[VIDEO_MAX_PLANES];
        memset(&qbuf, 0, sizeof(qbuf));
        memset(qplanes, 0, sizeof(qplanes));
        qbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        qbuf.memory = V4L2_MEMORY_MMAP;
        qbuf.index = buf.index;
        qbuf.m.planes = qplanes;
        qbuf.length = buf.length;
        if (ioctl(s->fd, VIDIOC_QBUF, &qbuf) < 0) {
            blog(LOG_ERROR, "VIDIOC_QBUF failed after DQBUF: %s", strerror(errno));
        }
    } else {
	    blog(LOG_DEBUG, "VIDIOC_DQBUF returned error: %s", strerror(errno));
    }
}

static void mplane_render(void *data, gs_effect_t *effect)
{
    struct v4l2_mplane_source *s = (struct v4l2_mplane_source *)data;
    if (!s) return;
    if (!s->texture) return;

    bool do_upload = false;
    pthread_mutex_lock(&s->frame_lock);
    if (s->new_frame) {
        s->new_frame = false;
        do_upload = true;
    }
    pthread_mutex_unlock(&s->frame_lock);

    if (do_upload) {
        gs_texture_set_image(s->texture, (const uint8_t *)s->rgb_front, (uint32_t)(WIDTH * 4), false);
    }

    gs_effect_set_texture(gs_effect_get_param_by_name(effect, "image"), s->texture);
    gs_draw_sprite(s->texture, 0, WIDTH, HEIGHT);
}

static void mplane_destroy(void *data)
{
    struct v4l2_mplane_source *s = (struct v4l2_mplane_source *)data;
    if (!s) return;

    if (s->fd >= 0) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        ioctl(s->fd, VIDIOC_STREAMOFF, &type);
    }

    free_mapped_buffers(s);

    if (s->fd >= 0) {
        close(s->fd);
        s->fd = -1;
    }

    if (s->texture) {
        obs_enter_graphics();
        gs_texture_destroy(s->texture);
        obs_leave_graphics();
        s->texture = NULL;
    }

    if (s->rgb_front) {
        bfree(s->rgb_front);
        s->rgb_front = NULL;
    }
    if (s->rgb_back) {
        bfree(s->rgb_back);
        s->rgb_back = NULL;
    }

    pthread_mutex_destroy(&s->frame_lock);
    bfree(s);
}

static struct obs_source_info mplane_source_info = {
    .id = "v4l2_mplane_source",
    .type = OBS_SOURCE_TYPE_INPUT,
    .output_flags = OBS_SOURCE_VIDEO,
    .get_name = mplane_get_name,
    .create = mplane_create,
    .destroy = mplane_destroy,
    .get_width = mplane_width,
    .get_height = mplane_height,
    .video_tick = mplane_tick,
    .video_render = mplane_render,
};

bool obs_module_load(void)
{
    obs_register_source(&mplane_source_info);
    return true;
}
