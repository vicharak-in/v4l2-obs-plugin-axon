#include <obs-module.h>
#include <util/platform.h>
#include <util/profiler.h>
#include <plugin-support.h>
#include <linux/videodev2.h>
#include <alsa/asoundlib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <pthread.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <CL/cl.h>

OBS_DECLARE_MODULE()
OBS_MODULE_USE_DEFAULT_LOCALE(PLUGIN_NAME, "en-US")

MODULE_EXPORT const char* obs_module_description(void)
{
    return "V4L2 mplane NV12 camera capture plugin";
}

#define BUFFER_COUNT 4
#define AUDIO_SAMPLE_RATE 48000
#define AUDIO_CHANNELS 2
#define AUDIO_FORMAT SND_PCM_FORMAT_S16_LE
#define AUDIO_FRAMES 1024

struct buffer {
    void*  start[VIDEO_MAX_PLANES];
    size_t length[VIDEO_MAX_PLANES];
};

struct axon_cl_ctx {
    bool             initialized;
    cl_platform_id   platform;
    cl_device_id     device;
    cl_context       context;
    cl_command_queue queue;
    cl_program       program;
    cl_kernel        kernel;

    cl_mem dev_y;
    cl_mem dev_uv;
    cl_mem dev_out;

    size_t width;
    size_t height;
};

struct v4l2_mplane_source {
    obs_source_t* source;

    int fd;
    int width;
    int height;

    /* negotiated strides (bytes per line) */
    int y_stride;
    int uv_stride;

    /* negotiated plane count (1 or 2 commonly) */
    int num_planes;

    /* driver-granted buffer count (<= BUFFER_COUNT) */
    int num_buffers;

    char          device_path[100];
    struct buffer buffers[BUFFER_COUNT];

    gs_texture_t* texture;
    uint8_t*      rgb_front;
    uint8_t*      rgb_back;
    bool          new_frame;

    gs_texture_t* tex_y;
    gs_texture_t* tex_uv;
    uint8_t*      tex_y_front;
    uint8_t*      tex_y_back;
    uint8_t*      tex_uv_front;
    uint8_t*      tex_uv_back;

    /* protect front/back swap + fd ops during update */
    pthread_mutex_t frame_lock;
    pthread_mutex_t io_lock;
    volatile bool   reconfiguring;

    /* audio state */
    snd_pcm_t*    pcm_handle;
    char          alsa_device[64];
    pthread_t     audio_thread;
    volatile bool audio_running;

    struct axon_cl_ctx clctx;
};

static const char* nv12_to_bgra_kernel_src =
    "__kernel void nv12_to_bgra(                                                       \n"
    "    __global const uchar *src_y, int stride_y,                                    \n"
    "    __global const uchar *src_uv, int stride_uv,                                  \n"
    "    __global uchar4 *dst, int dst_stride_bytes,                                   \n"
    "    int width, int height)                                                        \n"
    "{                                                                                 \n"
    "    int x = get_global_id(0);                                                     \n"
    "    int y = get_global_id(1);                                                     \n"
    "                                                                                  \n"
    "    if (x >= width || y >= height) return;                                        \n"
    "                                                                                  \n"
    "    /* Y index */                                                                 \n"
    "    int y_idx = y * stride_y + x;                                                 \n"
    "    uchar Y = src_y[y_idx];                                                       \n"
    "                                                                                  \n"
    "    /* UV are subsampled 2x2: compute UV index for this pixel */                  \n"
    "    int uv_x = (x >> 1) << 1; /* same as (x/2)*2 */                               \n"
    "    int uv_y = (y >> 1);                                                          \n"
    "    int uv_idx = uv_y * stride_uv + uv_x;                                         \n"
    "    uchar U = src_uv[uv_idx + 0]; /* U */                                         \n"
    "    uchar V = src_uv[uv_idx + 1]; /* V */                                         \n"
    "                                                                                  \n"
    "    /* convert to float and perform BT.601 style conversion (full-range adjust) */\n"
    "    float yf = (float)Y - 16.0f;                                                  \n"
    "    float uf = (float)U - 128.0f;                                                 \n"
    "    float vf = (float)V - 128.0f;                                                 \n"
    "                                                                                  \n"
    "    /* coefficients: R = 1.164*y + 1.596*v; G = 1.164*y - 0.391*u - 0.813*v; B = 1.164*y + "
    "2.018*u */\n"
    "    float c = 1.164382f * yf;                                                     \n"
    "    float rf = c + 1.596027f * vf;                                                \n"
    "    float gf = c - 0.391762f * uf - 0.812968f * vf;                               \n"
    "    float bf = c + 2.017232f * uf;                                                \n"
    "                                                                                  \n"
    "    /* saturate to [0,255] and convert to uchar */                                \n"
    "    uchar r = (uchar) clamp((int)floor(rf + 0.5f), 0, 255);                       \n"
    "    uchar g = (uchar) clamp((int)floor(gf + 0.5f), 0, 255);                       \n"
    "    uchar b = (uchar) clamp((int)floor(bf + 0.5f), 0, 255);                       \n"
    "                                                                                  \n"
    "    /* compute destination index (dst is uchar4*); dst_stride_bytes in bytes per row */\n"
    "    int dst_pitch = dst_stride_bytes / 4;                                         \n"
    "    int out_idx = y * dst_pitch + x;                                              \n"
    "                                                                                  \n"
    "    dst[out_idx] = (uchar4)(b, g, r, (uchar)255);                                 \n"
    "}                                                                                 \n";

static const char* nv24_to_bgra_kernel_src =
    "__kernel void nv24_to_bgra(                                                          \n"
    "    __global const uchar *src_y,  int stride_y,                                      \n"
    "    __global const uchar *src_uv, int stride_uv,                                     \n"
    "    __global uchar4 *dst,         int dst_stride,                                    \n"
    "    int width, int height)                                                           \n"
    "{                                                                                    \n"
    "    int x = get_global_id(0) * 2;                                                    \n"
    "    int y = get_global_id(1);                                                        \n"
    "                                                                                     \n"
    "    if (x + 1 >= width || y >= height) return;                                       \n"
    "                                                                                     \n"
    "    int y_idx  = y * stride_y + x;                                                   \n"
    "    int uv_idx = y * stride_uv + x * 2;                                              \n"
    "                                                                                     \n"
    "    uchar2 Y   = vload2(0, &src_y[y_idx]);                                           \n"
    "    uchar4 UV  = vload4(0, &src_uv[uv_idx]);                                         \n"
    "                                                                                     \n"
    "    float2 C = convert_float2(Y) - 16.0f;                                            \n"
    "    float2 U = convert_float2((uchar2)(UV.s0, UV.s2)) - 128.0f;                      \n"
    "    float2 V = convert_float2((uchar2)(UV.s1, UV.s3)) - 128.0f;                      \n"
    "                                                                                     \n"
    "    float2 R = (298.0f * C + 459.0f * V) * 0.00390625f; /* /256 */                   \n"
    "    float2 G = (298.0f * C - 55.0f  * U - 136.0f * V) * 0.00390625f;                 \n"
    "    float2 B = (298.0f * C + 541.0f * U) * 0.00390625f;                              \n"
    "                                                                                     \n"
    "    uchar2 Ru = convert_uchar2_sat(R);                                               \n"
    "    uchar2 Gu = convert_uchar2_sat(G);                                               \n"
    "    uchar2 Bu = convert_uchar2_sat(B);                                               \n"
    "                                                                                     \n"
    "    int out_idx = y * (dst_stride / 4) + x;                                          \n"
    "                                                                                     \n"
    "    dst[out_idx + 0] = (uchar4)(Bu.s0, Gu.s0, Ru.s0, (uchar)255);                    \n"
    "    dst[out_idx + 1] = (uchar4)(Bu.s1, Gu.s1, Ru.s1, (uchar)255);                    \n"
    "}                                                                                    \n";

static void axon_cl_init(struct axon_cl_ctx* cl, int width, int height, char* device_path)
{
    cl_int err;
    cl->initialized = false;
    cl->width       = width;
    cl->height      = height;

    err = clGetPlatformIDs(1, &cl->platform, NULL);
    err = clGetDeviceIDs(cl->platform, CL_DEVICE_TYPE_GPU, 1, &cl->device, NULL);

    cl->context = clCreateContext(NULL, 1, &cl->device, NULL, NULL, &err);
#if defined(CL_VERSION_2_0)
    const cl_queue_properties props[] = {CL_QUEUE_PROPERTIES, 0, 0};
    cl->queue = clCreateCommandQueueWithProperties(cl->context, cl->device, props, &err);
#else
    cl->queue = clCreateCommandQueue(cl->context, cl->device, 0, &err);
#endif

    if (strstr(device_path, "video11"))
        cl->program =
            clCreateProgramWithSource(cl->context, 1, &nv12_to_bgra_kernel_src, NULL, &err);
    else if (strstr(device_path, "video20"))
        cl->program =
            clCreateProgramWithSource(cl->context, 1, &nv24_to_bgra_kernel_src, NULL, &err);

    err = clBuildProgram(cl->program, 0, NULL, "", NULL, NULL);
    if (err != CL_SUCCESS) {
        size_t log_size = 0;
        clGetProgramBuildInfo(cl->program, cl->device, CL_PROGRAM_BUILD_LOG, 0, NULL, &log_size);
        char* log = (char*) malloc(log_size + 1);
        clGetProgramBuildInfo(cl->program, cl->device, CL_PROGRAM_BUILD_LOG, log_size, log, NULL);
        log[log_size] = '\0';
        blog(LOG_ERROR, "[axon] OpenCL build failed:\n%s", log);
        free(log);
    }

    if (strstr(device_path, "video11"))
        cl->kernel = clCreateKernel(cl->program, "nv12_to_bgra", &err);
    else if (strstr(device_path, "video20"))
        cl->kernel = clCreateKernel(cl->program, "nv24_to_bgra", &err);

    size_t y_bytes   = (size_t) width * (size_t) height;
    size_t uv_bytes  = (size_t) width * (size_t) height * 2;
    size_t out_bytes = (size_t) width * (size_t) height * 4;

    cl->dev_y   = clCreateBuffer(cl->context, CL_MEM_READ_ONLY, y_bytes, NULL, &err);
    cl->dev_uv  = clCreateBuffer(cl->context, CL_MEM_READ_ONLY, uv_bytes, NULL, &err);
    cl->dev_out = clCreateBuffer(cl->context, CL_MEM_WRITE_ONLY, out_bytes, NULL, &err);

    cl->initialized = true;
    blog(LOG_INFO, "[axon] OpenCL initialized for %dx%d", width, height);
}

static void axon_cl_convert_nv12_to_bgra(struct axon_cl_ctx* cl, const uint8_t* src_y, int stride_y,
                                         const uint8_t* src_uv, int stride_uv, uint8_t* dst,
                                         int dst_stride_bytes, int width, int height)
{
    if (!cl->initialized)
        return;

    cl_int err;
    size_t y_bytes   = (size_t) stride_y * (size_t) height;
    size_t uv_rows   = (size_t) ((height + 1) / 2);
    size_t uv_bytes  = (size_t) stride_uv * uv_rows;
    size_t out_bytes = (size_t) dst_stride_bytes * (size_t) height;

    err = clEnqueueWriteBuffer(cl->queue, cl->dev_y, CL_FALSE, 0, y_bytes, src_y, 0, NULL, NULL);
    if (err != CL_SUCCESS) {
        blog(LOG_ERROR, "[axon] clEnqueueWriteBuffer(Y) failed: %d", err);
        return;
    }
    err = clEnqueueWriteBuffer(cl->queue, cl->dev_uv, CL_FALSE, 0, uv_bytes, src_uv, 0, NULL, NULL);
    if (err != CL_SUCCESS) {
        blog(LOG_ERROR, "[axon] clEnqueueWriteBuffer(UV) failed: %d", err);
        return;
    }

    err = clSetKernelArg(cl->kernel, 0, sizeof(cl_mem), &cl->dev_y);
    err = clSetKernelArg(cl->kernel, 1, sizeof(int), &stride_y);
    err = clSetKernelArg(cl->kernel, 2, sizeof(cl_mem), &cl->dev_uv);
    err = clSetKernelArg(cl->kernel, 3, sizeof(int), &stride_uv);
    err = clSetKernelArg(cl->kernel, 4, sizeof(cl_mem), &cl->dev_out);
    err = clSetKernelArg(cl->kernel, 5, sizeof(int), &dst_stride_bytes);
    err = clSetKernelArg(cl->kernel, 6, sizeof(int), &width);
    err = clSetKernelArg(cl->kernel, 7, sizeof(int), &height);

    if (err != CL_SUCCESS) {
        blog(LOG_ERROR, "[axon] clSetKernelArg failed: %d", err);
        return;
    }

    size_t global[2] = {(size_t) width, (size_t) height};
    size_t local[2]  = {16, 8};

    err = clEnqueueNDRangeKernel(cl->queue, cl->kernel, 2, NULL, global, local, 0, NULL, NULL);
    if (err != CL_SUCCESS) {
        blog(LOG_ERROR, "[axon] Kernel launch failed: %d", err);
        return;
    }

    cl_event evt = NULL;
    err = clEnqueueReadBuffer(cl->queue, cl->dev_out, CL_FALSE, 0, out_bytes, dst, 0, NULL, &evt);
    if (err != CL_SUCCESS) {
        blog(LOG_ERROR, "[axon] clEnqueueReadBuffer failed: %d", err);
        return;
    }

    clFlush(cl->queue);
    // clWaitForEvents(1, &evt);
    clReleaseEvent(evt);
}

static void axon_cl_convert_nv24_to_bgra(struct axon_cl_ctx* cl, const uint8_t* src_y, int stride_y,
                                         const uint8_t* src_uv, int stride_uv, uint8_t* dst,
                                         int dst_stride, int width, int height)
{
    if (!cl->initialized)
        return;

    cl_int err;
    size_t y_bytes   = (size_t) width * (size_t) height;
    size_t uv_bytes  = (size_t) width * (size_t) height * 2;
    size_t out_bytes = (size_t) width * (size_t) height * 4;

    clEnqueueWriteBuffer(cl->queue, cl->dev_y, CL_FALSE, 0, y_bytes, src_y, 0, NULL, NULL);
    clEnqueueWriteBuffer(cl->queue, cl->dev_uv, CL_FALSE, 0, uv_bytes, src_uv, 0, NULL, NULL);

    clSetKernelArg(cl->kernel, 0, sizeof(cl_mem), &cl->dev_y);
    clSetKernelArg(cl->kernel, 1, sizeof(int), &stride_y);
    clSetKernelArg(cl->kernel, 2, sizeof(cl_mem), &cl->dev_uv);
    clSetKernelArg(cl->kernel, 3, sizeof(int), &stride_uv);
    clSetKernelArg(cl->kernel, 4, sizeof(cl_mem), &cl->dev_out);
    clSetKernelArg(cl->kernel, 5, sizeof(int), &dst_stride);
    clSetKernelArg(cl->kernel, 6, sizeof(int), &width);
    clSetKernelArg(cl->kernel, 7, sizeof(int), &height);

    size_t global[2] = {(size_t) width, (size_t) height};
    size_t local[2]  = {16, 8};

    err = clEnqueueNDRangeKernel(cl->queue, cl->kernel, 2, NULL, global, local, 0, NULL, NULL);
    if (err != CL_SUCCESS)
        blog(LOG_ERROR, "[axon] Kernel launch failed: %d", err);

    cl_event evt = NULL;
    clEnqueueReadBuffer(cl->queue, cl->dev_out, CL_FALSE, 0, out_bytes, dst, 0, NULL, &evt);

    clFlush(cl->queue);
    // clWaitForEvents(1, &evt);

    clReleaseEvent(evt);
}

static void zero_buffers(struct v4l2_mplane_source* s)
{
    memset(s->buffers, 0, sizeof(s->buffers));
}

static void stop_streaming(int fd)
{
    if (fd >= 0) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        ioctl(fd, VIDIOC_STREAMOFF, &type);
    }
}

static void free_mapped_buffers(struct v4l2_mplane_source* s)
{
    if (!s)
        return;

    for (int i = 0; i < s->num_buffers; i++) {
        void*  mapped0 = s->buffers[i].start[0];
        size_t len0    = s->buffers[i].length[0];

        if (mapped0 && len0 > 0) {
            munmap(mapped0, len0);
        }

        for (int p = 1; p < VIDEO_MAX_PLANES; p++) {
            void*  sp = s->buffers[i].start[p];
            size_t lp = s->buffers[i].length[p];
            if (sp && lp > 0 && sp != mapped0) {
                munmap(sp, lp);
            }
        }
    }

    zero_buffers(s);
    s->num_buffers = 0;
}

static void destroy_texture(struct v4l2_mplane_source* s)
{
    if (!s || !s->texture)
        return;
    obs_enter_graphics();
    gs_texture_destroy(s->texture);
    obs_leave_graphics();
    s->texture = NULL;
}

static void destroy_rgb(struct v4l2_mplane_source* s)
{
    if (s->rgb_front) {
        bfree(s->rgb_front);
        s->rgb_front = NULL;
    }
    if (s->rgb_back) {
        bfree(s->rgb_back);
        s->rgb_back = NULL;
    }
}

static bool alloc_rgb_and_texture(struct v4l2_mplane_source* s)
{
    size_t rgb_size = (size_t) s->y_stride * (size_t) s->height * 4;
    destroy_rgb(s);
    s->rgb_front = (uint8_t*) bzalloc(rgb_size);
    s->rgb_back  = (uint8_t*) bzalloc(rgb_size);

    if (!s->rgb_front || !s->rgb_back) {
        blog(LOG_ERROR, "[axon] Failed to allocate RGB buffers (%dx%d)", s->width, s->height);
        destroy_rgb(s);
        return false;
    }
    s->new_frame = false;

    destroy_texture(s);

    obs_enter_graphics();
    const uint8_t* init_data[1] = {(const uint8_t*) s->rgb_front};
    s->texture = gs_texture_create(s->width, s->height, GS_BGRA, 1, init_data, GS_DYNAMIC);

    obs_leave_graphics();

    if (!s->texture) {
        blog(LOG_ERROR, "[axon] gs_texture_create failed (%dx%d)", s->width, s->height);
        destroy_rgb(s);
        return false;
    }

    return true;
}

static void* audio_thread_fn(void* arg)
{
    struct v4l2_mplane_source* s        = (v4l2_mplane_source*) arg;
    size_t                     buf_size = AUDIO_FRAMES * AUDIO_CHANNELS * sizeof(int16_t);
    int16_t*                   buffer   = (int16_t*) bmalloc(buf_size);

    while (s->audio_running) {
        if (!s->pcm_handle) {
            break;
        }

        snd_pcm_sframes_t frames_read = snd_pcm_readi(s->pcm_handle, buffer, AUDIO_FRAMES);
        // blog(LOG_INFO, "[audio] frames_read=%ld", (long) frames_read);
        if (frames_read < 0) {
            snd_pcm_prepare(s->pcm_handle);
            continue;
        } else {
            int16_t max_sample = 0;
            for (int i = 0; i < frames_read * AUDIO_CHANNELS; i++) {
                if (abs(buffer[i]) > max_sample)
                    max_sample = abs(buffer[i]);
            }
            // blog(LOG_INFO, "[audio] max sample = %d", max_sample);
        }

        float boost = 24.0f;
        for (int i = 0; i < frames_read * AUDIO_CHANNELS; i++) {
            int32_t tmp = (int32_t) buffer[i] * boost;
            if (tmp > 32767)
                tmp = 32767;
            if (tmp < -32768)
                tmp = -32768;
            buffer[i] = (int16_t) tmp;
        }

        struct obs_source_audio ad = {0};
        ad.data[0]                 = (uint8_t*) buffer;
        ad.frames                  = (uint32_t) frames_read;
        ad.samples_per_sec         = AUDIO_SAMPLE_RATE;
        ad.speakers                = SPEAKERS_STEREO;
        ad.format                  = AUDIO_FORMAT_16BIT;

        static uint64_t audio_start_ts    = 0;
        static uint64_t audio_frame_count = 0;

        if (audio_start_ts == 0)
            audio_start_ts = os_gettime_ns();

        ad.timestamp = audio_start_ts + (audio_frame_count * 1000000000ULL / AUDIO_SAMPLE_RATE);

        audio_frame_count += frames_read;

        // blog(LOG_INFO, "[audio] frames_read=%ld", (long) frames_read);

        obs_source_output_audio(s->source, &ad);
    }

    bfree(buffer);
    return NULL;
}

static bool start_device(struct v4l2_mplane_source* s)
{
    if (!s)
        return false;

    s->fd = open(s->device_path, O_RDWR | O_NONBLOCK);
    if (s->fd < 0) {
        blog(LOG_ERROR, "[axon] Failed to open %s: %s", s->device_path, strerror(errno));
        return false;
    }

    // audio setup
    snprintf(s->alsa_device, sizeof(s->alsa_device), "hw:0,0");
    s->pcm_handle    = NULL;
    s->audio_running = false;

    if (snd_pcm_open(&s->pcm_handle, s->alsa_device, SND_PCM_STREAM_CAPTURE, 0) < 0) {
        blog(LOG_ERROR, "Failed to open ALSA device");
        s->pcm_handle = NULL;
    } else {
        blog(LOG_INFO, "Opened ASLA device successfully");
        if (snd_pcm_set_params(s->pcm_handle, AUDIO_FORMAT, SND_PCM_ACCESS_RW_INTERLEAVED,
                               AUDIO_CHANNELS, AUDIO_SAMPLE_RATE, 1, 500000)) {
            blog(LOG_ERROR, "Failed to set ALSA params");
            return false;
        };
        snd_pcm_prepare(s->pcm_handle);
        snd_pcm_start(s->pcm_handle);

        s->audio_running = true;
        pthread_create(&s->audio_thread, NULL, audio_thread_fn, s);
    }

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type                   = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.width       = s->width;
    fmt.fmt.pix_mp.height      = s->height;
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;

    if (strstr(s->device_path, "video11")) {
        fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
    } else if (strstr(s->device_path, "video20")) {
        fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV24;
    }

    if (ioctl(s->fd, VIDIOC_S_FMT, &fmt) < 0) {
        blog(LOG_ERROR, "[axon] VIDIOC_S_FMT failed: %s", strerror(errno));
        return false;
    }

    s->width      = (int) fmt.fmt.pix_mp.width;
    s->height     = (int) fmt.fmt.pix_mp.height;
    s->num_planes = fmt.fmt.pix_mp.num_planes > 0 ? fmt.fmt.pix_mp.num_planes : 1;
    s->y_stride   = fmt.fmt.pix_mp.plane_fmt[0].bytesperline > 0
                        ? (int) fmt.fmt.pix_mp.plane_fmt[0].bytesperline
                        : s->width;
    if (s->num_planes >= 2) {
        s->uv_stride = fmt.fmt.pix_mp.plane_fmt[1].bytesperline > 0
                           ? (int) fmt.fmt.pix_mp.plane_fmt[1].bytesperline
                           : s->y_stride;
    } else {
        s->uv_stride = s->y_stride;
    }

    if (strstr(s->device_path, "video20")) {
        s->y_stride  = s->width;
        s->uv_stride = s->width * 2;
        blog(LOG_INFO, "width and height %d%d\n", s->width, s->height);
    }

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count  = BUFFER_COUNT;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(s->fd, VIDIOC_REQBUFS, &req) < 0) {
        blog(LOG_ERROR, "[axon] VIDIOC_REQBUFS failed: %s", strerror(errno));
        close(s->fd);
        s->fd = -1;
        return false;
    }
    if (req.count == 0) {
        blog(LOG_ERROR, "[axon] Driver returned zero buffers");
        close(s->fd);
        s->fd = -1;
        return false;
    }
    s->num_buffers = (int) req.count;
    zero_buffers(s);

    for (int i = 0; i < s->num_buffers; i++) {
        struct v4l2_buffer buf;
        struct v4l2_plane  planes[VIDEO_MAX_PLANES];
        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));

        buf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory   = V4L2_MEMORY_MMAP;
        buf.index    = i;
        buf.length   = VIDEO_MAX_PLANES;
        buf.m.planes = planes;

        if (ioctl(s->fd, VIDIOC_QUERYBUF, &buf) < 0) {
            blog(LOG_ERROR, "[axon] VIDIOC_QUERYBUF failed: %s", strerror(errno));
            free_mapped_buffers(s);
            close(s->fd);
            s->fd = -1;
            return false;
        }

        int planes_count = buf.length;
        if (planes_count <= 0)
            planes_count = 1;

        if (planes_count >= 2) {
            for (int p = 0; p < planes_count && p < VIDEO_MAX_PLANES; p++) {
                size_t plen   = buf.m.planes[p].length;
                off_t  off    = buf.m.planes[p].m.mem_offset;
                void*  mapped = mmap(NULL, plen, PROT_READ | PROT_WRITE, MAP_SHARED, s->fd, off);
                if (mapped == MAP_FAILED) {
                    blog(LOG_ERROR, "[axon] mmap failed: %s", strerror(errno));
                    free_mapped_buffers(s);
                    close(s->fd);
                    s->fd = -1;
                    return false;
                }
                s->buffers[i].start[p]  = mapped;
                s->buffers[i].length[p] = plen;
            }
        } else {
            size_t plen   = buf.m.planes[0].length;
            off_t  off    = buf.m.planes[0].m.mem_offset;
            void*  mapped = mmap(NULL, plen, PROT_READ | PROT_WRITE, MAP_SHARED, s->fd, off);
            if (mapped == MAP_FAILED) {
                blog(LOG_ERROR, "[axon] mmap failed (single-plane): %s", strerror(errno));
                free_mapped_buffers(s);
                close(s->fd);
                s->fd = -1;
                return false;
            }
            s->buffers[i].start[0]  = mapped;
            s->buffers[i].length[0] = plen;

            size_t y_bytes = (size_t) s->y_stride * (size_t) s->height;
            if (y_bytes >= plen) {
                blog(LOG_ERROR, "[axon] NV12 split exceeds buffer: total=%zu y=%zu", plen, y_bytes);
                free_mapped_buffers(s);
                close(s->fd);
                s->fd = -1;
                return false;
            }
            s->buffers[i].start[1]  = (uint8_t*) mapped + y_bytes;
            s->buffers[i].length[1] = plen - y_bytes;
        }

        struct v4l2_buffer qbuf;
        struct v4l2_plane  qplanes[VIDEO_MAX_PLANES];
        memset(&qbuf, 0, sizeof(qbuf));
        memset(qplanes, 0, sizeof(qplanes));
        qbuf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        qbuf.memory   = V4L2_MEMORY_MMAP;
        qbuf.index    = i;
        qbuf.m.planes = qplanes;
        qbuf.length   = (unsigned int) ((s->num_planes >= 2) ? 2 : 1);

        if (ioctl(s->fd, VIDIOC_QBUF, &qbuf) < 0) {
            blog(LOG_ERROR, "[axon] VIDIOC_QBUF failed: %s", strerror(errno));
            free_mapped_buffers(s);
            close(s->fd);
            s->fd = -1;
            return false;
        }
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(s->fd, VIDIOC_STREAMON, &type) < 0) {
        blog(LOG_ERROR, "[axon] VIDIOC_STREAMON failed: %s", strerror(errno));
        free_mapped_buffers(s);
        close(s->fd);
        s->fd = -1;
        return false;
    }

    if (!alloc_rgb_and_texture(s)) {
        stop_streaming(s->fd);
        free_mapped_buffers(s);
        close(s->fd);
        s->fd = -1;
        return false;
    }

    return true;
}

static void stop_device(struct v4l2_mplane_source* s)
{
    if (!s)
        return;

    if (s->audio_running) {
        s->audio_running = false;
        pthread_join(s->audio_thread, NULL);
    }

    if (s->pcm_handle) {
        snd_pcm_drop(s->pcm_handle);
        snd_pcm_close(s->pcm_handle);
        s->pcm_handle = NULL;
    }

    stop_streaming(s->fd);
    free_mapped_buffers(s);

    if (s->fd >= 0) {
        close(s->fd);
        s->fd = -1;
    }
}

static const char* mplane_get_name(void* unused)
{
    (void) unused;
    return "V4L2 axon camera";
}

static uint32_t mplane_width(void* data)
{
    return ((struct v4l2_mplane_source*) data)->width;
}

static uint32_t mplane_height(void* data)
{
    return ((struct v4l2_mplane_source*) data)->height;
}

static void* mplane_create(obs_data_t* settings, obs_source_t* source)
{
    struct v4l2_mplane_source* s = (struct v4l2_mplane_source*) bzalloc(sizeof(*s));
    if (!s)
        return NULL;

    s->source      = source;
    s->fd          = -1;
    s->texture     = NULL;
    s->rgb_front   = NULL;
    s->rgb_back    = NULL;
    s->new_frame   = false;
    s->y_stride    = 0;
    s->uv_stride   = 0;
    s->num_planes  = 0;
    s->num_buffers = 0;
    pthread_mutex_init(&s->frame_lock, NULL);
    pthread_mutex_init(&s->io_lock, NULL);
    s->reconfiguring = false;

    const char* dev     = obs_data_get_string(settings, "device_path");
    const char* res_str = obs_data_get_string(settings, "resolution");

    int w = 1920, h = 1080;
    if (res_str) {
        if (strcmp(res_str, "1920x1080") == 0) {
            w = 1920;
            h = 1080;
        } else if (strcmp(res_str, "1280x720") == 0) {
            w = 1280;
            h = 720;
        } else if (strcmp(res_str, "640x480") == 0) {
            w = 640;
            h = 480;
        } else if (strcmp(res_str, "4096x2160") == 0) {
            w = 4096;
            h = 2160;
        }
    }

    s->width  = w;
    s->height = h;
    snprintf(s->device_path, sizeof(s->device_path), "%s", (dev && dev[0]) ? dev : "/dev/video20");

    if (!start_device(s)) {
        destroy_texture(s);
        destroy_rgb(s);
        pthread_mutex_destroy(&s->io_lock);
        pthread_mutex_destroy(&s->frame_lock);
        bfree(s);
        return NULL;
    }

    return s;
}

static void mplane_update(void* data, obs_data_t* settings)
{
    struct v4l2_mplane_source* s = (struct v4l2_mplane_source*) data;
    if (!s)
        return;

    const char* dev     = obs_data_get_string(settings, "device_path");
    const char* res_str = obs_data_get_string(settings, "resolution");

    int w = s->width;
    int h = s->height;
    if (res_str) {
        if (!strcmp(res_str, "1920x1080")) {
            w = 1920;
            h = 1080;
        } else if (!strcmp(res_str, "1280x720")) {
            w = 1280;
            h = 720;
        } else if (!strcmp(res_str, "640x480")) {
            w = 640;
            h = 480;
        } else if (!strcmp(res_str, "4096x2160")) {
            w = 4096;
            h = 2160;
        }
    }

    const char* dev_safe    = (dev && dev[0]) ? dev : "/dev/video11";
    bool        dev_changed = strcmp(s->device_path, dev_safe) != 0;
    bool        res_changed = (w != s->width) || (h != s->height);

    if (!dev_changed && !res_changed) {
        blog(LOG_INFO, "[axon] Requested format %dx%d NV12 not available", s->width, s->height);
        return;
    }

    s->reconfiguring = true;

    pthread_mutex_lock(&s->io_lock);
    pthread_mutex_lock(&s->frame_lock);

    stop_device(s);
    os_sleep_ms(100);
    destroy_texture(s);
    destroy_rgb(s);

    snprintf(s->device_path, sizeof(s->device_path), "%s", dev_safe);
    s->width             = w;
    s->height            = h;
    s->clctx.initialized = false;

    bool started = start_device(s);
    if (started && s->rgb_front) {
        memset(s->rgb_front, 0, (size_t) s->width * (size_t) s->height * 4);
        s->new_frame = false;
        blog(LOG_INFO, "[axon] Reconfigured successfully to %dx%d", s->width, s->height);
    } else {
        blog(LOG_ERROR, "[axon] Reconfigure failed");
    }

    pthread_mutex_unlock(&s->frame_lock);
    pthread_mutex_unlock(&s->io_lock);

    s->reconfiguring = false;

    if (!started) {
        blog(LOG_ERROR, "[axon] Reconfigure failed");
    }
}

static void mplane_tick(void* data, float seconds)
{
    (void) seconds;
    struct v4l2_mplane_source* s = (struct v4l2_mplane_source*) data;
    if (!s)
        return;
    if (s->reconfiguring) {
        os_sleep_ms(1);
        return;
    }
    if (s->fd < 0)
        return;

    struct v4l2_buffer buf;
    struct v4l2_plane  planes[VIDEO_MAX_PLANES];
    memset(&buf, 0, sizeof(buf));
    memset(planes, 0, sizeof(planes));

    buf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory   = V4L2_MEMORY_MMAP;
    buf.m.planes = planes;
    buf.length   = VIDEO_MAX_PLANES;

    if (ioctl(s->fd, VIDIOC_DQBUF, &buf) < 0) {
        if (errno == EAGAIN) {
            return;
        }
        blog(LOG_DEBUG, "[axon] DQBUF error: %s", strerror(errno));
        return;
    }

    int idx = buf.index;
    if (idx >= 0 && idx < s->num_buffers) {
        const uint8_t* y_plane  = (const uint8_t*) s->buffers[idx].start[0];
        const uint8_t* uv_plane = NULL;

        if (s->buffers[idx].start[1]) {
            uv_plane = (const uint8_t*) s->buffers[idx].start[1];
        } else if (y_plane) {
            uv_plane = y_plane + (size_t) s->y_stride * (size_t) s->height;
        }

        if (y_plane && uv_plane && s->rgb_back) {
            if (strstr(s->device_path, "video11")) {
                if (!s->clctx.initialized)
                    axon_cl_init(&s->clctx, s->width, s->height, s->device_path);

                int dst_stride_bytes = s->width * 4;
                axon_cl_convert_nv12_to_bgra(&s->clctx, y_plane, s->y_stride, uv_plane,
                                             s->uv_stride, s->rgb_back, dst_stride_bytes, s->width,
                                             s->height);
            } else if (strstr(s->device_path, "video20")) {
                uv_plane = y_plane + (size_t) s->y_stride * (size_t) s->height;
                if (!s->clctx.initialized)
                    axon_cl_init(&s->clctx, s->width, s->height, s->device_path);

                axon_cl_convert_nv24_to_bgra(&s->clctx, y_plane, s->y_stride, uv_plane,
                                             s->uv_stride, s->rgb_back, s->y_stride * 4, s->width,
                                             s->height);
            }
            pthread_mutex_lock(&s->frame_lock);
            uint8_t* tmp = s->rgb_front;

            s->rgb_front = s->rgb_back;
            s->rgb_back  = tmp;
            s->new_frame = true;
            pthread_mutex_unlock(&s->frame_lock);
        }
    } else {
        blog(LOG_ERROR, "[axon] DQBUF invalid index %d", idx);
    }

    struct v4l2_buffer qbuf;
    struct v4l2_plane  qplanes[VIDEO_MAX_PLANES];
    memset(&qbuf, 0, sizeof(qbuf));
    memset(qplanes, 0, sizeof(qplanes));
    qbuf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    qbuf.memory   = V4L2_MEMORY_MMAP;
    qbuf.index    = buf.index;
    qbuf.m.planes = qplanes;
    qbuf.length   = buf.length;

    if (ioctl(s->fd, VIDIOC_QBUF, &qbuf) < 0) {
        blog(LOG_ERROR, "[axon] QBUF after DQBUF failed: %s", strerror(errno));
    }
}

static void log_current_thread(const char* prefix)
{
    pid_t tid = (pid_t) syscall(SYS_gettid);

    char name[32] = {0};
    pthread_getname_np(pthread_self(), name, sizeof(name));

    blog(LOG_INFO, "%s: tid=%d pthread_name=%s", prefix, (int) tid, name);
}

static void mplane_render(void* data, gs_effect_t* effect)
{
    struct v4l2_mplane_source* s = (struct v4l2_mplane_source*) data;
    if (!s || !s->texture)
        return;

    bool do_upload = false;
    pthread_mutex_lock(&s->frame_lock);
    if (s->new_frame) {
        s->new_frame = false;
        do_upload    = true;
    }
    pthread_mutex_unlock(&s->frame_lock);

    if (do_upload) {
        gs_texture_set_image(s->texture, (const uint8_t*) s->rgb_front, (uint32_t) (s->width * 4),
                             false);
    }

    gs_effect_set_texture(gs_effect_get_param_by_name(effect, "image"), s->texture);
    gs_draw_sprite(s->texture, 0, s->width, s->height);
}

static void mplane_get_defaults(obs_data_t* settings)
{
    obs_data_set_default_string(settings, "device_path", "/dev/video20");
    obs_data_set_default_string(settings, "resolution", "1920x1080");
}

static obs_properties_t* mplane_get_properties(void* unused)
{
    (void) unused;
    obs_properties_t* props = obs_properties_create();

    obs_property_t* res = obs_properties_add_list(props, "resolution", "Resolution",
                                                  OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_STRING);
    obs_property_list_add_string(res, "1280x720", "1280x720");
    obs_property_list_add_string(res, "1920x1080", "1920x1080");
    obs_property_list_add_string(res, "4096x2160", "4096x2160");

    obs_property_t* p = obs_properties_add_list(props, "device_path", "Video Device",
                                                OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_STRING);

    for (int i = 0; i < 25; i++) {
        char path[64];
        snprintf(path, sizeof(path), "/dev/video%d", i);
        if (access(path, F_OK) == 0) {
            obs_property_list_add_string(p, path, path);
        }
    }

    return props;
}

static void mplane_destroy(void* data)
{
    struct v4l2_mplane_source* s = (struct v4l2_mplane_source*) data;
    if (!s)
        return;

    s->reconfiguring = true;
    pthread_mutex_lock(&s->io_lock);

    stop_device(s);
    destroy_texture(s);
    destroy_rgb(s);

    pthread_mutex_unlock(&s->io_lock);

    pthread_mutex_destroy(&s->io_lock);
    pthread_mutex_destroy(&s->frame_lock);

    bfree(s);
}

static struct obs_source_info mplane_source_info = {
    .id             = "v4l2_mplane_source_axon",
    .type           = OBS_SOURCE_TYPE_INPUT,
    .output_flags   = OBS_SOURCE_VIDEO | OBS_SOURCE_AUDIO,
    .get_name       = mplane_get_name,
    .create         = mplane_create,
    .destroy        = mplane_destroy,
    .get_width      = mplane_width,
    .get_height     = mplane_height,
    .get_defaults   = mplane_get_defaults,
    .get_properties = mplane_get_properties,
    .update         = mplane_update,
    .video_tick     = mplane_tick,
    .video_render   = mplane_render,
    .icon_type      = OBS_ICON_TYPE_CAMERA,
};

bool obs_module_load(void)
{
    blog(LOG_INFO, "[v4l2 axon camera plugin]: plugin loaded successfully");
    obs_register_source(&mplane_source_info);
    return true;
}
