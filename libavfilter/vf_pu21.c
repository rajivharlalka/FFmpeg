
#include "libavfilter/avfilter.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/mem.h"
#include "libavutil/imgutils.h"
#include "libavfilter/formats.h"
#include "libavfilter/internal.h"
#include "video.h"


typedef struct PU21Context {
  const AVClass* class;
  double L_min, L_max;
  char* type;
  int multiplier;

  int depth;
  int planewidth[4];
  int planeheight[4];
  float* buffer;
  int nb_planes;
} PU21Context;

#define OFFSET(x) offsetof(PU21Context, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM
static const AVOption pu21_options[] = {
    { "type", "Set the type of encoding", OFFSET(type), AV_OPT_TYPE_STRING, {.str = "banding_glare"}, 0, 0, FLAGS }, // options can be banding, banding_glare, peaks, peaks_glare
    { "L_min", "Set the minimum luminance value", OFFSET(L_min), AV_OPT_TYPE_DOUBLE, {.dbl = 0.01}, 0.005, 10000, FLAGS },
    { "L_max", "Set the maximum luminance value", OFFSET(L_max), AV_OPT_TYPE_DOUBLE, {.dbl = 1000}, 0.005, 10000, FLAGS },
    { "multiplier", "Set the parameters for the encoding", OFFSET(multiplier), AV_OPT_TYPE_INT, {.i64 = 1}, 1, 10000, FLAGS},
    { NULL }
};

static const float PU21_MATRICES[4][7] = {
  // Reference: "PU21: A novel perceptually uniform encoding for adapting existing quality metrics for HDR"
  // Rafał K. Mantiuk and M. Azimi, Picture Coding Symposium 2021
  // https://github.com/gfxdisp/pu21
{1.070275272f, 0.4088273932f, 0.153224308f, 0.2520326168f, 1.063512885f, 1.14115047f, 521.4527484f},  // BANDING
{0.353487901f, 0.3734658629f, 8.277049286e-05f, 0.9062562627f, 0.09150303166f, 0.9099517204f, 596.3148142f},  // BANDING_GLARE
{1.043882782f, 0.6459495343f, 0.3194584211f, 0.374025247f, 1.114783422f, 1.095360363f, 384.9217577f},  // PEAKS
{816.885024f, 1479.463946f, 0.001253215609f, 0.9329636822f, 0.06746643971f, 1.573435413f, 419.6006374f}  // PEAKS_GLARE
};

AVFILTER_DEFINE_CLASS(pu21);

static av_cold int pu21_init(AVFilterContext* ctx) {
  PU21Context* pu21 = ctx->priv;
  return 0;
}

static const enum AVPixelFormat pix_fmts[] = {
     AV_PIX_FMT_YUV444P10LE,
     AV_PIX_FMT_NONE
};

static void yuv_rgb(float y, float u, float v, float* r, float* g, float* b) {
  // YUV TO RGB conversion based on BT 2020
  *r = y + 1.4746 * v;
  *g = y - ((0.2627 * 1.4746) / (0.6780)) * v - (0.0593 * 1.8814) / (0.6780) * u;
  *b = y + 1.8814 * u;
}


static void rgb2yuv(float r, float g, float b, float* y, float* u, float* v) {
  const float kr = 0.2627;
  const float kb = 0.0593;
  *y = kr * r + (1 - kr - kb) * g + kb * b;
  *u = 0.5 * (b - *y) / (1 - kb);
  *v = 0.5 * (r - *y) / (1 - kr);
}


static void hlg2lin(float r_in, float g_in, float b_in, float* r_d, float* g_d, float* b_d, const int depth, const int L_black, const int L_peak) {
  // hlg to linear conversion
  float a = 0.17883277;
  float b = 1 - 4 * a;
  float c = 0.5 - a * log(4 * a);
  float gamma = 1.2;

  float r_s, g_s, b_s;

  // convert rgb to 0-1 range
  r_in = r_in / (1 << depth);
  g_in = g_in / (1 << depth);
  b_in = b_in / (1 << depth);

  // Apply the OETF
  r_s = (r_in <= 0.5) ? (r_in * r_in) / 3.0 : (exp((r_in - c) / a) + b) / 12.0;
  g_s = (g_in <= 0.5) ? (g_in * g_in) / 3.0 : (exp((g_in - c) / a) + b) / 12.0;
  b_s = (b_in <= 0.5) ? (b_in * b_in) / 3.0 : (exp((b_in - c) / a) + b) / 12.0;

  float Y_s = 0.2627 * r_s + 0.6780 * g_s + 0.0593 * b_s;

  // Apply OOTF
  *r_d = (pow(Y_s, gamma) * r_s) * (L_peak - L_black) + L_black;
  *r_d = (pow(Y_s, gamma) * r_s) * (L_peak - L_black) + L_black;
  *r_d = (pow(Y_s, gamma) * r_s) * (L_peak - L_black) + L_black;
}

static int filter_frame(AVFilterLink* inlink, AVFrame* input) {
  AVFilterContext* ctx = inlink->dst;
  PU21Context* pu21 = ctx->priv;
  AVFilterLink* outlink = ctx->outputs[0];
  AVFrame* out;

  av_log(inlink->dst, AV_LOG_DEBUG, "Filter input: %s, %ux%u (%"PRId64").\n",
    av_get_pix_fmt_name(input->format),
    input->width, input->height, input->pts);

  if (av_frame_is_writable(input)) {
    out = input;
  }
  else {
    out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
    if (!out) {
      av_frame_free(&input);
      return AVERROR(ENOMEM);
    }
    av_frame_copy_props(out, input);
  }

  const int height = input->height;
  const int width = input->width;

  // image pixel sources in 8. 16, 32 bit formats for Y, U and V
  const uint16_t* src16_y = (const uint16_t*)input->data[0];
  const uint16_t* src16_u = (const uint16_t*)input->data[1];
  const uint16_t* src16_v = (const uint16_t*)input->data[2];

  // image pixel destination in 8. 16, 32 bit formats for Y, U and V
  uint16_t* dst16_y = (uint16_t*)out->data[0];
  uint16_t* dst16_u = (uint16_t*)out->data[1];
  uint16_t* dst16_v = (uint16_t*)out->data[2];
  int depth = pu21->depth;
  int linesize;

  linesize = input->linesize[0] / 2;
  float rgb[3], r_linear, g_linear, b_linear;
  float y_s, u_s, v_s;
  float y_d, u_d, v_d;
  float max_bit_depth = (1 << depth);
  out->color_trc = AVCOL_TRC_LINEAR;
  av_log(inlink->dst, AV_LOG_DEBUG, "Filter input: %s, %ux%u depth:%d(%"PRId64").\n",
    av_get_pix_fmt_name(input->format),
    input->width, input->height, depth, input->pts);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      // Get YUV values and convert to their corresponding rgb values
      y_s = src16_y[y * linesize + x];
      u_s = src16_u[y * linesize + x];
      v_s = src16_v[y * linesize + x];

      yuv_rgb(y_s, u_s, v_s, &rgb[0], &rgb[1], &rgb[2]);

      // EOTF/OOTF conversion
      hlg2lin(rgb[0], rgb[1], rgb[2], &r_linear, &g_linear, &b_linear, depth, pu21->L_min, pu21->L_max);

      // PU21 conversion ( currently for bandling_glare)
      // float r_d = fmax(PU21_MATRICES[1][6] * (pow((PU21_MATRICES[1][0] + PU21_MATRICES[1][1] * pow(r_linear, PU21_MATRICES[1][3])) / (1 + PU21_MATRICES[1][2] * pow(r_linear, PU21_MATRICES[1][3])), PU21_MATRICES[1][4]) - PU21_MATRICES[1][5]), 0);
      // float g_d = fmax(PU21_MATRICES[1][6] * (pow((PU21_MATRICES[1][0] + PU21_MATRICES[0][1] * pow(g_linear, PU21_MATRICES[1][3])) / (1 + PU21_MATRICES[1][2] * pow(g_linear, PU21_MATRICES[1][3])), PU21_MATRICES[1][4]) - PU21_MATRICES[1][5]), 0);
      // float b_d = fmax(PU21_MATRICES[1][6] * (pow((PU21_MATRICES[1][0] + PU21_MATRICES[1][1] * pow(b_linear, PU21_MATRICES[1][3])) / (1 + PU21_MATRICES[1][2] * pow(b_linear, PU21_MATRICES[1][3])), PU21_MATRICES[1][4]) - PU21_MATRICES[1][5]), 0);

      // direct for yuv->rgb->yuv conversion
      // rgb2yuv(rgb[0], rgb[1], rgb[2], &y_d, &u_d, &v_d);

      // yuv->rgb -> ootf/ootf -> yuv consersion 
      rgb2yuv(r_linear, g_linear, b_linear, &y_d, &u_d, &v_d);

      // yuv -> rgb -> ootf/ootf -> pu21-> yuv conversion
      // rgb2yuv(r_d, g_d, b_d, &y_d, &u_d, &v_d);

      dst16_y[y * linesize + x] = y_d;
      dst16_u[y * linesize + x] = u_d;
      dst16_v[y * linesize + x] = v_d;
    }
  }

  if (out != input)
    av_frame_free(&input);
  return ff_filter_frame(outlink, out);
}

static av_cold void uninit(AVFilterContext* ctx) {
  PU21Context* s = ctx->priv;

  av_freep(&s->buffer);
}

static int config_input(AVFilterLink* inlink) {
  const AVPixFmtDescriptor* desc = av_pix_fmt_desc_get(inlink->format);
  PU21Context* s = inlink->dst->priv;

  uninit(inlink->dst);

  s->depth = desc->comp[0].depth;
  // s->planewidth[1] = s->planewidth[2] = AV_CEIL_RSHIFT(inlink->w, desc->log2_chroma_w);
  // s->planewidth[0] = s->planewidth[3] = inlink->w;
  // s->planeheight[1] = s->planeheight[2] = AV_CEIL_RSHIFT(inlink->h, desc->log2_chroma_h);
  // s->planeheight[0] = s->planeheight[3] = inlink->h;

  s->nb_planes = av_pix_fmt_count_planes(inlink->format);



  return 0;
}

static const AVFilterPad pu21_inputs[] = {
    {
        .name = "default",
        .type = AVMEDIA_TYPE_VIDEO,
        .config_props = config_input,
        .filter_frame = filter_frame,
    }
};

static const AVFilterPad pu21_outputs[] = {
    {
        .name = "default",
        .type = AVMEDIA_TYPE_VIDEO,
    }
};

const AVFilter ff_vf_pu21 = {
    .name = "pu21",
    .description = NULL_IF_CONFIG_SMALL("Convert HDR linear values to PU values using PU21 transform"),
    .priv_size = sizeof(PU21Context),
    .init = pu21_init,
    .uninit = uninit,
    FILTER_INPUTS(pu21_inputs),
    FILTER_OUTPUTS(pu21_outputs),
    .priv_class = &pu21_class,
    .flags = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC,
    FILTER_PIXFMTS_ARRAY(pix_fmts)
};
