
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
  double par[7];
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
    { "L_min", "Set the minimum luminance value", OFFSET(L_min), AV_OPT_TYPE_DOUBLE, {.dbl = 0.005}, 0.005, 10000, FLAGS },
    { "L_max", "Set the maximum luminance value", OFFSET(L_max), AV_OPT_TYPE_DOUBLE, {.dbl = 10000}, 0.005, 10000, FLAGS },
    { "multiplier", "Set the parameters for the encoding", OFFSET(multiplier), AV_OPT_TYPE_INT, {.i64 = 1}, 1, 10000, FLAGS},
    { NULL }
};

AVFILTER_DEFINE_CLASS(pu21);

static av_cold int pu21_init(AVFilterContext* ctx) {
  PU21Context* pu21 = ctx->priv;

  // These are the default parameters for the banding_glare encoding, would add rest of the parameters using switch case based on pu21->type
  pu21->par[0] = 0.353487901;
  pu21->par[1] = 0.3734658629;
  pu21->par[2] = 8.277049286 * pow(10, -5);
  pu21->par[3] = 0.9062562627;
  pu21->par[4] = 0.09150303166;
  pu21->par[5] = 0.9099517204;
  pu21->par[6] = 596.3148142;
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

static void rgb_yuv(float* r, float* g, float* b, float* y, float* u, float* v) {
  *y = 0.2627 * (*r) + 0.6780 + (*g) + 0.0593 * (*b);
  *u = (*b - *y) / (1.8814);
  *v = (*r - *y) / (1.4746);
}


static int filter_frame(AVFilterLink* inlink, AVFrame* input) {
  AVFilterContext* ctx = inlink->dst;
  PU21Context* pu21 = ctx->priv;
  AVFilterLink* outlink = ctx->outputs[0];
  AVFrame* out;

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

  const int height = pu21->planeheight[0];
  const int width = pu21->planewidth[0];

  // image pixel sources in 8. 16, 32 bit formats for Y, U and V
  const uint8_t* src_y = input->data[0];
  const uint16_t* src16_y = (const uint16_t*)input->data[0];
  const float* src32_y = (const float*)input->data[0];
  const uint8_t* src_u = input->data[1];
  const uint16_t* src16_u = (const uint16_t*)input->data[1];
  const float* src32_u = (const float*)input->data[1];
  const uint8_t* src_v = input->data[2];
  const uint16_t* src16_v = (const uint16_t*)input->data[2];
  const float* src32_v = (const float*)input->data[2];

  // image pixel destination in 8. 16, 32 bit formats for Y, U and V
  uint8_t* dst_y = out->data[0];
  uint16_t* dst16_y = (uint16_t*)out->data[0];
  float* dst32_y = (float*)out->data[0];
  uint8_t* dst_u = out->data[1];
  uint16_t* dst16_u = (uint16_t*)out->data[1];
  float* dst32_u = (float*)out->data[1];
  uint8_t* dst_v = out->data[2];
  uint16_t* dst16_v = (uint16_t*)out->data[2];
  float* dst32_v = (float*)out->data[2];

  if (input->color_trc == AVCOL_TRC_SMPTE2084 || input->color_trc == AVCOL_TRC_ARIB_STD_B67) {
  }
  else if (input->color_trc == AVCOL_TRC_UNSPECIFIED) {
    // assuming linear input
    av_log(ctx, AV_LOG_WARNING, "Assuming input frame to be linearized, colors may be off\n");
    out->color_trc = AVCOL_TRC_LINEAR;
  }
  else {

    av_log(ctx, AV_LOG_ERROR, "Input color_trc not supported, convert to linear before applying the filter\n");
  }

  int depth = pu21->depth;
  int linesize;

  if (depth <= 8) {
    linesize = input->linesize[0];
  }
  else if (depth <= 16) {
    linesize = input->linesize[0] / 2;
  }
  else {
    linesize = input->linesize[0] / 4;
  }

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      float r, g, b;
      float y_s, u_s, v_s;
      // Get YUV values and convert to their corresponding rgb values
      if (depth <= 8) {
        y_s = (float)src_y[y + linesize + x];
        u_s = (float)src_u[y + linesize + x];
        v_s = (float)src_v[y + linesize + x];
      }
      else if (depth <= 16) {
        y_s = (float)src16_y[y + linesize + x];
        u_s = (float)src16_u[y + linesize + x];
        v_s = (float)src16_v[y + linesize + x];
      }
      else {

        y_s = (float)src32_y[y + linesize + x];
        u_s = (float)src32_u[y + linesize + x];
        v_s = (float)src32_v[y + linesize + x];
      }

      yuv_rgb(y_s, u_s, v_s, &r, &g, &b);

      // Linearize the input based on pix_trc and apply OETF/OOTF on the rgb values.
      if (input->color_trc == AVCOL_TRC_ARIB_STD_B67) {
        // hlg_lin
      }
      else if (input->color_trc == AVCOL_TRC_SMPTE2084) {
        // pq_li
        printf("Hello");
      }

      // apply the pu21 encoding
      float r_s = r * (pu21->multiplier);
      float r_d = fmax(pu21->par[6] * (pow((pu21->par[0] + pu21->par[1] * pow(r_s, pu21->par[3])) / (1 + pu21->par[2] * pow(r_s, pu21->par[3])), pu21->par[4]) - pu21->par[5]), 0);
      float g_s = g * (pu21->multiplier);
      float g_d = fmax(pu21->par[6] * (pow((pu21->par[0] + pu21->par[1] * pow(g_s, pu21->par[3])) / (1 + pu21->par[2] * pow(g_s, pu21->par[3])), pu21->par[4]) - pu21->par[5]), 0);
      float b_s = b * (pu21->multiplier);
      float b_d = fmax(pu21->par[6] * (pow((pu21->par[0] + pu21->par[1] * pow(b_s, pu21->par[3])) / (1 + pu21->par[2] * pow(b_s, pu21->par[3])), pu21->par[4]) - pu21->par[5]), 0);

      // convert the rgb values to respective Y, U and V values and pass them in their destination buffers.
      float y_d, u_d, v_d;
      rgb_yuv(&r_d, &g_d, &b_d, &y_d, &u_d, &v_d);
      if (depth <= 8) {
        dst_y[y * linesize + x] = y_d;
        dst_u[y * linesize + x] = u_d;
        dst_v[y * linesize + x] = v_d;
      }
      else if (depth <= 16) {
        dst16_y[y * linesize + x] = y_d;
        dst16_u[y * linesize + x] = u_d;
        dst16_v[y * linesize + x] = v_d;
      }
      else {
        dst32_y[y * linesize + x] = y_d;
        dst32_u[y * linesize + x] = u_d;
        dst32_v[y * linesize + x] = v_d;
      }
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
