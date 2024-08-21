
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

static void hlg2lin(float r_in, float g_in, float b_in, float* r_d, float* g_d, float* b_d) {
  // hlg to linear conversion
  float a = 0.17883277;
  float b = 1 - 4 * a;
  float c = 0.5 - a * log(4 * a);
  float gamma = 1.2;

  float r_s, g_s, b_s;

  // Apply the OETF

  r_s = (r_in <= 0.5) ? pow(r_in, 2) / 3.0 : (exp((r_in - c) / a) + b) / 12;
  g_s = (g_in <= 0.5) ? pow(g_in, 2) / 3.0 : (exp((g_in - c) / a) + b) / 12;
  b_s = (b_in <= 0.5) ? pow(b_in, 2) / 3.0 : (exp((b_in - c) / a) + b) / 12;


  float Y_s = 0.2627 * r_s + 0.6780 * g_s + 0.0593 * b_s;

  // Apply OOTF
  *r_d = pow(Y_s, gamma) * r_s;
  *g_d = pow(Y_s, gamma) * g_s;
  *b_d = pow(Y_s, gamma) * b_s;
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
    // throw error
    av_log(ctx, AV_LOG_ERROR, "Input color_trc not supported, convert to linear before applying the filter\n");
    return AVERROR(EINVAL); // Return an appropriate error code
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

  float r, g, b, r_linear, g_linear, b_linear;
  float y_s, u_s, v_s;
  float max_bit_depth = (1 << depth) - 1;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      // Get YUV values and convert to their corresponding rgb values
      if (depth <= 8) {
        y_s = (float)src_y[y + linesize + x];
        u_s = (float)src_u[y + linesize + x];
        v_s = (float)src_v[y + linesize + x];
      }
      else if (depth <= 16) {
        y_s = (float)((uint16_t*)src16_y)[y + linesize + x];
        u_s = (float)((uint16_t*)src16_u)[y + linesize + x];
        v_s = (float)((uint16_t*)src16_v)[y + linesize + x];
      }
      else {
        y_s = (float)((float*)src32_y)[y + linesize + x];
        u_s = (float)((float*)src32_u)[y + linesize + x];
        v_s = (float)((float*)src32_v)[y + linesize + x];
      }

      // YUV to RGB conversions
      yuv_rgb(y_s, u_s, v_s, &r, &g, &b);

      // convert rgb to 0-1 scale
      r = r / max_bit_depth;
      g = g / max_bit_depth;
      b = b / max_bit_depth;

      // Linearize the input based on pix_trc and apply OETF/OOTF on the rgb values.
      if (input->color_trc == AVCOL_TRC_ARIB_STD_B67) {
        // hlg_lin
        hlg2lin(r, g, b, &r_linear, &g_linear, &b_linear);
        // r_linear = r_linear * (pu21->L_max - pu21->L_min) + pu21->L_min;
        // g_linear = g_linear * (pu21->L_max - pu21->L_min) + pu21->L_min;
        // b_linear = b_linear * (pu21->L_max - pu21->L_min) + pu21->L_min;
      }
      else if (input->color_trc == AVCOL_TRC_SMPTE2084) {
        // pq_li
        // TODO
        r_linear = r;
        g_linear = g;
        b_linear = b;
      }

      // apply the pu21 encoding
      float r_s = r_linear * (pu21->multiplier);
      float r_pu21 = fmax(pu21->par[6] * (pow((pu21->par[0] + pu21->par[1] * pow(r_s, pu21->par[3])) / (1 + pu21->par[2] * pow(r_s, pu21->par[3])), pu21->par[4]) - pu21->par[5]), 0);
      float g_s = g_linear * (pu21->multiplier);
      float g_pu21 = fmax(pu21->par[6] * (pow((pu21->par[0] + pu21->par[1] * pow(g_s, pu21->par[3])) / (1 + pu21->par[2] * pow(g_s, pu21->par[3])), pu21->par[4]) - pu21->par[5]), 0);
      float b_s = b_linear * (pu21->multiplier);
      float b_pu21 = fmax(pu21->par[6] * (pow((pu21->par[0] + pu21->par[1] * pow(b_s, pu21->par[3])) / (1 + pu21->par[2] * pow(b_s, pu21->par[3])), pu21->par[4]) - pu21->par[5]), 0);

      // convert the rgb values to respective Y, U and V values and pass them in their destination buffers.
      float y_d, u_d, v_d;
      rgb_yuv(&r_pu21, &g_pu21, &b_pu21, &y_d, &u_d, &v_d);
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
