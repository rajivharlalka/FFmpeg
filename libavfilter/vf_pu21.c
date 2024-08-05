
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
    {"multiplier", "Set the parameters for the encoding", OFFSET(multiplier), AV_OPT_TYPE_INT, {.i64 = 1}, 1, 10000, FLAGS},
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


  double input_avg_val[3];
  double input_max_val[3];
  double output_avg_val[3];
  double output_max_val[3];

  int plane;
  for (plane = 0; plane < pu21->nb_planes; plane++) {
    const AVPixFmtDescriptor* desc = av_pix_fmt_desc_get(inlink->format);
    const int height = pu21->planeheight[plane];
    const int width = pu21->planewidth[plane];
    const uint8_t* src = input->data[plane];
    const uint16_t* src16 = (const uint16_t*)input->data[plane];
    const float* src32 = (const float*)input->data[plane];
    uint8_t* dst = out->data[plane];
    uint16_t* dst16 = (uint16_t*)out->data[plane];
    float* dst32 = (float*)out->data[plane];

    int depth = pu21->depth;
    int pixel_format = input->format;
    int linesize;

    if (depth <= 8) {
      linesize = input->linesize[plane];
    }
    else if (depth <= 16) {
      linesize = input->linesize[plane] / 2;
    }
    else {
      linesize = input->linesize[plane] / 4;
    }

    av_log(inlink->dst, AV_LOG_DEBUG, "Filter input: %s, %ux%u (%"PRId64").\n",
      av_get_pix_fmt_name(input->format),
      input->width, input->height, input->pts);

    double par[7];
    memcpy(par, pu21->par, sizeof(par));
    double pixel_val;

    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {

        if (depth <= 8) {
          pixel_val = ((double)src[y * linesize + x]);
        }
        else if (depth <= 16) {
          pixel_val = ((double)src16[y * linesize + x]);
        }
        else {
          pixel_val = src32[y * linesize + x];
        }
        float Y = pixel_val * (pu21->multiplier);

        input_avg_val[plane] += Y;
        input_max_val[plane] = FFMAX(input_max_val[plane], Y);

        // av_log(inlink->dst, AV_LOG_DEBUG, "Frame Value: %d %d %d %f\n", x, y, plane, pixel_val);

        float V = fmax(par[6] * (pow((par[0] + par[1] * pow(Y, par[3])) / (1 + par[2] * pow(Y, par[3])), par[4]) - par[5]), 0);

        output_avg_val[plane] += V;
        output_max_val[plane] = FFMAX(output_max_val[plane], V);

        if (depth <= 8) {
          dst[y * linesize + x] = V;
        }
        else if (depth <= 16) {
          dst16[y * linesize + x] = V;
        }
        else {
          dst32[y * linesize + x] = (V);
        }
      }
    }
  }

  av_log(inlink->dst, AV_LOG_DEBUG, "INPUT: avg_val Value: %f %f %f Max Value: %f %f %f\n", input_avg_val[0] / (pu21->planeheight[0] * pu21->planewidth[0]), input_avg_val[1] / (pu21->planeheight[1] * pu21->planewidth[1]), input_avg_val[2] / (pu21->planeheight[2] * pu21->planewidth[2]), input_max_val[0], input_max_val[1], input_max_val[2]);

  av_log(inlink->dst, AV_LOG_DEBUG, "OUTPUT: avg_val Value: %f %f %f Max Value: %f %f %f\n", output_avg_val[0] / (pu21->planeheight[0] * pu21->planewidth[0]), output_avg_val[1] / (pu21->planeheight[1] * pu21->planewidth[1]), output_avg_val[2] / (pu21->planeheight[2] * pu21->planewidth[2]), output_max_val[0], output_max_val[1], output_max_val[2]);
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
  s->planewidth[1] = s->planewidth[2] = AV_CEIL_RSHIFT(inlink->w, desc->log2_chroma_w);
  s->planewidth[0] = s->planewidth[3] = inlink->w;
  s->planeheight[1] = s->planeheight[2] = AV_CEIL_RSHIFT(inlink->h, desc->log2_chroma_h);
  s->planeheight[0] = s->planeheight[3] = inlink->h;

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
};
