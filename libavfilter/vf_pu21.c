/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "libavfilter/avfilter.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/mem.h"
#include "libavutil/imgutils.h"
#include "libavfilter/formats.h"
#include "libavutil/internal.h"
#include "video.h"

enum pu21_mode {
  BANDING,
  BANDING_GLARE,
  PEAKS,
  PEAKS_GLARE
};

typedef struct PU21Context {
  const AVClass* class;
  double L_min, L_max;
  int multiplier;
  int mode;

  int depth;
  int nb_planes;
} PU21Context;

const float pu21_params[4][7] = {
  // Reference: "PU21: A novel perceptually uniform encoding for adapting existing quality metrics for HDR"
  // Rafał K. Mantiuk and M. Azimi, Picture Coding Symposium 2021
  // https://github.com/gfxdisp/pu21
  {1.070275272f, 0.4088273932f, 0.153224308f, 0.2520326168f, 1.063512885f, 1.14115047f, 521.4527484f},  // BANDING
  {0.353487901f, 0.3734658629f, 8.277049286e-05f, 0.9062562627f, 0.09150303166f, 0.9099517204f, 596.3148142f},  // BANDING_GLARE
  {1.043882782f, 0.6459495343f, 0.3194584211f, 0.374025247f, 1.114783422f, 1.095360363f, 384.9217577f},  // PEAKS
  {816.885024f, 1479.463946f, 0.001253215609f, 0.9329636822f, 0.06746643971f, 1.573435413f, 419.6006374f}  // PEAKS_GLARE
};


#define OFFSET(x) offsetof(PU21Context, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM
static const AVOption pu21_options[] = {
    { "L_min", "Set the minimum luminance value", OFFSET(L_min), AV_OPT_TYPE_DOUBLE, {.dbl = 0.005}, 0.005, 10000, FLAGS },
    { "L_max", "Set the maximum luminance value", OFFSET(L_max), AV_OPT_TYPE_DOUBLE, {.dbl = 10000}, 0.005, 10000, FLAGS },
    { "multiplier", "Set the parameters for the encoding", OFFSET(multiplier), AV_OPT_TYPE_INT, {.i64 = 1}, 1, 10000, FLAGS},
    { "mode", "Set the mode of the PU21 encoding", OFFSET(mode), AV_OPT_TYPE_INT, {.i64 = BANDING_GLARE}, BANDING, PEAKS_GLARE, FLAGS},
    { NULL }
};

AVFILTER_DEFINE_CLASS(pu21);

static av_cold int pu21_init(AVFilterContext* ctx) {
  PU21Context* pu21 = ctx->priv;

  pu21->L_max = 300;
  pu21->L_min = 0.005;
  pu21->multiplier = 1;
  pu21->mode = BANDING_GLARE;
  return 0;
}

static float apply_pu21(float pixel_val, PU21Context* pu21) {
  float Y, V;
  Y = pixel_val * (pu21->multiplier);
  V = fmax(pu21_params[pu21->mode][6] * (pow((pu21_params[pu21->mode][0] + pu21_params[pu21->mode][1] * pow(Y, pu21_params[pu21->mode][3])) / (1 + pu21_params[pu21->mode][2] * pow(Y, pu21_params[pu21->mode][3])), pu21_params[pu21->mode][4]) - pu21_params[pu21->mode][5]), 0);
  return V;
}

static void pu21_encode8(AVFilterContext* ctx, AVFrame* in, AVFrame* out) {
  PU21Context* pu21 = ctx->priv;
  const int height = in->height;
  const int width = in->width;

  int plane;
  for (plane = 0; plane < pu21->nb_planes; plane++) {
    const uint8_t* src8 = in->data[plane];
    uint8_t* dst8 = out->data[plane];

    int  linesize = in->linesize[plane];

    uint8_t pixel_val;
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        pixel_val = src8[y * linesize + x];
        dst8[y * linesize + x] = (uint8_t)apply_pu21(pixel_val, pu21);
      }
    }
  }
}

static void pu21_encode16(AVFilterContext* ctx, AVFrame* in, AVFrame* out) {
  PU21Context* pu21 = ctx->priv;
  const int height = in->height;
  const int width = in->width;

  int plane;
  for (plane = 0; plane < pu21->nb_planes; plane++) {
    const uint16_t* src16 = (uint16_t*)in->data[plane];
    uint16_t* dst16 = (uint16_t*)out->data[plane];

    int  linesize = in->linesize[plane] / 2;

    uint16_t pixel_val;
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        pixel_val = src16[y * linesize + x];
        dst16[y * linesize + x] = (uint16_t)apply_pu21(pixel_val, pu21);
      }
    }
  }
}

static void pu21_encode32(AVFilterContext* ctx, AVFrame* in, AVFrame* out) {
  PU21Context* pu21 = ctx->priv;
  const int height = in->height;
  const int width = in->width;

  int plane;
  for (plane = 0; plane < pu21->nb_planes; plane++) {
    const float* src32 = (float*)in->data[plane];
    float* dst32 = (float*)out->data[plane];

    int  linesize = in->linesize[plane] / 4;

    float pixel_val;
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        pixel_val = src32[y * linesize + x];
        dst32[y * linesize + x] = (float)apply_pu21(pixel_val, pu21);
      }
    }
  }
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

  if (pu21->depth <= 8) {
    pu21_encode8(ctx, input, out);
  }
  else if (pu21->depth <= 16) {
    pu21_encode16(ctx, input, out);
  }
  else {
    pu21_encode32(ctx, input, out);
  }

  if (out != input)
    av_frame_free(&input);
  return ff_filter_frame(outlink, out);
}

static av_cold void uninit(AVFilterContext* ctx) {}

static int config_input(AVFilterLink* inlink) {
  const AVPixFmtDescriptor* desc = av_pix_fmt_desc_get(inlink->format);
  PU21Context* s = inlink->dst->priv;

  uninit(inlink->dst);

  s->depth = desc->comp[0].depth;
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
