
#include <libavfilter/avfilter.h>
#include <libavutil/opt.h>
#include <libavutil/pixdesc.h>
#include <libavutil/imgutils.h>
#include <libavfilter/formats.h>
#include <libavfilter/internal.h>

typedef struct PU21Context {
  const AVClass* class;
  double L_min, L_max;
  double par[7];
  char* type;
} PU21Context;

#define OFFSET(x) offsetof(PU21Context, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM
static const AVOption pu21_options[] = {
    { "type", "Set the type of encoding", OFFSET(type), AV_OPT_TYPE_STRING, {.str = "banding_glare"}, 0, 0, FLAGS }, // options can be banding, banding_glare, peaks, peaks_glare
    { NULL }
};

AVFILTER_DEFINE_CLASS(pu21);

static av_cold int pu21_init(AVFilterContext* ctx) {
  PU21Context* pu21 = ctx->priv;
  pu21->L_min = 0.005;
  pu21->L_max = 10000;

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

static int filter_frame(AVFilterLink* inlink, AVFrame* frame) {
  PU21Context* pu21 = inlink->dst->priv;
  int width = frame->width;
  int height = frame->height;
  int linesize = frame->linesize[0];
  uint8_t* data = frame->data[0];

  double par[7];
  memcpy(par, pu21->par, sizeof(par));

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      double Y = ((double)data[y * linesize + x]) / 255.0 * 10000.0;
      double V = fmax(par[6] * (pow((par[0] + par[1] * pow(Y, par[3])) / (1 + par[2] * pow(Y, par[3])), par[4]) - par[5]), 0);
      data[y * linesize + x] = (uint8_t)(fmin(fmax(V, 0), 255));
    }
  }

  return ff_filter_frame(inlink->dst->outputs[0], frame);
}

static const AVFilterPad pu21_inputs[] = {
    {
        .name = "default",
        .type = AVMEDIA_TYPE_VIDEO,
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
    FILTER_INPUTS(pu21_inputs),
    FILTER_OUTPUTS(pu21_outputs),
    .priv_class = &pu21_class,
    .flags = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC,
};
