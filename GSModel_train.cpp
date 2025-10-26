#include "GSModel.h"

using std::vector;
using std::string;

GSModelBuilder::GSModelBuilder(const HumanBody& human, const TrainOptions& opt)
    : human_(human), opt_(opt) {}

void GSModelBuilder::AddMotion(const Motion& m) {
    // スケルトン整合性チェック（HumanBodyのSkeletonと同一）
    if (m.body != human_.GetSkeleton()) {
        throw std::runtime_error("GSModelBuilder::AddMotion: Skeleton mismatch.");
    }
    motions_.push_back(&m);
}

void GSModelBuilder::AppendMotionSplats(const Motion& m, std::vector<GaussianSplat>& out) const {
    const int N = m.num_frames;
    if (N <= 1) return;

    // 代表姿勢の距離計算用に一時モデルを用意（FK距離を使うため）
    GSModel temp(human_);

    for (int i = 0; i < N - 1; i += std::max(1, opt_.sample_stride)) {
        const Posture& cur = *m.GetFrame(i);
        const Posture& nxt = *m.GetFrame(i + 1);

        GaussianSplat g;
        g.id = int(out.size());
        g.mean_pose = cur;
        g.next_pose = nxt;
        g.has_next  = true;
        g.occ_sigma_m = opt_.occ_sigma_m;
        g.source_motion = m.name;
        g.source_frame  = i;
        g.source_interval = m.interval;

        // 速度ノルム（FK距離 / s）
        float dist = temp.FKDistance(cur, nxt);
        float v = (m.interval > 0.0f) ? dist / m.interval : dist;
        g.v_norm_ref = std::max(0.001f, v);
        g.v_norm_min = 0.5f * g.v_norm_ref;
        g.v_norm_max = 2.0f * g.v_norm_ref;

        // 停止可能性：しきい値以下は高め、以上は低め（連続化）
        // s = clamp(1 - v / th, 0, 1)
        float s = 1.0f - (g.v_norm_ref / std::max(1e-4f, opt_.stop_v_threshold));
        g.stopability = GSModel::Clamp(s, 0.0f, 1.0f);

        out.push_back(std::move(g));
    }

    // 最終フレームは next が無いのでオプション：必要なら追加（ここでは追加しない）
}

void GSModelBuilder::MergeNearby(std::vector<GaussianSplat>& splats) const {
    if (!opt_.enable_merge || splats.empty()) return;

    GSModel temp(human_);
    vector<bool> removed(splats.size(), false);

    for (size_t i = 0; i < splats.size(); ++i) {
        if (removed[i]) continue;
        for (size_t j = i + 1; j < splats.size(); ++j) {
            if (removed[j]) continue;
            float d = temp.FKDistance(splats[i].mean_pose, splats[j].mean_pose);
            if (d <= opt_.merge_radius_m) {
                // 近傍：iへ吸収（meanは簡易に「より停止可能な方」を優先）
                if (splats[j].stopability > splats[i].stopability) {
                    splats[i].mean_pose = splats[j].mean_pose;
                    splats[i].next_pose = splats[j].next_pose;
                }
                // 速度レンジは平均的に更新
                splats[i].v_norm_ref = 0.5f * (splats[i].v_norm_ref + splats[j].v_norm_ref);
                splats[i].v_norm_min = std::min(splats[i].v_norm_min, splats[j].v_norm_min);
                splats[i].v_norm_max = std::max(splats[i].v_norm_max, splats[j].v_norm_max);
                splats[i].stopability = 0.5f * (splats[i].stopability + splats[j].stopability);
                removed[j] = true;
            }
        }
    }
    // 圧縮
    vector<GaussianSplat> compact;
    compact.reserve(splats.size());
    for (size_t i = 0; i < splats.size(); ++i) {
        if (!removed[i]) {
            GaussianSplat g = splats[i];
            g.id = int(compact.size());
            compact.push_back(std::move(g));
        }
    }
    splats.swap(compact);
}

GSModel GSModelBuilder::Build() const {
    GSModel model(human_);
    vector<GaussianSplat> buf;
    for (auto m : motions_) {
        AppendMotionSplats(*m, buf);
    }
    MergeNearby(buf);
    model.splats_ = std::move(buf);

#if GSM_ENABLE_DUMP
    if (opt_.dump.enabled) {
        model.DumpModel(opt_.dump.out_dir);
    }
#endif

    if (model.splats_.empty()) {
        throw std::runtime_error("GSModelBuilder::Build: No splats created.");
    }
    return model;
}
