#include "GSModel.h"

using std::vector;

GSModel::GSModel(const HumanBody& human) : human_(human) {
#if GSM_ENABLE_DUMP
    default_dump_.enabled = false;
    default_dump_.out_dir = "gs_dump";
    default_dump_.detail  = 1;
#endif
}

bool GSModel::IsCompatible(const Posture& p) const {
    return (p.body == human_.GetSkeleton()); // HumanBodyのSkeletonと一致かを確認
}

void GSModel::FKJointPositions(const Posture& p, std::vector<Point3f>& joints) const {
    joints.clear();
    std::vector<Matrix4f> seg_frames;
    std::vector<Point3f>  joi_pos;
    ForwardKinematics(p, seg_frames, joi_pos); // SimpleHuman のFK（関節位置も得る版） :contentReference[oaicite:5]{index=5}
    joints = std::move(joi_pos);
}

float GSModel::FKDistance(const Posture& a, const Posture& b) const {
    // スケルトン一貫性（安全策）
    if (a.body != b.body) return std::numeric_limits<float>::infinity();
    std::vector<Point3f> ja, jb;
    FKJointPositions(a, ja);
    FKJointPositions(b, jb);
    if (ja.empty() || jb.empty() || ja.size() != jb.size())
        return std::numeric_limits<float>::infinity();

    // root平行移動の影響を除去するため、各関節座標からroot_posを減算して比較
    Vector3f ra(a.root_pos.x, a.root_pos.y, a.root_pos.z);
    Vector3f rb(b.root_pos.x, b.root_pos.y, b.root_pos.z);
    double acc = 0.0;
    for (size_t i = 0; i < ja.size(); ++i) {
        Vector3f va(ja[i].x - ra.x, ja[i].y - ra.y, ja[i].z - ra.z);
        Vector3f vb(jb[i].x - rb.x, jb[i].y - rb.y, jb[i].z - rb.z);
        double dx = double(va.x) - double(vb.x);
        double dy = double(va.y) - double(vb.y);
        double dz = double(va.z) - double(vb.z);
        acc += dx*dx + dy*dy + dz*dz;
    }
    return float(std::sqrt(acc / double(ja.size()))); // RMSE[m]
}

int GSModel::FindNearestSplat(const Posture& p, float* out_dist) const {
    int best = -1;
    float best_d = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i < splats_.size(); ++i) {
        float d = FKDistance(p, splats_[i].mean_pose);
        if (d < best_d) {
            best_d = d;
            best = int(i);
        }
    }
    if (out_dist) *out_dist = best_d;
    return best;
}

// 一括Fitユーティリティ
GSModel GSModel::Fit(const HumanBody& human,
                     const std::vector<const Motion*>& motions,
                     const TrainOptions& opt) {
    GSModelBuilder builder(human, opt);
    for (auto m : motions) {
        if (!m) continue;
        builder.AddMotion(*m);
    }
    GSModel model = builder.Build();
#if GSM_ENABLE_DUMP
    if (opt.dump.enabled) {
        model.DumpModel(opt.dump.out_dir);
    }
#endif
    return model;
}
