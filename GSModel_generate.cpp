#include "GSModel.h"

using std::vector;

static inline float safe_div(float a, float b) {
    return (b > 1e-8f) ? (a / b) : 0.0f;
}

KeyframeMotion GSModel::Generate(const Posture& start,
                                 const Posture& goal,
                                 float tempo) const {
    GenerateOptions opt;
    opt.tempo = tempo;
    return Generate(start, goal, opt);
}

KeyframeMotion GSModel::Generate(const Posture& start,
                                 const Posture& goal,
                                 const GenerateOptions& opt_in) const {
    if (!IsCompatible(start) || !IsCompatible(goal)) {
        throw std::runtime_error("GSModel::Generate: Skeleton mismatch in input Posture.");
    }
    if (splats_.empty()) {
        throw std::runtime_error("GSModel::Generate: Empty model.");
    }

    // オプション（デフォルトダンプの継承）
    GenerateOptions opt = opt_in;
#if GSM_ENABLE_DUMP
    if (!opt.dump.enabled) opt.dump = default_dump_;
#endif

    // ロールアウト
    vector<float>   times;
    vector<Posture> poses;
#if GSM_ENABLE_DUMP
    vector<StepLog> logs;
    GenerateInitLog initlog;
#endif

    float t = 0.0f;
    Posture cur = start;

    times.push_back(t);
    poses.push_back(cur);

    const float dt = (opt.dt_seconds > 0.0f ? opt.dt_seconds : (1.0f/30.0f));
    const float goal_th = std::max(1e-4f, opt.goal_tolerance_m);

    // ゴール最近傍スプラット（停止性確認用）
    int goal_sid = FindNearestSplat(goal, nullptr);
    bool goal_stoppable = (goal_sid >= 0) && (splats_[goal_sid].stopability >= opt.stopability_th);

    // 初期診断
    float d_goal0 = FKDistance(cur, goal);
    float d_tmp = 0.0f;
    int start_sid = FindNearestSplat(cur, &d_tmp);
//    int goal_sid  = FindNearestSplat(goal, nullptr);
//    bool goal_stoppable = (goal_sid >= 0) && (splats_[goal_sid].stopability >= opt.stopability_th);
#if GSM_ENABLE_DUMP
    initlog.d_goal0 = d_goal0;
    initlog.start_sid = start_sid;
    initlog.d_start_splat = d_tmp;
    if (start_sid >= 0) {
        initlog.d_start_next = FKDistance(cur, splats_[start_sid].next_pose);
    }
    initlog.goal_sid = goal_sid;
    initlog.goal_stopability = (goal_sid >= 0) ? splats_[goal_sid].stopability : -1.0f;
#endif

    for (int step = 0; step < opt.max_steps; ++step) {
        // 終了条件（距離）
        float d_goal = FKDistance(cur, goal);
        if (d_goal <= goal_th && (goal_stoppable || !opt.extend_to_stable)) {
            break;
        }

        // 近傍スプラット
        float d_s = 0.0f;
        int sid = FindNearestSplat(cur, &d_s);
        if (sid < 0) break;
        const GaussianSplat& S = splats_[sid];

        // 目標姿勢Qの決定
        //   非停止(s小) -> next_pose寄り, 停止可(s大) -> 目標姿勢寄り
        float s = S.stopability;
        Posture target_model = S.has_next ? S.next_pose : S.mean_pose;

        // 速度ノルム
        float v_ref = S.v_norm_ref * opt.tempo;
        float v_min = S.v_norm_min * opt.tempo;
        float v_max = S.v_norm_max * opt.tempo;

        // 2つの候補への距離
        float d_model = FKDistance(cur, target_model);
        float d_goal2 = d_goal; // 既に計算済み

        // 目標混合
        //   w = (1-s) でモデルフォロー、sでゴール吸引
        //   1ステップの補間率 r = clamp( v*dt / distance )
//        Posture p1, p2, next;
        Posture p1( human_.GetSkeleton() ), p2( human_.GetSkeleton() ), next( human_.GetSkeleton() );

        // モデル側への一歩
//        float r_model = GSModel::Clamp( safe_div( GSModel::Clamp(v_ref, v_min, v_max) * dt, std::max(1e-6f, d_model) ), 0.0f, 1.0f );
        float v_used = GSModel::Clamp(v_ref, v_min, v_max);
        v_used = std::max(v_used, opt.v_floor_mps);
        float r_model = GSModel::Clamp( safe_div( v_used * dt, std::max(1e-6f, d_model) ), 0.0f, 1.0f );
        PostureInterpolation(cur, target_model, r_model, p1); // SimpleHumanの補間 :contentReference[oaicite:6]{index=6}

        // ゴール側への一歩（停止可ほど強く寄せる）
        // ゴールへの速度基準は v_ref を使用（簡易）
//        float r_goal  = GSModel::Clamp( s * safe_div( GSModel::Clamp(v_ref, v_min, v_max) * dt, std::max(1e-6f, d_goal2) ), 0.0f, 1.0f );
        float r_goal  = GSModel::Clamp( s * safe_div( v_used * dt, std::max(1e-6f, d_goal2) ), 0.0f, 1.0f );
        PostureInterpolation(cur, goal, r_goal, p2);

        // 混合：停止可ならp2寄り、非停止ならp1寄り
        float alpha = GSModel::Clamp( s, 0.0f, 1.0f );
        PostureInterpolation(p1, p2, alpha, next);

        // 前進
        t += dt;
        cur = next;
        times.push_back(t);
        poses.push_back(cur);

#if GSM_ENABLE_DUMP
        if (opt.dump.enabled) {
            StepLog L;
            L.step = step;
            L.splat_id = sid;
            L.t = t;
            L.dist_goal = d_goal;
            L.stopability = s;
            L.v_ref = v_ref;
            L.v_used = v_used;
            L.dist_model = d_model;
            L.r_model = r_model;
            L.r_goal  = r_goal;
            L.alpha   = alpha;
            logs.push_back(L);
        }
#endif

        // ゴール到達判定（ゴールが非停止なら延長する可能性あり）
        if (d_goal <= goal_th && goal_stoppable) {
            break;
        }
        // もしゴールが非停止なら、停止可になるまで進める
        if (d_goal <= goal_th && !goal_stoppable && !opt.extend_to_stable) {
            break; // 延長しない設定ならここで終了
        }
    }

    // もしゴールが非停止で extend_to_stable=true なら、停止可になるまで数歩追加
    if (opt.extend_to_stable) {
        for (int k = 0; k < 120; ++k) { // 最長 ~4秒延長
            int sid = FindNearestSplat(poses.back(), nullptr);
            if (sid < 0) break;
            if (splats_[sid].stopability >= opt.stopability_th) break; // 停止可になった
            // そのままモデルフォローで少し進める
            const GaussianSplat& S = splats_[sid];
            Posture next = S.has_next ? S.next_pose : S.mean_pose;
            float v_ref = S.v_norm_ref * opt.tempo;
            float d = FKDistance(poses.back(), next);
            float r = GSModel::Clamp( safe_div( v_ref * opt.dt_seconds, std::max(1e-6f, d) ), 0.0f, 1.0f );
            Posture cur = poses.back();
//            Posture out;
            Posture out( human_.GetSkeleton() );
            PostureInterpolation(cur, next, r, out);
            t += opt.dt_seconds;
            times.push_back(t);
            poses.push_back(out);
        }
    }

    // KeyframeMotion を組み立て
    KeyframeMotion kf(human_.GetSkeleton(), (int)times.size()); // KeyframeMotionのInitは内部で配列を確保 :contentReference[oaicite:7]{index=7}
    for (int i = 0; i < (int)times.size(); ++i) {
        kf.key_times[i] = times[i];
        kf.key_poses[i] = poses[i];
    }

#if GSM_ENABLE_DUMP
    if (opt.dump.enabled) {
//        DumpGenerateTrace(opt.dump.out_dir, logs, times, poses);
        DumpGenerateTrace(opt.dump.out_dir, logs, times, poses, &initlog);
    }
#endif

    return kf;
}
