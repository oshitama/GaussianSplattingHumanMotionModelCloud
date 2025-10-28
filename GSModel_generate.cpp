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

    const float eps_progress = 1e-6f;
    int stagnation_count = 0;
    int force_goal_steps = 0;
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

        float v_used = GSModel::Clamp(v_ref, v_min, v_max);
        v_used = std::max(v_used, opt.v_floor_mps);

        auto format_float = [](float value) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(4) << value;
            return oss.str();
        };

        bool force_goal_mode = (force_goal_steps > 0);
        if (force_goal_mode) {
            --force_goal_steps;
        }

        float dt_try = dt;
        int dt_backoff = 0;
        bool advanced = false;
        Posture best_pose(human_.GetSkeleton());
        float best_delta = -std::numeric_limits<float>::infinity();
        float best_dist_goal = d_goal;
        float best_step_norm = 0.0f;
        float best_alpha = 0.0f;
        std::string best_mode;
        float best_dt = dt;
        float best_r_model = 0.0f;
        float best_r_goal = 0.0f;
        std::vector<std::string> event_tags;

        while (dt_backoff <= 2 && !advanced) {
            float dt_local = dt_try;
            float r_model_base = GSModel::Clamp( safe_div( v_used * dt_local, std::max(1e-6f, d_model) ), 0.0f, 1.0f );
            float r_goal_base  = GSModel::Clamp( safe_div( v_used * dt_local, std::max(1e-6f, d_goal) ), 0.0f, 1.0f );

            const float alpha_candidates[] = {0.0f, 0.25f, 0.5f, 0.75f, 1.0f};

            auto evaluate_alpha = [&](float alpha, const std::string& mode) {
                Posture p_model(human_.GetSkeleton());
                Posture p_goal(human_.GetSkeleton());
                Posture candidate(human_.GetSkeleton());

                float r_model = (1.0f - alpha) * r_model_base;
                float r_goal  = alpha * r_goal_base;
                PostureInterpolation(cur, target_model, r_model, p_model);
                PostureInterpolation(cur, goal, r_goal, p_goal);
                PostureInterpolation(p_model, p_goal, alpha, candidate);

                float dist_goal_next = FKDistance(candidate, goal);
                float delta_goal = d_goal - dist_goal_next;
                float step_norm = FKDistance(cur, candidate);

                if (delta_goal > best_delta + eps_progress) {
                    best_delta = delta_goal;
                    best_pose = candidate;
                    best_dist_goal = dist_goal_next;
                    best_step_norm = step_norm;
                    best_alpha = alpha;
                    best_mode = mode;
                    best_dt = dt_local;
                    best_r_model = r_model;
                    best_r_goal = r_goal;
                }
            };

            if (force_goal_mode) {
                evaluate_alpha(1.0f, "force_goal");
            } else {
                for (float alpha_candidate : alpha_candidates) {
                    evaluate_alpha(alpha_candidate, "grid");
                }
            }

            if (best_delta > eps_progress) {
                advanced = true;
                break;
            }

            // フォールバック: α=1 で再評価
            evaluate_alpha(1.0f, force_goal_mode ? "force_goal" : "fallback_goal");
            if (best_delta > eps_progress) {
                advanced = true;
                if (!force_goal_mode && best_mode != "force_goal") {
                    best_mode = "fallback_goal";
                }
                break;
            }

            dt_try *= 0.5f;
            ++dt_backoff;
        }

        if (!advanced && best_delta <= eps_progress) {
            break;
        }

        bool progressed = (best_delta > eps_progress);
        int next_stagnation = progressed ? 0 : (stagnation_count + 1);
        bool trigger_force = (!progressed && next_stagnation >= 3);

        if (dt_backoff > 0) {
            event_tags.push_back("dt=" + format_float(best_dt));
        }
        if (force_goal_mode) {
            event_tags.push_back("force_goal");
        } else if (best_mode == "fallback_goal") {
            event_tags.push_back("fallback_goal");
        }
        if (trigger_force) {
            event_tags.push_back("trigger_force_goal");
        }

        // 前進
        t += best_dt;
        cur = best_pose;
        times.push_back(t);
        poses.push_back(cur);

#if GSM_ENABLE_DUMP
        if (opt.dump.enabled) {
            StepLog L;
            L.step = step;
            L.splat_id = sid;
            L.t_sec = t;
            L.dist_goal = best_dist_goal;
            L.delta_goal = best_delta;
            L.alpha = best_alpha;
            L.alpha_mode = best_mode;
            L.step_norm = best_step_norm;
            L.dt = best_dt;
            L.v_ref = v_ref;
            L.v_min = v_min;
            L.v_max = v_max;
            L.used_speed = v_used;
            L.v_floor = opt.v_floor_mps;
            L.stopability = s;
            L.r_model = best_r_model;
            L.r_goal = best_r_goal;
            if (!event_tags.empty()) {
                std::ostringstream oss;
                for (size_t i = 0; i < event_tags.size(); ++i) {
                    if (i) oss << ';';
                    oss << event_tags[i];
                }
                L.events = oss.str();
            }
            logs.push_back(L);
        }
#endif

        if (trigger_force) {
            force_goal_steps = 3;
        }
        stagnation_count = trigger_force ? 0 : next_stagnation;

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
