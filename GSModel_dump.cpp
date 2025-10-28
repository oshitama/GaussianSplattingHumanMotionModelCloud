#include "GSModel.h"

#if GSM_ENABLE_DUMP
#include <sys/stat.h>
#include <sys/types.h>

#if defined(_WIN32)
#include <direct.h>
#endif

static inline void ensure_dir(const std::string& d) {
#if defined(_WIN32)
    _mkdir(d.c_str());
#else
    mkdir(d.c_str(), 0777);
#endif
}

void GSModel::DumpModel(const std::string& dir) const {
    ensure_dir(dir);
    // 概要
    {
        std::ofstream ofs(dir + "/model_summary.json");
        ofs << "{\n";
        ofs << "  \"num_splats\": " << splats_.size() << ",\n";
        ofs << "  \"skeleton_joints\": " << (human_.GetSkeleton() ? human_.GetSkeleton()->num_joints : -1) << "\n";
        ofs << "}\n";
    }
    // スプラット一覧
    {
        std::ofstream ofs(dir + "/splats.csv");
        ofs << "id,source,frame,occ_sigma_m,stopability,v_ref,v_min,v_max,has_next\n";
        for (const auto& s : splats_) {
            ofs << s.id << ","
                << "\"" << s.source_motion << "\"," << s.source_frame << ","
                << s.occ_sigma_m << ","
                << s.stopability << ","
                << s.v_norm_ref << ","
                << s.v_norm_min << ","
                << s.v_norm_max << ","
                << (s.has_next ? 1 : 0) << "\n";
        }
    }
}

void GSModel::DumpGenerateTrace(const std::string& dir,
                                const std::vector<StepLog>& logs,
                                const std::vector<float>& times,
                                const std::vector<Posture>& poses,
                                const GenerateInitLog* init) const 
{
    ensure_dir(dir);
    {
        auto escape_csv = [](const std::string& v) {
            bool need_wrap = false;
            std::string out;
            out.reserve(v.size());
            for (char ch : v) {
                if (ch == '"') {
                    out.push_back('"');
                    out.push_back('"');
                    need_wrap = true;
                } else {
                    if (ch == ',' || ch == '\n') {
                        need_wrap = true;
                    }
                    out.push_back(ch);
                }
            }
            if (need_wrap) {
                return std::string("\"") + out + "\"";
            }
            return out;
        };

        std::ofstream ofs(dir + "/gen_trace.csv");
        ofs << "step,t_sec,dist_goal,delta_goal,alpha,alpha_mode,step_norm,dt,splat_id,v_ref,v_min,v_max,used_speed,v_floor,events" << '\n';
        for (const auto& L : logs) {
            ofs << L.step << ','
                << L.t_sec << ','
                << L.dist_goal << ','
                << L.delta_goal << ','
                << L.alpha << ','
                << escape_csv(L.alpha_mode) << ','
                << L.step_norm << ','
                << L.dt << ','
                << L.splat_id << ','
                << L.v_ref << ','
                << L.v_min << ','
                << L.v_max << ','
                << L.used_speed << ','
                << L.v_floor << ','
                << escape_csv(L.events) << '\n';
        }
    }
    {
        // キーフレーム座標（FK距離の検証に役立つ簡易ダンプ）
        std::ofstream ofs(dir + "/keyframes.jsonl");
        for (size_t i = 0; i < times.size(); ++i) {
            std::vector<Point3f> joints;
            FKJointPositions(poses[i], joints);
            ofs << "{ \"i\": " << i << ", \"t\": " << times[i] << ", \"joints\": [";
            for (size_t j = 0; j < joints.size(); ++j) {
                ofs << "[" << joints[j].x << "," << joints[j].y << "," << joints[j].z << "]";
                if (j + 1 < joints.size()) ofs << ",";
            }
            ofs << "] }\n";
        }
    }
    if (init) {
        std::ofstream ofs(dir + "/gen_init.json");
        ofs << "{\n";
        ofs << "  \"d_goal0\": " << init->d_goal0 << ",\n";
        ofs << "  \"start_sid\": " << init->start_sid << ",\n";
        ofs << "  \"d_start_splat\": " << init->d_start_splat << ",\n";
        ofs << "  \"d_start_next\": " << init->d_start_next << ",\n";
        ofs << "  \"goal_sid\": " << init->goal_sid << ",\n";
        ofs << "  \"goal_stopability\": " << init->goal_stopability << "\n";
        ofs << "}\n";
    }
}
#endif
