#pragma once
// GSModel.h : ガウシアンスプラットベースの最小モーションモデル

// 依存ライブラリ
#include <vector>
#include <string>
#include <memory>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <limits>
#include <cmath>
#include <algorithm>

// ユーザ提供ライブラリ
#define NOMINMAX
#include "SimpleHuman.h"   // Skeleton / Posture / Motion / KeyframeMotion / FK / 補間 など
#include "HumanBody.h"     // HumanBody（主要関節情報・身長など・Skeleton取得）

// ダンプ出力の有効/無効（ビルド時に -DGSM_ENABLE_DUMP=1 などで切替）
#ifndef GSM_ENABLE_DUMP
#define GSM_ENABLE_DUMP 1
#endif

// -------------- 設計概要 --------------
//
// * GaussianSplat：高次元姿勢空間の局所代表（占有中心 = 代表姿勢）
//   - mean_pose       : スプラットの中心姿勢（占有）
//   - next_pose       : その姿勢から「1ステップ先」に遷移した代表姿勢（学習データ由来）
//   - occ_sigma_m     : FK距離空間での等方ガウス半径（m）。近傍判定や重み付けに利用。
//   - stopability     : [0..1] 停止「可能性」スコア（1=その場で自由に止まりやすい）
//   - v_norm_ref      : 代表的な遷移速度（FK距離[m]/s）
//   - v_norm_min/max  : 許容速度レンジ（生成のテンポ制御で使用）
//   - source_*        : デバッグ用（どのMotionの何フレーム由来か）
//
// * FK距離：ForwardKinematicsで得た全関節ワールド位置の差の二乗和に基づく距離。
//            root平行移動の影響を除去するため、各関節座標からそれぞれのroot_posを減算して比較。
//            （姿勢の“形”に寄与する成分を優先）
//
// * GSModel：
//   - HumanBody を内部に保持（Skeleton一貫性のため）。生成時に姿勢のbody一致を検証。
//   - スプラット集合（std::vector<GaussianSplat>）。
//   - 生成API：開始/目標姿勢＋テンポから KeyframeMotion を生成。
//   - 学習は Builder を通じて（逐次Add → Build）。Motion配列からの一括Fitも可能。
//
// * 生成アルゴリズム（最小版）
//   1) 現在姿勢Pから最近傍スプラットSをFK距離で取得
//   2) モデル推奨遷移Q_model = S.next_pose、目標引力Q_goal = 目標姿勢G
//   3) 停止可能性s = S.stopability に応じて Q = blend(Q_model, Q_goal)
//      - 非停止(s≈0)：データに沿ってQ_model寄り
//      - 停止可(s≈1)：目標へ寄せる
//   4) 速度ノルム v を [v_min, v_max] にクランプし、Δtから補間率を決めて P ← interp(P, Q)
//   5) 終了条件（Gに十分近い）で停止。目標が非停止なら、停止可になるまで延長。
//
// ------------------------------------

// デバッグダンプ設定
struct DumpOptions {
    bool        enabled = false;      // 実行時のダンプON/OFF（ビルド時に無効化されていれば無視）
    std::string out_dir = "gs_dump";  // 出力先ディレクトリ
    int         detail  = 1;          // 0:概要のみ, 1:通常, 2:詳細
};

// 生成開始時の診断ログ（1回だけ）
struct GenerateInitLog {
    float d_goal0 = -1.0f;     // 初期のゴール距離（FK距離; root平行移動除去）
    int   start_sid = -1;      // 開始姿勢の最近傍スプラットID
    float d_start_splat = -1.0f; // 開始→最近傍スプラット中心距離
    float d_start_next  = -1.0f; // 開始→そのスプラットの next_pose 距離
    int   goal_sid  = -1;      // ゴール姿勢の最近傍スプラットID
    float goal_stopability = -1.0f; // その停止可能性
};

// スプラットの最小構造（先行議論のうち「必要確定」分のみ）
struct GaussianSplat {
    int      id = -1;                 // 識別子（0..）
    Posture  mean_pose;               // 占有中心（代表姿勢）
    Posture  next_pose;               // 1ステップ先の代表姿勢（学習データから取得）
    bool     has_next = true;         // 学習末尾などで next が無い場合は false

    // 簡易ガウス（等方、FK距離[m]空間）
    float    occ_sigma_m = 0.05f;     // 占有半径（メートル相当）

    // 速度と停止性
    float    stopability = 0.5f;      // [0..1] 1:停止しやすい（両足支持など）、0:停止しにくい（空中等）
    float    v_norm_ref  = 0.5f;      // 代表速度（m/s; FK距離換算）
    float    v_norm_min  = 0.2f;      // 最小速度（生成時のレンジ下限）
    float    v_norm_max  = 1.5f;      // 最大速度（生成時のレンジ上限）

    // 由来情報（デバッグ用）
    std::string source_motion;
    int         source_frame = -1;
    float       source_interval = 1.0f;
};

// 学習オプション（最小）
struct TrainOptions {
    int   sample_stride     = 1;      // 学習時のフレーム間引き
    float occ_sigma_m       = 0.05f;  // スプラット占有半径（等方）
    float merge_radius_m    = 0.03f;  // 代表姿勢の近傍マージ半径（FK距離[m]）
    bool  enable_merge      = true;   // 近傍マージの有無
    float stop_v_threshold  = 0.15f;  // v_norm_ref がこの値未満なら「停止可」に寄せる
    DumpOptions dump;                 // モデル構築時のダンプ
};

// 生成オプション（最小）
struct GenerateOptions {
    float tempo             = 1.0f;   // 全体のテンポ倍率（1.0=学習相当）
    float dt_seconds        = 1.0f / 30.0f; // 1ステップの基準Δt（未指定なら30Hz想定）
    float goal_tolerance_m  = 0.02f;  // ゴール到達判定（FK距離[m]）
    float stopability_th    = 0.6f;   // s >= th を「停止可」とみなす
    int   max_steps         = 600;    // 安全上限（20秒@30Hz）
    bool  extend_to_stable  = true;   // ゴールが非停止なら安定姿勢まで延長
    float v_floor_mps       = 0.20f; // 最低速度[m/s]（FK距離換算）。パンチ等で進みを確保
    DumpOptions dump;                 // 生成時のダンプ
};

// 前方宣言
class GSModelBuilder;

// メインモデル
class GSModel {
public:
    // HumanBodyを必須で受け取り内部保持。Skeletonは外部から渡さない。
    explicit GSModel(const HumanBody& human);

    // スケルトン一致確認（Posture.bodyがHumanBodyのSkeletonと一致か）
    bool IsCompatible(const Posture& p) const;

    // スプラット集合の参照
    const std::vector<GaussianSplat>& GetSplats() const { return splats_; }

    // モデルにHumanBodyを含んでいるので、生成時にHumanBodyは不要
    // 生成：開始姿勢・目標姿勢・テンポ→KeyframeMotion
    KeyframeMotion Generate(const Posture& start,
                            const Posture& goal,
                            float tempo = 1.0f) const;

    KeyframeMotion Generate(const Posture& start,
                            const Posture& goal,
                            const GenerateOptions& opt) const;

    // 一括学習ユーティリティ
    static GSModel Fit(const HumanBody& human,
                       const std::vector<const Motion*>& motions,
                       const TrainOptions& opt);

#if GSM_ENABLE_DUMP
    // 実行時ダンプの既定設定を変更（モデル生成時/動作生成時に使われる）
    void SetDefaultDump(const DumpOptions& dump) { default_dump_ = dump; }
#endif

private:
    friend class GSModelBuilder;

    HumanBody human_;                      // モデル内に保持（Skeleton一貫性の源）
    std::vector<GaussianSplat> splats_;    // スプラット集合

#if GSM_ENABLE_DUMP
    DumpOptions default_dump_;             // 既定ダンプ設定
#endif

    // --- ヘルパ ---
    // FKで全関節ワールド位置を取得（joint配列サイズは body->num_joints）
    void FKJointPositions(const Posture& p, std::vector<Point3f>& joints) const;

    // FK距離（root平行移動を除去）
    float FKDistance(const Posture& a, const Posture& b) const;

    // 最近傍スプラット探索（線形走査の最小版）
    int FindNearestSplat(const Posture& p, float* out_dist = nullptr) const;

    // 速度ノルムのクランプ
    static float Clamp(float x, float lo, float hi) {
        return std::max(lo, std::min(hi, x));
    }

#if GSM_ENABLE_DUMP
    // ダンプ
    void DumpModel(const std::string& dir) const;
    struct StepLog {
        int   step;
        int   splat_id;
        float t;
        float dist_goal;
        float stopability;
        float v_ref;
        float v_used;
        float dist_model;   // cur → モデル推奨(next_pose) の距離
        float r_model;      // その一歩の補間率
        float r_goal;       // ゴール側一歩の補間率
        float alpha;        // p1/p2 の混合係数
    };
    void DumpGenerateTrace(const std::string& dir,
                           const std::vector<StepLog>& logs,
                           const std::vector<float>& times,
                           const std::vector<Posture>& poses,
                           const GenerateInitLog* init) const;
#endif
};

// 逐次学習ビルダ（Motionを複数回Add→Build）
class GSModelBuilder {
public:
    explicit GSModelBuilder(const HumanBody& human, const TrainOptions& opt);

    // Motionを追加（学習データ）
    void AddMotion(const Motion& m);

    // モデルを構築
    GSModel Build() const;

private:
    HumanBody human_;
    TrainOptions opt_;
    std::vector<const Motion*> motions_;  // 参照保持（寿命は呼び出し側で確保してください）

    // スプラット作成ヘルパ
    void AppendMotionSplats(const Motion& m, std::vector<GaussianSplat>& out) const;

    // 近傍マージ
    void MergeNearby(std::vector<GaussianSplat>& splats) const;
};
