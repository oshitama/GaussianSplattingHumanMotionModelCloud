/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  Gaussian Splatting モデルによる動作生成アプリケーション
**/


// ライブラリ・クラス定義の読み込み
#define NOMINMAX
#include "SimpleHuman.h"
#include "BVH.h"
#include "HumanBody.h"

#include "GSModel.h"
//#include "GSModelApp.h"

using namespace  std;


// 骨格の追加情報の設定（Rikiya）
void  SetPrimaryBodyPartsRikiya( HumanBody * hb );


// ログを出力するディレクトリの設定
string  gsm_dump_directory( "gs_dump" );


//
//  ログを出力するディレクトリの設定
//
void  SetGSMDumpDirectory( const char * dir )
{
	gsm_dump_directory = dir;
}


//
//  サンプル動作データの読み込み
//
void  LoadSampleMotions( std::vector< const Motion * > & sample_motions, const HumanBody ** sample_body, std::vector< Posture * > & sample_key_poses )
{
	// サンプル動作の情報（格闘動作）
	const int  num_sample_motions = 1;
        const char *  sample_motion_files[ num_sample_motions ] = {
                "motion_rikiya/I25.bvh" // パンチ
        };
	const float  sample_motion_keytimes[ num_sample_motions ][ 2 ] = {
		{ 0.0f, 1.75f } // パンチ
	};

	Motion *  new_motion = NULL;
	const Skeleton *  skeleton = NULL;
	HumanBody *  human_body = NULL;
	Posture *  key_posture = NULL;

	// BVHファイルのスケールの一時変更
	float  bvh_scale_org = GetBVHScale();
	SetBVHScale( 0.025f );

	// 動作データの読み込みと登録
	for ( int i = 0; i < num_sample_motions; i++ )
	{
		// 動作データの読み込み
		new_motion = LoadAndCoustructBVHMotion( sample_motion_files[ i ], skeleton );
		if ( !new_motion )
			continue;

		// 一つ目の動作データの骨格情報を記録
		if ( !skeleton )
			skeleton = new_motion->body;

		// サンプル動作データに追加
		sample_motions.push_back( new_motion );

		// キー姿勢に追加
		for ( int j = 0; j < 2; j++ )
		{
			key_posture = new Posture( skeleton );
			new_motion->GetPosture( sample_motion_keytimes[ i ][ j ], *key_posture );
			sample_key_poses.push_back( key_posture );
		}
	}

	// BVHファイルのスケールの復元
	SetBVHScale( bvh_scale_org );

	// 骨格情報の初期化
	human_body = new HumanBody( skeleton );
	SetPrimaryBodyPartsRikiya( human_body );
	*sample_body = human_body;
}


//
//  骨格の追加情報の設定（Rikiya）
//
void  SetPrimaryBodyPartsRikiya( HumanBody * hb )
{
	hb->SetPrimaryJoint( JOI_R_SHOULDER, "RightShoulder" );
	hb->SetPrimaryJoint( JOI_L_SHOULDER, "LeftShoulder" );
	hb->SetPrimaryJoint( JOI_R_ELBOW, "RightElbow" );
	hb->SetPrimaryJoint( JOI_L_ELBOW, "LeftElbow" );
	hb->SetPrimaryJoint( JOI_R_WRIST, "RightWrist" );
	hb->SetPrimaryJoint( JOI_L_WRIST, "LeftWrist" );
	hb->SetPrimaryJoint( JOI_R_HIP, "RightHip" );
	hb->SetPrimaryJoint( JOI_L_HIP, "LeftHip" );
	hb->SetPrimaryJoint( JOI_R_KNEE, "RightKnee" );
	hb->SetPrimaryJoint( JOI_L_KNEE, "LeftKnee" );
	hb->SetPrimaryJoint( JOI_R_ANKLE, "RightAnkle" );
	hb->SetPrimaryJoint( JOI_L_ANKLE, "LeftAnkle" );
	hb->SetPrimaryJoint( JOI_BACK, "Chest" );
	hb->SetPrimaryJoint( JOI_NECK, "Neck" );
	hb->SetPrimarySegment( SEG_R_FOOT, "RightAnkle" );
	hb->SetPrimarySegment( SEG_L_FOOT, "LeftAnkle" );
	hb->SetPrimarySegment( SEG_R_HAND, "RightWrist" );
	hb->SetPrimarySegment( SEG_L_HAND, "LeftWrist" );
	hb->SetPrimarySegment( SEG_PELVIS, "Hips" );
	hb->SetPrimarySegment( SEG_CHEST, "Chest" );
	hb->SetPrimarySegment( SEG_HEAD, "Neck" );
}


//
//  モデルの学習
//
GSModel *  TrainGSModel( std::vector< const Motion * > & sample_motions, const HumanBody * sample_body )
{
	if ( !sample_body || ( sample_motions.size() == 0 ) )
		return  NULL;

	// 学習オプション設定
    TrainOptions  topt;
    topt.sample_stride    = 1;
    topt.occ_sigma_m      = 0.05f;
    topt.merge_radius_m   = 0.03f;
    topt.stop_v_threshold = 0.15f;
    topt.dump.enabled = true;
    topt.dump.out_dir = gsm_dump_directory;

	// 学習
    GSModel model = GSModel::Fit( *sample_body, sample_motions, topt );
	GSModel * gsmodel = new GSModel( model );

	// ダンプオプション設定
    DumpOptions dopt;
	dopt.enabled = true;
 	dopt.out_dir = gsm_dump_directory;
    gsmodel->SetDefaultDump(dopt);

	return  gsmodel;
}


//
//  動作生成テスト
//
KeyframeMotion *  GenerateTestMotion( int no, const std::vector< Posture * > & sample_key_poses, GSModel * gsmodel )
{
	if ( ( no < 0 ) || ( no * 2 >= sample_key_poses.size() ) )
		return  NULL;

	// 開始・目標姿勢の設定
	const Posture *  start_posture = sample_key_poses[ no * 2 ];
	const Posture *  goal_posture = sample_key_poses[ no * 2 + 1 ];

	// 動作生成オプション設定
    GenerateOptions  gopt;
    gopt.tempo = 1.0f;
    gopt.dt_seconds = 1.0f / 30.0f;
    gopt.goal_tolerance_m = 0.02f;
    gopt.extend_to_stable = true;
    gopt.dump.enabled = true;
    gopt.dump.out_dir = gsm_dump_directory;

	// 動作生成
    KeyframeMotion  kf = gsmodel->Generate( *start_posture, *goal_posture, gopt );
	KeyframeMotion *  generated_motion = new KeyframeMotion( kf );

	return  generated_motion;
}
