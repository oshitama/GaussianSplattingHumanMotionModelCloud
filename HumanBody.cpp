/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  人体モデルの骨格の追加情報
**/


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "HumanBody.h"

using namespace  std;


//
//  コンストラクタ
//
HumanBody::HumanBody()
{
	skeleton = NULL;
	for ( int i = 0; i < NUM_PRIMARY_SEGMENTS; i++ )
		primary_segments[ i ] = -1;
	for ( int i = 0; i < NUM_PRIMARY_JOINTS; i++ )
		primary_joints[ i ] = -1;
	for ( int i = 0; i < NUM_LIMBS; i++ )
		limbs[ i ] = NULL;
	body_height = 0.0f;
	pelvis_height = 0.0f;
	segment_weights = NULL;
	segment_inertia = NULL;
}

HumanBody::HumanBody( const Skeleton * body ) : HumanBody()
{
	SetSkeleton( body );
}


//
//  デストラクタ
//
HumanBody::~HumanBody()
{
	if ( segment_weights )
		delete  segment_weights;
	if ( segment_inertia )
		delete[]  segment_inertia;
}


//
//  対象の骨格情報の設定
//
void  HumanBody::SetSkeleton( const Skeleton * body )
{
	skeleton = body;

	// 高さの設定（骨格モデルから計算）
	SetHeightsFromSkeleton();
}
	

//
//  主要な部位の体節番号の設定（番号で設定）
//
void  HumanBody::SetPrimarySegment( PrimarySegmentType segment, int no )
{
	primary_segments[ segment ] = no;
}


//
//  主要な部位の関節番号の設定（番号で設定）
//
void  HumanBody::SetPrimaryJoint( PrimaryJointType joint, int no )
{
	primary_joints[ joint ] = no;
}


//
//  主要な部位の体節番号の設定（名前で設定）
//
void  HumanBody::SetPrimarySegment( PrimarySegmentType segment, const char * name )
{
	if ( !skeleton )
		return;

	for ( int i = 0; i < skeleton->num_segments; i++ )
	{
		if ( strstr( skeleton->segments[ i ]->name.c_str(), name ) )
		{
			primary_segments[ segment ] = i;
			break;
		}
	}
}


//
//  主要な部位の関節番号の設定（名前で設定）
//
void  HumanBody::SetPrimaryJoint( PrimaryJointType joint, const char * name )
{
	if ( !skeleton )
		return;

	for ( int i = 0; i < skeleton->num_joints; i++ )
	{
		if ( strstr( skeleton->joints[ i ]->name.c_str(), name ) )
		{
			primary_joints[ joint ] = i;
			break;
		}
	}
}


/*
//
//  逆運動学計算（解析的手法）のための手足の情報の設定
//
void  HumanBody::SetLimbParameter( LimbType limb, LimbParameter * info )
{
	limbs[ limb ] = info;
}
*/

//
//  高さの設定（骨格モデルから計算）
//
void  HumanBody::SetHeightsFromSkeleton()
{
	if ( !skeleton )
		return;

	// 計算用変数
	float  min_height = 0.0f, max_height = 0.0f, height = 0.0f;
	Vector3f  vec;
	Point3f  pos;
	Posture  posture( skeleton );
	const Segment *  seg = NULL;

	// 順運動学計算
	vector< Matrix4f >  seg_frame_array;
	ForwardKinematics( posture, seg_frame_array );

	// 全ての体節の位置を求めて最高点・最低点を取得
	for ( int i = 0; i < skeleton->num_segments; i++ )
	{
		// 体節の位置を計算
		seg = skeleton->segments[ i ];
		if ( seg->has_site )
		{
			seg_frame_array[ i ].transform( seg->site_position, &pos );
			height = pos.y;
		}
		else
		{
			seg_frame_array[ i ].get( &vec );
			height = vec.y;
		}

		// 最高点・最低点を更新
		if ( height < min_height )
			min_height = height;
		if ( height > max_height )
			max_height = height;
	}

	// 身長・腰の高さを計算
	body_height = max_height - min_height;
	pelvis_height = -min_height;
}


//
//  体節の質量・慣性モーメント行列の設定
//
void  HumanBody::SetPhysicalProperties( const float * weights, const Matrix3f * inertia )
{
	if ( !skeleton )
		return;

	if ( segment_weights )
		delete  segment_weights;
	if ( segment_inertia )
		delete[]  segment_inertia;

	segment_weights = new float[ skeleton->num_segments ];
	segment_inertia = new Matrix3f[ skeleton->num_segments ];

	memcpy( segment_weights, weights, sizeof( float ) * skeleton->num_segments );
	memcpy( segment_inertia, inertia, sizeof( Matrix3f ) * skeleton->num_segments );
}


//
//  主要な部位の体節・関節番号の取得
//
bool  HumanBody::GetPrimaryBodyPart( PrimaryBodyPartType body_part, int & segment_no, int & joint_no ) const
{
	if ( ( body_part < BODY_SEG_R_FOOT ) || ( body_part > BODY_JOI_NECK ) )
		return  false;

	if ( ( body_part >= BODY_SEG_R_FOOT ) && ( body_part <= BODY_SEG_HEAD ) )
	{
		segment_no = GetPrimarySegment( (PrimarySegmentType) body_part );
		joint_no = -1;
		return  true;
	}
	else
	{
		joint_no = GetPrimaryJoint( (PrimaryJointType) ( body_part - BODY_JOI_R_SHOULDER ) );
		segment_no = -1;
		return  false;
	}
}

