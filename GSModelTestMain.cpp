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
#include "GSModelTest.h"

using namespace  std;

#include <filesystem>



//
//  メイン関数（プログラムはここから開始）
//
int  main( int argc, char ** argv )
{
	// 全サンプル動作データ
	std::vector< const Motion * >  sample_motions;
	
	// サンプル動作データの骨格情報
	const HumanBody *  sample_body;

	// 動作生成に使用するキー姿勢
	std::vector< Posture * >  sample_key_poses;

	// 学習モデル
	GSModel *  gsmodel;

	// 動作生成により生成されたキーフレーム動作
	KeyframeMotion *  generated_motion;

	// テスト入力（キー姿勢番号）
	int  test_input_no = 0;


	// ログを出力するディレクトリの設定
    namespace fs = std::filesystem;
    std::string  dump_dir = ( argc > 1 ) ? argv[ 1 ] : "gs_dump";
    try { 
		fs::create_directories( dump_dir ); 
	} catch (...) {}
	SetGSMDumpDirectory( dump_dir.c_str() );
	std::cout << "[HEADLESS] dump_dir=" << dump_dir << std::endl;

	// サンプル動作データの読み込み
	LoadSampleMotions( sample_motions, &sample_body, sample_key_poses );
	std::cout << "[HEADLESS] loaded motions=" << sample_motions.size() << std::endl;

	// モデルの学習
	gsmodel = TrainGSModel( sample_motions, sample_body );

	// 動作生成テスト
	generated_motion = GenerateTestMotion( test_input_no, sample_key_poses, gsmodel );
}

