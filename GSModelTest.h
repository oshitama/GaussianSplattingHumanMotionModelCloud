/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  Gaussian Splatting モデルによる動作生成アプリケーション
**/

#ifndef  _GS_MODEL_TEST_H_
#define  _GS_MODEL_TEST_H_


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"

class  HumanBody;
class  GSModel;


// ログを出力するディレクトリの設定
void  SetGSMDumpDirectory( const char * dir );

// サンプル動作データの読み込み
void  LoadSampleMotions( std::vector< const Motion * > & sample_motions, const HumanBody ** sample_body, std::vector< Posture * > & sample_key_poses );

// 骨格の追加情報の設定（Rikiya）
void  SetPrimaryBodyPartsRikiya( HumanBody * hb );

// モデルの学習
GSModel *  TrainGSModel( std::vector< const Motion * > & sample_motions, const HumanBody * sample_body );

// 動作生成テスト
KeyframeMotion *  GenerateTestMotion( int no, const std::vector< Posture * > & sample_key_poses, GSModel * gsmodel );


#endif // _GS_MODEL_TEST_H_
