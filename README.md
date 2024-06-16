# 双腕ロボットによる食材切断のための食材把持方法
# 概要
## 研究背景
近年，家事ロボットへの関心が高まっている．
様々な家事作業の中でも，料理は日常生活に欠かせないものといえる．
そして，料理作業の中で，「切る」作業はもっとも重要な作業の一つである．

食材の切断には，食材を安定して把持することが重要となる．
原因：
1. 意図せずの移動（包丁が発生させた）は把持で抑えられる．
2. 摩擦力が足りないときに食材の移動を抑えられる．
3. 切断作業の効率を向上させる．

切断中の食材を安定して把持するには，以下のような難しさがある：
1. 食材の形状は多様である．
2. 把持の適切さを評価するのが難しい．
3. 包丁力を抵抗するための指先力の冗長性を効率よく利用するのも難しい．

## 先行研究
- 「把持が必要なロボットの作業（材料の切断）」の研究：
  - 作業の実現のみに集中していて，材料の固定はほぼ考慮しませんでした．
- 「把持」の研究：
  - 安定に物体を持ち上げることを目的とし，把持後の作業を考慮しませんでした．

そこで，私は「作業」と「把持」両方を考慮する把持方法を開発したいと思います．

## 目的
以上の観点を踏まえて，本論文は：
- 環境
  - ロボットアーム
  - 手先の3自由度の2本指ハンド．
- ある包丁の状態（姿勢や速度など）において，把持位置を評価することで
  - 指先位置情報による評価．
  - 指先力情報（方向や大きさなど）による評価．
    - 各指の合力は包丁からの力と釣り合うのが前提．
- 最終的に評価が高い把持位置を出力する．
  - その包丁の状態において，食材の姿勢を維持できる把持位置を．

具体的な開発内容は
1. 包丁動作の分析や計算．
2. 把持の評価アルゴリズムの設計と実装．
3. 実験するため，多指ハンドの設計と製造．


# 把持位置を評価するアルゴリズム
## 全体の流れ
![全体の流れ](image.png)

1. 1行目：包丁の軌道$s$から，ある時間$t$の包丁の情報$\mathrm{T}_\mathrm{j}$（姿勢と速度）を取る．
2. 2行目：「PrepareData」関数は包丁情報$\mathrm{T}_\mathrm{j}$と食材表面情報$\Omega$を利用し，食材と包丁の接触面$\Omega_{\mathrm{c}}$（包丁からの力を計算するため）や把持に使う表面$\Omega_{\mathrm{g}}$（$\Omega$を$\Omega_{\mathrm{c}}$で分割した片方の表面），初期探索空間 $\mathbb{P}_\mathrm{init}$を生成する．
	- ｄｄ
3. 各包丁姿勢の各把持位置の点数を計算する．
4. 各包丁姿勢から計算された点数を足し合わせて，この\nameholdingpointset の点数となる．
5. 点数が一番高い\nameholdingpointset を出力する．

具体的に，ある時間$t$の包丁の情報$\mathrm{T}_\mathrm{j}$（姿勢と速度）に対して：
まず，「PrepareData」関数は包丁情報$\mathrm{T}_\mathrm{j}$と食材表面情報$\Omega$を利用し，食材と包丁の接触面$\Omega_{\mathrm{c}}$（包丁からの力を計算するため）や把持に使う表面$\Omega_{\mathrm{g}}$（$\Omega$を$\Omega_{\mathrm{c}}$で分割した片方の表面），初期\namesearchspace $\equsearchspace_\mathrm{init}$を生成する．

\namesearchspace $\equsearchspace$は，\nameholdingpointset $\equholdingpointset$の集合であり，$\equsearchspace = \{\equholdingpointset[1], \equholdingpointset[2], \cdots \}$と定義する．

\nameholdingpointset $\equholdingpointset$は，$n$個の\namefingerpoint $\equfingerpoint[i]$ の集合である（つまり，$\equholdingpointset = \{\equfingerpoint[1], \equfingerpoint[2], \cdots, \equfingerpoint[][n]\}$）．$n$は食材への接触点の数である（指の本数）．

\namefingerpoint $\equfingerpoint[][i] ($i = 1,\ldots,n$)$は，指と食材の接触点座標であり，三次元のベクトルである．
% \namesearchspace $\equsearchspace$について，
% \namefingerpoint $\equfingerpoint[][i] ($i = 1,\ldots,n$)$は，指と食材の接触点座標であり，三次元のベクトルである．
% \nameholdingpointset $\equholdingpointset$は，$n$個の\namefingerpoint $\equfingerpoint[i]$ の集合である（つまり，$\equholdingpointset = \{\equfingerpoint[1], \equfingerpoint[2], \cdots, \equfingerpoint[][n]\}$）．$n$は食材への接触点の数である（指の本数）．
% \nameholdingpointset の候補の集合を\namesearchspace $\equsearchspace = \{\equholdingpointset[1], \equholdingpointset[2], \cdots \}$とよぶ．
% 
次に，「FilterByGeoScore」関数は初期\namesearchspace $\equsearchspace_\mathrm{init}$を削減する．\namesearchspace の中で使えない\nameholdingpointset が大量に存在する（例えば，指同士が近いや指が接触面の点に近いなど）ため，正式の計算が開始する前に不適切な\nameholdingpointset を削除する．
% 
また，「CalKnifeForce」関数は包丁の力$t_\mathrm{k}$（力とトルク，6次元のベクトル）を計算する．
% 
さらに，5-8行は包丁力$t_\mathrm{k}$と\nameholdingpointset で\namesearchspace $\equsearchspace$にあるすべての\nameholdingpointset に点数を計算し，\nameholdingpointset の最終点数$D_\mathrm{fin}[\equholdingpointset]$に累積する．
最後に，「FindPointSetWithMaxScore」関数は最終点数$D_\mathrm{fin}$の中で一番高い点数\nameholdingpointset $\equholdingpointset[fin]$を最終\nameholdingpointset として出力する．

# 把持位置を評価するプログラム
## 依頼
| 名前         | バージョン | 目的                              | URL                                                   |
| ------------ | ---------- | --------------------------------- | ----------------------------------------------------- |
| Eigen        | 3.3.7      | 行列演算                          | https://eigen.tuxfamily.org/index.php?title=Main_Page |
| json         | 3.11.3     | Jsonシリアライザー                | https://github.com/nlohmann/json                      |
| spdlog       | 1.14.0     | ログ                              | https://github.com/gabime/spdlog                      |
| cppitertools | master     | イテレータのライブラリ            | https://github.com/ryanhaining/cppitertools           |
| CGAL         | 5.6.0      | EMSTの生成                        | https://github.com/cgal/cgal                          |
| pcl          | 1.12       | 点群の処理                        | https://github.com/PointCloudLibrary/pcl              |
| vtk          | 9.2        | pclが依頼する可視化ライブラリ     | https://gitlab.kitware.com/vtk/vtk                    |
| pybind11     | master     | C/C++とPythonを連携するライブラリ | https://github.com/pybind/pybind11                    |

## 流れ

1. 

## 各部分の説明



## 実行




