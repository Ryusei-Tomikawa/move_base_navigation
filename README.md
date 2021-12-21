# move_base_navigation
既存のnavigationのパッケージのコードをいじったパッケージ

いじったといってもコストマップとグローバルプランナーに関して2.5DMapのためにいじっただけ

コストマップ・・・既存に対して１つレイヤーを斜面用に増やした

グローバルプランナー・・・上記のコストマップを使って新しいアルゴリズム（ダイクストラ法かA*を変更した）を適用したもの

## 使い方
costmap2d_ros.cpp やpluginを変更している
実験自体はtutlebot3を用いている

  ```shell
  $ roslaunch slope_world slopw_wolrd_turtlebot3.launch
  $ roslaunch turtlebot3_navigation navigation.launch
  ```
基本的にはこれでnavigationは動かすことが可能
コードをいじるならstatic_slope_layer.cppの中身をいじるとコストマップに関するコードをいじることができる
現状高さ値が付加された地図部分をコスト値に変換し、コストマップとして可視化することができている
あとは、このコストマップを用いて2.5Dアルゴリズムを用いてスロープを考慮したナビゲーションができれば終了
(12/20)

