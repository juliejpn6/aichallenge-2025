Author: yoshi <julie.jpn6@gmail.com>
Date:   Fri Oct 17 00:15:36 2025 +0900

    feat: ラップごとに自動で速度が上がる機能を実装
    
    ## 📝 実装内容
    
    ### 新機能
    - ゴールライン通過を自動検出
    - 1周ごとに速度を0.5m/sずつ自動増加
    - 初期速度は設定ファイル（launch）から自動取得
    
    ### 技術的な詳細
    
    #### 1. goal_checker パッケージ（新規作成）
    **役割：** ゴールライン通過の検出とパラメータ変更
    
    **主な機能：**
    - 車両位置とゴール位置の距離を計算
    - ゴールエリア（半径2m）への進入・退出を監視
    - ラップ完了時に目標速度を更新
    
    **実装ファイル：**
    - `goal_checker/src/goal_line_checker_node.cpp`
    - `goal_checker/CMakeLists.txt`
    - `goal_checker/package.xml`
    
    #### 2. goal_pose_setter の改修
    **変更内容：**
    - ゴール位置を `/planning/mission_planning/goal` トピックで配信
    - パラメータファイルからゴール座標を読み込み
    - 起動時と定期的にゴール位置をパブリッシュ
    
    **変更ファイル：**
    - `goal_pose_setter/src/goal_pose_setter_node.cpp`
    - `goal_pose_setter/src/goal_pose_setter_node.hpp`
    
    #### 3. simple_pure_pursuit の改修
    **変更内容：**
    - パラメータの動的変更に対応（const を削除）
    - パラメータ変更時のコールバック関数を追加
    - 変更時にログ出力で確認可能
    
    **変更ファイル：**
    - `simple_pure_pursuit/src/simple_pure_pursuit.cpp`
    - `simple_pure_pursuit/include/simple_pure_pursuit/simple_pure_pursuit.hpp`
    
    #### 4. 依存関係の追加
    **変更内容：**
    - aichallenge_submit_launch に goal_checker の依存を追加
    
    **変更ファイル：**
    - `aichallenge_submit_launch/package.xml`
    
    ## 🎯 使い方
    
    ### 初期速度の変更方法
    `reference.launch.xml` の以下の行を編集：
    ```xml
    <param name="external_target_vel" value="4.0"/>
    ```
    
    ### 速度増加量の変更方法
    `goal_checker/src/goal_line_checker_node.cpp` の以下の行を編集：
    ```cpp
    velocity_increment_ = 0.5;  // この値を変更
    ```
    
    ## 📊 動作確認方法
    
    ### ログ監視コマンド
    ```bash
    ros2 topic echo /rosout | grep "external_target_vel updated"
    ```
    
    ### パラメータ確認コマンド
    ```bash
    watch -n 0.5 'ros2 param get /simple_pure_pursuit_node external_target_vel'
    ```
    
    ## ⚠️ 注意事項
    
    - パラメータ変更は車両PC内で行われるため、運営ルールに抵触しません
    - 速度を上げすぎるとコースアウトの危険があります
    - 初期テストは低速（4.0 m/s程度）から始めることを推奨
    
    ## 🔗 関連Issue
:
    - `simple_pure_pursuit/src/simple_pure_pursuit.cpp`
    - `simple_pure_pursuit/include/simple_pure_pursuit/simple_pure_pursuit.hpp`
    
    #### 4. 依存関係の追加
    **変更内容：**
    - aichallenge_submit_launch に goal_checker の依存を追加
    
    **変更ファイル：**
    - `aichallenge_submit_launch/package.xml`
    
    ## 🎯 使い方
    
    ### 初期速度の変更方法
    `reference.launch.xml` の以下の行を編集：
    ```xml
    <param name="external_target_vel" value="4.0"/>
    ```
    
    ### 速度増加量の変更方法
    `goal_checker/src/goal_line_checker_node.cpp` の以下の行を編集：
    ```cpp
    velocity_increment_ = 0.5;  // この値を変更
    ```
    
    ## 📊 動作確認方法
    
    ### ログ監視コマンド
    ```bash
    ros2 topic echo /rosout | grep "external_target_vel updated"
    ```
    
    ### パラメータ確認コマンド
    ```bash
    watch -n 0.5 'ros2 param get /simple_pure_pursuit_node external_target_vel'
    ```
    
    ## ⚠️ 注意事項
    
    - パラメータ変更は車両PC内で行われるため、運営ルールに抵触しません
    - 速度を上げすぎるとコースアウトの危険があります
    - 初期テストは低速（4.0 m/s程度）から始めることを推奨
    
    ## 🔗 関連Issue
    
    （該当するIssue番号があれば記載）
:
    
    #### 4. 依存関係の追加
    **変更内容：**
    - aichallenge_submit_launch に goal_checker の依存を追加
    
    **変更ファイル：**
    - `aichallenge_submit_launch/package.xml`
    
    ## 🎯 使い方
    
    ### 初期速度の変更方法
    `reference.launch.xml` の以下の行を編集：
    ```xml
    <param name="external_target_vel" value="4.0"/>
    ```
    
    ### 速度増加量の変更方法
    `goal_checker/src/goal_line_checker_node.cpp` の以下の行を編集：
    ```cpp
    velocity_increment_ = 0.5;  // この値を変更
    ```
    
    ## 📊 動作確認方法
    
    ### ログ監視コマンド
    ```bash
    ros2 topic echo /rosout | grep "external_target_vel updated"
    ```
    
    ### パラメータ確認コマンド
    ```bash
    watch -n 0.5 'ros2 param get /simple_pure_pursuit_node external_target_vel'
    ```
    
    ## ⚠️ 注意事項
    
    - パラメータ変更は車両PC内で行われるため、運営ルールに抵触しません
    - 速度を上げすぎるとコースアウトの危険があります
    - 初期テストは低速（4.0 m/s程度）から始めることを推奨
    
    ## 🔗 関連Issue
    
    （該当するIssue番号があれば記載）
:
    
    #### 4. 依存関係の追加
    **変更内容：**
    - aichallenge_submit_launch に goal_checker の依存を追加
    
    **変更ファイル：**
    - `aichallenge_submit_launch/package.xml`
    
    ## 🎯 使い方
    
    ### 初期速度の変更方法
    `reference.launch.xml` の以下の行を編集：
    ```xml
    <param name="external_target_vel" value="4.0"/>
    ```
    
    ### 速度増加量の変更方法
    `goal_checker/src/goal_line_checker_node.cpp` の以下の行を編集：
    ```cpp
    velocity_increment_ = 0.5;  // この値を変更
    ```
    
    ## 📊 動作確認方法
    
    ### ログ監視コマンド
    ```bash
    ros2 topic echo /rosout | grep "external_target_vel updated"
    ```
    
    ### パラメータ確認コマンド
    ```bash
    watch -n 0.5 'ros2 param get /simple_pure_pursuit_node external_target_vel'
    ```
    
    ## ⚠️ 注意事項
    
    - パラメータ変更は車両PC内で行われるため、運営ルールに抵触しません
    - 速度を上げすぎるとコースアウトの危険があります
    - 初期テストは低速（4.0 m/s程度）から始めることを推奨
    
    ## 🔗 関連Issue
    
    （該当するIssue番号があれば記載）
:
    
    #### 4. 依存関係の追加
    **変更内容：**
    - aichallenge_submit_launch に goal_checker の依存を追加
    
    **変更ファイル：**
    - `aichallenge_submit_launch/package.xml`
    
    ## 🎯 使い方
    
    ### 初期速度の変更方法
    `reference.launch.xml` の以下の行を編集：
    ```xml
    <param name="external_target_vel" value="4.0"/>
    ```
    
    ### 速度増加量の変更方法
    `goal_checker/src/goal_line_checker_node.cpp` の以下の行を編集：
    ```cpp
    velocity_increment_ = 0.5;  // この値を変更
    ```
    
    ## 📊 動作確認方法
    
    ### ログ監視コマンド
    ```bash
    ros2 topic echo /rosout | grep "external_target_vel updated"
    ```
    
    ### パラメータ確認コマンド
    ```bash
    watch -n 0.5 'ros2 param get /simple_pure_pursuit_node external_target_vel'
    ```
    
    ## ⚠️ 注意事項
    
    - パラメータ変更は車両PC内で行われるため、運営ルールに抵触しません
    - 速度を上げすぎるとコースアウトの危険があります
    - 初期テストは低速（4.0 m/s程度）から始めることを推奨
    
    ## 🔗 関連Issue
    
    （該当するIssue番号があれば記載）
:
    - `simple_pure_pursuit/include/simple_pure_pursuit/simple_pure_pursuit.hpp`
    
    #### 4. 依存関係の追加
    **変更内容：**
    - aichallenge_submit_launch に goal_checker の依存を追加
    
    **変更ファイル：**
    - `aichallenge_submit_launch/package.xml`
    
    ## 🎯 使い方
    
    ### 初期速度の変更方法
    `reference.launch.xml` の以下の行を編集：
    ```xml
    <param name="external_target_vel" value="4.0"/>
    ```
    
    ### 速度増加量の変更方法
    `goal_checker/src/goal_line_checker_node.cpp` の以下の行を編集：
    ```cpp
    velocity_increment_ = 0.5;  // この値を変更
    ```
    
    ## 📊 動作確認方法
    
    ### ログ監視コマンド
    ```bash
    ros2 topic echo /rosout | grep "external_target_vel updated"
    ```
    
    ### パラメータ確認コマンド
    ```bash
    watch -n 0.5 'ros2 param get /simple_pure_pursuit_node external_target_vel'
    ```
    
    ## ⚠️ 注意事項
    
    - パラメータ変更は車両PC内で行われるため、運営ルールに抵触しません
    - 速度を上げすぎるとコースアウトの危険があります
    - 初期テストは低速（4.0 m/s程度）から始めることを推奨
    
    ## 🔗 関連Issue
