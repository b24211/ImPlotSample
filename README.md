# ImPlotSample - Waveform Analysis & Bode Plot Fork

This project is a fork of [daigokk/ImPlotSample](https://github.com/daigokk/ImPlotSample).  
This fork extends the original ImPlot implementation to include real-time instrument control via VISA, Bode plot sweeping, and waveform analysis.

本プロジェクトは `daigokk/ImPlotSample` をフォークし、VISAライブラリを用いた計測機器（オシロスコープ・ファンクションジェネレータ）の制御機能と、波形解析・ボード線図作成機能を追加したものです。

## Added Features (追加機能)

### 1. Waveform Analysis App (波形解析・フィルタ)
- **Signal Generation**: Generate waveforms (Sine, Square, Triangle, Sawtooth) via Simulation or Function Generator (VISA).
- **Oscilloscope Capture**: Fetch real-time waveform data from a connected oscilloscope via VISA.
- **Digital Filter**: Apply LPF, HPF, BPF to the signal and view results in real-time.
- **FFT Analysis**: Real-time FFT display of raw and filtered signals.
- **CSV Export**: Save captured waveforms and FFT data.

### 2. Bode Plot Sweep App (ボード線図スイープ)
- **Frequency Sweep**: Automatically controls the Function Generator to sweep frequencies.
- **Gain/Phase Calculation**: Fetches Input/Output waveforms from the Oscilloscope and calculates Gain (dB) and Phase (deg) using relative phase PSD.
- **Real-time Plotting**: Plots the Bode diagram (Gain & Phase) in real-time during the sweep.

## Prerequisites (必須環境)

To build and run this project, you need a VISA library installed:
ビルドおよび実行には VISA ライブラリが必要です。

- **NI-VISA** or **Keysight IO Libraries Suite**
- **Architecture**: x64 (Required for `visa64.lib`)

## Build Settings (ビルド設定)

1. **Platform**: Set Visual Studio build platform to **`x64`**.
2. **Linker**: Ensure the VISA library path is added to the Linker settings.
   - Typically: `C:\Program Files\IVI Foundation\VISA\Win64\Lib_x64\msc`
   - Library file: `visa64.lib`

## Usage (使い方)

Run the application and select the mode in the console:

```text
========================================
 Select Application to Run:
========================================
 1. Waveform Analysis & Filter
 2. Bode Plot & Sweep Control
========================================
 Enter number (1 or 2):
```


The following is the same as the original fork:

# ImPlotSample
このリポジトリ(ソースコードやファイルを一元的に管理する場所)が管理しているプログラムは[ImPlot](https://github.com/epezent/implot)をVisual Studioで利用するためのひな型です。ImPlotを使うためには、環境に合わせたいくつかの作業が必要になり、それをショートカットするために用意しました。ご自身で0から環境構築をしたい場合、本ページ内のリンク先等が参考になります。

  ![Hard copy](./docs/images/HardCopy.png)

## 概要
- このリポジトリは[GLFW](https://www.glfw.org/)、[Dear ImGui](https://github.com/ocornut/imgui)、ImPlot、及び[Visual Studio](https://visualstudio.microsoft.com/ja/vs/community/)に依存しています。
- ImPlotとは、データをリアルタイムで視覚化したり、インタラクティブなプロットを作成することを目的としたImGuiを利用したライブラリです。
- ImGuiとは、GLFW等を用いた3Dプログラム上で直感的かつ簡潔なコードによってGUIを作成することを目的としたC++言語用ライブラリです。
- GLFWとは、3Dプログラムを作りやすくするためのC,C++言語用ライブラリです。
- Visual StudioはC言語やC++言語をWindows上で開発することができる統合開発環境です。
- つまり、計測したデータを視覚化するImPlotを使うためには、ImGuiとGLFWとの依存関係をVisual Studio上で正しく解決する必要があり、それは初学者にとってやや複雑なので、それらを行ったこのリポジトリを用意した次第です。
- [職業能力開発総合大学校 電気工学専攻](https://www.uitec.jeed.go.jp/department/sogo/about/sogo_d_denki.html)の「[自動計測実習](https://daigokk.github.io/ImPlotSample/)」で用います。

## 使い方
1. [`https://github.com/daigokk/ImPlotSample/tree/master`](https://github.com/daigokk/ImPlotSample/tree/master)右上の緑のボタン「<>Code▽」をクリックし、現れたメニューの一番下`Download Zip`をクリック。すると[ダウンロード](https://github.com/daigokk/ImPlotSample/archive/refs/heads/master.zip)が始まります。
1. ダウンロードされた`ImPlotSample-master.zip`を「すべて展開」し、任意の場所(学校ではZドライブを推奨)に保存します。
1. `ImPlotSample.sln`ファイルをダブルクリックするとVisual Studioが起動します。もちろんVisual Studioがインストールされている必要があります。Visual Studio のバージョンによっては初期設定ではビルドできない場合があります。[ここ](https://github.com/daigokk/ImPlotSample/blob/master/docs/2_1_GenerateWaveform.md#%E3%83%80%E3%82%A6%E3%83%B3%E3%83%AD%E3%83%BC%E3%83%89%E3%81%97%E3%81%9Fsln%E3%81%8C%E3%83%93%E3%83%AB%E3%83%89%E3%81%A7%E3%81%8D%E3%81%AA%E3%81%84%E5%A0%B4%E5%90%88)が問題解決の参考になるかもしれません。
- Visual Studioにはgithubのリポジトリをクローンする機能がプロジェクトの新規作成画面からアクセスできます。こちらのほうが便利かもしれません。

---

# 🧱 ImGui ウィジェット使用例チートシート

## 🟦 基本ウィジェット

| ウィジェット | 説明 | 使用例 |
|--------------|------|--------|
| `Text()` | テキスト表示 | `ImGui::Text("Hello, world!");` |
| `Button()` | ボタン | `if (ImGui::Button("Click Me")) { /* 処理 */ }` |
| `Checkbox()` | チェックボックス | `bool checked = false; ImGui::Checkbox("Enable", &checked);` |
| `RadioButton()` | ラジオボタン | `int mode = 0; ImGui::RadioButton("Mode A", mode == 0);` |
| `InputText()` | 文字入力欄 | `char buf[64] = ""; ImGui::InputText("Name", buf, 64);` |
| `InputFloat()` | 浮動小数点入力 | `float value = 0.0f; ImGui::InputFloat("Value", &value);` |
| `SliderFloat()` | スライダー | `float val = 0.5f; ImGui::SliderFloat("Opacity", &val, 0.0f, 1.0f);` |
| `DragInt()` | ドラッグ式数値入力 | `int count = 10; ImGui::DragInt("Count", &count, 1, 0, 100);` |
| `ColorEdit3()` | RGB色選択 | `float color[3] = {1.0f, 0.0f, 0.0f}; ImGui::ColorEdit3("Color", color);` |

## 🟩 ウィンドウの開始・終了

```cpp
ImGui::Begin("Window title");
static std::string text;
if (ImGui::Button("View text")) {
  text += "Hello!\n";
}
ImGui::Text(text.c_str());
ImGui::End();
```

## 🟨 レイアウト・構造

| ウィジェット | 説明 | 使用例 |
|--------------|------|--------|
| `SameLine()` | 同じ行に配置 | `ImGui::Text("Label"); ImGui::SameLine(); ImGui::Button("Button");` |
| `Separator()` | 区切り線 | `ImGui::Separator();` |
| `Spacing()` | 空白スペース | `ImGui::Spacing();` |
| `Indent()` / `Unindent()` | インデント調整 | `ImGui::Indent(); ImGui::Text("Indented"); ImGui::Unindent();` |
| `BeginGroup()` / `EndGroup()` | グループ化 | `ImGui::BeginGroup(); /* 複数ウィジェット */ ImGui::EndGroup();` |

## 🟥 コンボ・リスト・ツリー

| ウィジェット | 説明 | 使用例 |
|--------------|------|--------|
| `Combo()` | ドロップダウン選択 | `const char* items[] = {"A", "B", "C"}; int current = 0; ImGui::Combo("Select", &current, items, IM_ARRAYSIZE(items));` |
| `ListBox()` | リストボックス | `ImGui::ListBox("Options", &current, items, IM_ARRAYSIZE(items));` |
| `TreeNode()` | ツリーノード | `if (ImGui::TreeNode("Details")) { ImGui::Text("More info"); ImGui::TreePop(); }` |
| `CollapsingHeader()` | 折りたたみセクション | `if (ImGui::CollapsingHeader("Advanced Settings")) { /* 内容 */ }` |

## 🟪 テーブル・グリッド

| ウィジェット | 説明 | 使用例 |
|--------------|------|--------|
| `BeginTable()` | テーブル開始 | `if (ImGui::BeginTable("MyTable", 2)) { ImGui::TableNextRow(); ImGui::TableNextColumn(); ImGui::Text("Cell 1"); ImGui::TableNextColumn(); ImGui::Text("Cell 2"); ImGui::EndTable(); }` |
| `TableSetupColumn()` | 列の設定 | `ImGui::TableSetupColumn("Name"); ImGui::TableSetupColumn("Value");` |

## 🧪 その他便利ウィジェット

| ウィジェット | 説明 | 使用例 |
|--------------|------|--------|
| `ProgressBar()` | 進捗バー | `float progress = 0.7f; ImGui::ProgressBar(progress);` |
| `Image()` | テクスチャ表示 | `ImGui::Image((void*)myTextureID, ImVec2(64, 64));` |
| `BeginChild()` | 子ウィンドウ | `ImGui::BeginChild("Child", ImVec2(200, 100)); ImGui::Text("Inside child"); ImGui::EndChild();` |
| `OpenPopup()` / `BeginPopup()` | ポップアップ表示 | `ImGui::OpenPopup("MyPopup"); if (ImGui::BeginPopup("MyPopup")) { ImGui::Text("Popup content"); ImGui::EndPopup(); }` |
| `SetNextWindowSize()` | ウィンドウサイズ指定 | `ImGui::SetNextWindowSize(ImVec2(400, 300), ImGuiCond_Once);` |

## 📌 インタラクション・状態取得

| ウィジェット | 説明 | 使用例 |
|--------------|------|--------|
| `IsItemHovered()` | ホバー判定 | `if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Hovered!"); }` |
| `IsItemClicked()` | クリック判定 | `if (ImGui::IsItemClicked()) { /* クリック時の処理 */ }` |
| `GetMousePos()` | マウス座標取得 | `ImVec2 pos = ImGui::GetMousePos();` |
| `GetIO()` | 入力状態取得 | `ImGuiIO& io = ImGui::GetIO(); if (io.MouseDown[0]) { /* 左クリック中 */ }` |

---

# 📊 ImPlot 使用例チートシート

## 🟦 基本プロット関数

| 関数 | 説明 | 使用例 |
|------|------|--------|
| `ImPlot::PlotLine()` | 折れ線グラフ | `ImPlot::PlotLine("y = sin(x)", x, y, 100);` |
| `ImPlot::PlotScatter()` | 散布図 | `ImPlot::PlotScatter("Points", x, y, 100);` |
| `ImPlot::PlotBars()` | 棒グラフ（縦） | `ImPlot::PlotBars("Bar", values, 10);` |
| `ImPlot::PlotBarsH()` | 棒グラフ（横） | `ImPlot::PlotBarsH("BarH", values, 10);` |
| `ImPlot::PlotHistogram()` | ヒストグラム | `ImPlot::PlotHistogram("Hist", data, 50);` |
| `ImPlot::PlotDigital()` | デジタル信号 | `ImPlot::PlotDigital("Signal", x, y, 100);` |

## 🟩 プロットの開始・終了

```cpp
ImGui::Begin("Plot Window");
if (ImPlot::BeginPlot("My Plot")) {
    ImPlot::PlotLine("Data", x, y, 100);
    ImPlot::EndPlot();
}
ImGui::End();
```

## 🟨 軸の設定

| 関数 | 説明 | 使用例 |
|------|------|--------|
| `ImPlot::SetupAxis()` | 軸ラベル設定 | `ImPlot::SetupAxis(ImAxis_X1, "Time (s)");` |
| `ImPlot::SetupAxisLimits()` | 軸範囲指定 | `ImPlot::SetupAxisLimits(ImAxis_Y1, -1.0, 1.0);` |
| `ImPlot::SetNextAxesToFit()` | 自動スケーリング | `ImPlot::SetNextAxesToFit();` |

## 🟥 軸の種類と識別子

| 定数 | 軸 |
|------|----|
| `ImAxis_X1` | 主X軸 |
| `ImAxis_Y1` | 主Y軸 |
| `ImAxis_X2` | 補助X軸 |
| `ImAxis_Y2` | 補助Y軸 |

## 🟪 スタイル・色設定

| 関数 | 説明 | 使用例 |
|------|------|--------|
| `ImPlot::PushStyleColor()` | 色変更 | `ImPlot::PushStyleColor(ImPlotCol_Line, IM_COL32(255,0,0,255));` |
| `ImPlot::PopStyleColor()` | 色リセット | `ImPlot::PopStyleColor();` |
| `ImPlot::PushStyleVar()` | スタイル変更 | `ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 2.0f);` |
| `ImPlot::PopStyleVar()` | スタイルリセット | `ImPlot::PopStyleVar();` |

## 🧪 補助機能

| 関数 | 説明 | 使用例 |
|------|------|--------|
| `ImPlot::ShowColormapScale()` | カラーマップスケール表示 | `ImPlot::ShowColormapScale(0.0, 1.0);` |
| `ImPlot::SetColormap()` | カラーマップ変更 | `ImPlot::SetColormap(ImPlotColormap_Jet);` |
| `ImPlot::BeginLegendPopup()` | 凡例ポップアップ | `if (ImPlot::BeginLegendPopup("Data")) { /* 内容 */ ImPlot::EndLegendPopup(); }` |

## 📌 よく使うカラーマップ

| 定数 | 説明 |
|------|------|
| `ImPlotColormap_Default` | デフォルト |
| `ImPlotColormap_Jet` | ジェット（虹色） |
| `ImPlotColormap_Hot` | ホット（赤系） |
| `ImPlotColormap_Cool` | クール（青系） |
| `ImPlotColormap_Deep` | 深海風 |
| `ImPlotColormap_Dark` | ダーク系 |
| `ImPlotColormap_Pastel` | パステル系 |

## 🎯 まとめ

- `BeginPlot()` ～ `EndPlot()` の間に描画関数を呼び出す
- 軸やスタイルは `SetupAxis` や `PushStyleVar` で柔軟に設定可能
- `SetNextAxesToFit()` を使えば自動スケーリングが有効になる

---

# ⚙️ Dependencies
- [GLFW](https://www.glfw.org/)
- [Dear ImGui](https://github.com/ocornut/imgui)
- [ImPlot](https://github.com/epezent/implot)

# 🙏 Acknowledgments
This software was developed with user-friendly yet powerful software, GLFW, Dear ImGui, and ImPlot. The author would like to express their gratitude to the OSS communities.
