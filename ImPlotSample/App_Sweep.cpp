#define _CRT_SECURE_NO_WARNINGS

// --- 警告抑制 (Visual Studio用) ---
#pragma warning(push)
#pragma warning(disable: 26812)
#pragma warning(disable: 26451)
#pragma warning(disable: 26495)
#pragma warning(disable: 6031)

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <complex>
#include <random>
#include <fstream>
#include <algorithm>
#include <cstring>
#include <sstream>
#include <thread>
#include <chrono>
#include <iomanip>

// OpenGL / GLFW / ImGui
#include <GLFW/glfw3.h>
#include "Gui.h" 

#pragma warning(pop)

// --- VISAライブラリ ---
#pragma comment(lib, "visa64.lib")
#include "visa.h"
#include "AppApps.h"

namespace App_Sweep {
    // --- 定数定義 ---
#ifndef PI
#define PI 3.14159265358979323846
#endif

// --- グローバル設定 ---
    static double g_dt = 1.0e-7;
    static int    g_size = 4096;
    static double g_target_freq_shared = 1000.0;

    // FG設定用グローバル
    static int    g_wave_type = 0;    // 0:Sine, 1:Square, 2:Triangle, 3:Sawtooth
    static double g_gen_amp = 1.0;    // Vpp
    static double g_gen_phase = 0.0;  // Degree

    // スイープ設定用グローバル
    static double g_sweep_start_freq = 1.0;
    static double g_sweep_stop_freq = 100000000.0;
    static int    g_sweep_points_dec = 10; // Points per Decade

    // リアルタイムモニター用グローバル
    static std::vector<double> g_monitor_t;
    static std::vector<double> g_monitor_v1;
    static std::vector<double> g_monitor_v2;

    // VISA関連グローバル
    static bool g_use_visa = false;
    static ViSession g_rm = VI_NULL;
    static ViSession g_awg = VI_NULL;
    static ViSession g_scope = VI_NULL;

    // デバイス選択
    static std::vector<std::string> g_visa_resources;
    static int g_selected_awg_idx = 0;
    static int g_selected_scope_idx = 0;

    const char* CSV_FILENAME = "waveform_data.csv";
    const char* BMP_FILENAME = "waveform_capture.bmp";
    const char* SWEEP_FILENAME = "bode_sweep_result.csv";

    // --- キャプチャ用グローバル変数 ---
    static bool   g_capture_request = false;
    static ImVec2 g_capture_pos;
    static ImVec2 g_capture_size;

    // --- 測定結果構造体 ---
    struct MeasureResult {
        double freq;
        double gain_db;
        double phase_deg;
    };
    static std::vector<MeasureResult> g_sweep_results;

    // --- プロトタイプ宣言 ---
    void ShowConnectionWindow(const char* title);
    void ShowManualGeneratorWindow(const char* title);
    void ShowSweepControlWindow(const char* title);
    void ShowViewerWindow(const char* title);
    void SaveSnapshotBMP(const char* filename, int x, int y, int w, int h);
    void RefreshVisaResources();
    void ConnectVISA();
    void DisconnectVISA();
    bool FetchWaveformFromScope(int channel, std::vector<double>& out_t, std::vector<double>& out_v, double& out_dt);

    // =========================================================
    // ユーティリティ関数
    // =========================================================

    void SaveSnapshotBMP(const char* filename, int x, int y, int w, int h) {
        long long w_ll = static_cast<long long>(w);
        long long h_ll = static_cast<long long>(h);
        long long row_stride_ll = (w_ll * 3 + 3) & ~3;
        int row_stride = static_cast<int>(row_stride_ll);
        size_t data_size = static_cast<size_t>(row_stride_ll * h_ll);
        size_t file_size = 54 + data_size;

        std::vector<unsigned char> pixels(static_cast<size_t>(w_ll * h_ll * 3));
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(x, y, w, h, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

        std::ofstream ofs(filename, std::ios::binary);
        if (!ofs) return;

        unsigned char file_header[14] = { 'B', 'M', 0,0,0,0, 0,0,0,0, 54,0,0,0 };
        file_header[2] = (unsigned char)(file_size);
        file_header[3] = (unsigned char)(file_size >> 8);
        file_header[4] = (unsigned char)(file_size >> 16);
        file_header[5] = (unsigned char)(file_size >> 24);

        unsigned char info_header[40] = { 40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0 };
        info_header[4] = (unsigned char)(w); info_header[5] = (unsigned char)(w >> 8);
        info_header[6] = (unsigned char)(w >> 16); info_header[7] = (unsigned char)(w >> 24);
        info_header[8] = (unsigned char)(h); info_header[9] = (unsigned char)(h >> 8);
        info_header[10] = (unsigned char)(h >> 16); info_header[11] = (unsigned char)(h >> 24);

        ofs.write((char*)file_header, 14);
        ofs.write((char*)info_header, 40);

        for (size_t i = 0; i < static_cast<size_t>(w_ll * h_ll); ++i) {
            std::swap(pixels[i * 3], pixels[i * 3 + 2]);
        }

        std::vector<unsigned char> pad(3, 0);
        for (int j = 0; j < h; ++j) {
            size_t src_idx = static_cast<size_t>(j) * static_cast<size_t>(w) * 3;
            ofs.write((char*)&pixels[src_idx], static_cast<std::streamsize>(w) * 3);
            long long pad_size = row_stride_ll - (w_ll * 3);
            if (pad_size > 0) ofs.write((char*)pad.data(), pad_size);
        }
    }

    // PSD: 位相敏感検波（相対位相計算版）
    // 絶対時刻tを使わず、Ch1の位相を基準としてCh2の位相差を直接計算します。
    // これによりトリガ位置ズレによる位相の暴れを防ぎます。
    void psd_relative(
        const std::vector<double>& v1,
        const std::vector<double>& v2,
        double freq, double dt,
        double* out_gain_db, double* out_phase_deg)
    {
        if (v1.empty() || v2.empty() || freq <= 0.0 || dt <= 0.0) return;

        int n = (int)v1.size();
        if ((int)v2.size() < n) n = (int)v2.size();

        // 整数周期切り出し
        double samples_per_cycle = 1.0 / (freq * dt);
        int num_cycles = (int)(n / samples_per_cycle);
        if (num_cycles < 1) num_cycles = 1; // 最低1周期分は無理やり計算
        int n_use = (int)(num_cycles * samples_per_cycle);
        if (n_use > n) n_use = n;

        // 複素相関 (内積) を計算
        // Ref: exp(-j*omega*t)
        double sum_re1 = 0.0, sum_im1 = 0.0;
        double sum_re2 = 0.0, sum_im2 = 0.0;

        for (int i = 0; i < n_use; i++) {
            double theta = 2.0 * PI * freq * dt * i;
            double ref_re = cos(theta);
            double ref_im = -sin(theta); // 複素共役

            // Ch1 (Input)
            sum_re1 += v1[i] * ref_re;
            sum_im1 += v1[i] * ref_im;

            // Ch2 (Output)
            sum_re2 += v2[i] * ref_re;
            sum_im2 += v2[i] * ref_im;
        }

        // 複素ベクトル化
        std::complex<double> c1(sum_re1, sum_im1);
        std::complex<double> c2(sum_re2, sum_im2);

        // 振幅
        double amp1 = std::abs(c1);
        double amp2 = std::abs(c2);

        if (amp1 < 1e-12) amp1 = 1e-12; // ゼロ除算防止

        // Gain (dB)
        *out_gain_db = 20.0 * log10(amp2 / amp1);

        // 相対位相 (Ch2 phase - Ch1 phase)
        // std::arg(c2 / c1) と同義
        double phase_rad = std::arg(c2) - std::arg(c1);

        // Degree変換
        double phase_deg = phase_rad * 180.0 / PI;

        // 位相ラップ処理 (-180 ~ +180)
        while (phase_deg > 180.0) phase_deg -= 360.0;
        while (phase_deg < -180.0) phase_deg += 360.0;

        *out_phase_deg = phase_deg;
    }

    // =========================================================
    // VISA 接続・検索・切断関数
    // =========================================================

    void RefreshVisaResources() {
        g_visa_resources.clear();
        if (g_rm == VI_NULL) if (viOpenDefaultRM(&g_rm) < VI_SUCCESS) return;
        ViFindList findList; ViUInt32 retCount; char desc[VI_FIND_BUFLEN];
        if (viFindRsrc(g_rm, (ViString)"?*INSTR", &findList, &retCount, desc) >= VI_SUCCESS) {
            g_visa_resources.push_back(desc);
            for (ViUInt32 i = 1; i < retCount; ++i) if (viFindNext(findList, desc) >= VI_SUCCESS) g_visa_resources.push_back(desc);
            viClose(findList);
        }
    }

    void ConnectVISA() {
        if (g_visa_resources.empty()) RefreshVisaResources();
        if (g_rm == VI_NULL) if (viOpenDefaultRM(&g_rm) < VI_SUCCESS) return;
        if (g_visa_resources.empty()) return;
        if (g_selected_awg_idx < 0 || g_selected_awg_idx >= (int)g_visa_resources.size()) g_selected_awg_idx = 0;
        if (g_selected_scope_idx < 0 || g_selected_scope_idx >= (int)g_visa_resources.size()) g_selected_scope_idx = 0;

        if (g_awg == VI_NULL) {
            viOpen(g_rm, (ViRsrc)g_visa_resources[g_selected_awg_idx].c_str(), VI_NULL, VI_NULL, &g_awg);
            if (g_awg) viSetAttribute(g_awg, VI_ATTR_TMO_VALUE, 10000);
        }
        if (g_scope == VI_NULL) {
            viOpen(g_rm, (ViRsrc)g_visa_resources[g_selected_scope_idx].c_str(), VI_NULL, VI_NULL, &g_scope);
            if (g_scope) {
                viSetAttribute(g_scope, VI_ATTR_TMO_VALUE, 10000);
                viSetAttribute(g_scope, VI_ATTR_TERMCHAR, 0x0A);
                viSetAttribute(g_scope, VI_ATTR_TERMCHAR_EN, VI_TRUE);
                viClear(g_scope);
            }
        }
    }

    void DisconnectVISA() {
        if (g_awg) { viClose(g_awg); g_awg = VI_NULL; }
        if (g_scope) { viClose(g_scope); g_scope = VI_NULL; }
        if (g_rm) { viClose(g_rm); g_rm = VI_NULL; }
    }

    // チャンネルを指定して波形取得 (Ch1 or Ch2)
    bool FetchWaveformFromScope(int channel, std::vector<double>& out_t, std::vector<double>& out_v, double& out_dt) {
        if (!g_scope) return false;

        // 波形取得設定
        viPrintf(g_scope, ":COMMunicate:HEADer OFF\n");
        viPrintf(g_scope, ":WAVeform:TRACe %d\n", channel); // チャンネル指定
        viPrintf(g_scope, ":WAVeform:FORMat ASCii\n");
        viPrintf(g_scope, ":WAVeform:STARt 1\n");
        viPrintf(g_scope, ":WAVeform:END 10000\n");

        // Timebase取得
        char tdiv_buf[256] = { 0 };
        ViUInt32 rc = 0;
        viPrintf(g_scope, ":TIMebase:TDIV?\n");
        viRead(g_scope, (ViPBuf)tdiv_buf, 255, &rc);

        // 文字列の掃除（改行削除）
        std::string s_tdiv = tdiv_buf;
        s_tdiv.erase(std::remove(s_tdiv.begin(), s_tdiv.end(), '\n'), s_tdiv.end());
        s_tdiv.erase(std::remove(s_tdiv.begin(), s_tdiv.end(), '\r'), s_tdiv.end());

        double tdiv = 1.0e-3;
        try {
            if (!s_tdiv.empty()) tdiv = std::stod(s_tdiv);
        }
        catch (...) {}

        // データ転送要求
        viPrintf(g_scope, ":WAVeform:SEND?\n");

        std::string rawData = "";
        static char buf[4096];
        bool keepReading = true;
        int maxLoops = 2000;
        while (keepReading && maxLoops > 0) {
            memset(buf, 0, sizeof(buf));
            ViUInt32 retCount = 0;
            ViStatus status = viRead(g_scope, (ViPBuf)buf, sizeof(buf) - 1, &retCount);
            if (retCount > 0) rawData += buf;
            if (status == VI_SUCCESS || status < VI_SUCCESS) keepReading = false;
            maxLoops--;
        }

        std::vector<double> temp_v;
        if (!rawData.empty()) {
            size_t dataStart = 0;
            size_t hashPos = rawData.find('#');
            if (hashPos != std::string::npos && hashPos + 1 < rawData.size()) {
                int digits = rawData[hashPos + 1] - '0';
                if (digits > 0) dataStart = hashPos + 2 + digits;
            }
            std::stringstream ss(rawData.substr(dataStart));
            std::string seg;
            while (std::getline(ss, seg, ',')) {
                try { temp_v.push_back(std::stod(seg)); }
                catch (...) {}
            }
        }

        if (temp_v.empty()) return false;

        int n = (int)temp_v.size();
        double total_time = tdiv * 10.0; // 簡易計算 (10div)
        out_dt = total_time / n;
        out_t.resize(n);
        out_v = temp_v;
        for (int i = 0; i < n; ++i) out_t[i] = i * out_dt;

        return true;
    }

    // =========================================================
    // メイン関数
    // =========================================================
    int Run() {
        Gui::Initialize("Bode Plot (Relative Phase Calc)", 0, 30, 1280, 850);
        if (Gui::GetWindow() == nullptr) return -1;

        g_target_freq_shared = 1000.0;

        while (!glfwWindowShouldClose(Gui::GetWindow())) {
            Gui::BeginFrame();

            ShowConnectionWindow("1. Connection");
            ShowManualGeneratorWindow("2. Manual Generator");
            ShowSweepControlWindow("3. Bode Sweep Control");
            ShowViewerWindow("4. Realtime Monitor");

            Gui::EndFrame();

            if (g_capture_request) {
                int win_w, win_h, fb_w, fb_h;
                glfwGetWindowSize(Gui::GetWindow(), &win_w, &win_h);
                glfwGetFramebufferSize(Gui::GetWindow(), &fb_w, &fb_h);
                float scale_x = (float)fb_w / win_w;
                float scale_y = (float)fb_h / win_h;
                int ix = (int)(g_capture_pos.x * scale_x);
                int iw = (int)(g_capture_size.x * scale_x);
                int ih = (int)(g_capture_size.y * scale_y);
                int iy = (int)(fb_h - (g_capture_pos.y * scale_y + ih));
                if (iw > 0 && ih > 0) {
                    glReadBuffer(GL_FRONT);
                    SaveSnapshotBMP(BMP_FILENAME, ix < 0 ? 0 : ix, iy < 0 ? 0 : iy, iw, ih);
                }
                g_capture_request = false;
            }
        }

        DisconnectVISA();
        Gui::Shutdown();
        return 0;
    }

    // =========================================================
    // Window 0: 接続管理
    // =========================================================
    void ShowConnectionWindow(const char* title) {
        ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(400 * Gui::monitorScale, 180 * Gui::monitorScale), ImGuiCond_FirstUseEver);

        if (ImGui::Begin(title)) {
            if (ImGui::Button("Search Devices")) RefreshVisaResources();
            ImGui::SameLine(); ImGui::Text("Found: %d", (int)g_visa_resources.size());

            auto vector_getter = [](void* vec, int idx, const char** out_text) {
                auto& vector = *static_cast<std::vector<std::string>*>(vec);
                if (idx < 0 || idx >= static_cast<int>(vector.size())) return false;
                *out_text = vector.at(idx).c_str(); return true;
            };

            if (!g_visa_resources.empty()) {
                ImGui::Combo("FG Address", &g_selected_awg_idx, vector_getter, &g_visa_resources, (int)g_visa_resources.size());
                ImGui::Combo("Scope Address", &g_selected_scope_idx, vector_getter, &g_visa_resources, (int)g_visa_resources.size());
            }

            ImGui::Separator();
            if (ImGui::Checkbox("Use VISA (Connect)", &g_use_visa)) {
                if (g_use_visa) ConnectVISA(); else DisconnectVISA();
            }

            if (g_use_visa) {
                ImGui::Text("AWG: %s", g_awg ? "Connected" : "Not Found");
                ImGui::Text("Scope: %s", g_scope ? "Connected" : "Not Found");
            }
        }
        ImGui::End();
    }

    // =========================================================
    // Window 2: マニュアルジェネレータ
    // =========================================================
    void ShowManualGeneratorWindow(const char* title) {
        ImGui::SetNextWindowPos(ImVec2(0, 190 * Gui::monitorScale), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(400 * Gui::monitorScale, 200 * Gui::monitorScale), ImGuiCond_FirstUseEver);

        static std::string status = "Idle";

        if (ImGui::Begin(title)) {
            ImGui::Text("Manual Signal Output");
            ImGui::Combo("Wave Type", &g_wave_type, "Sine\0Square\0Triangle\0");
            ImGui::InputDouble("Freq (Hz)", &g_target_freq_shared, 10.0, 100000.0, "%.1f");
            ImGui::InputDouble("Amp (Vpp)", &g_gen_amp, 0.1, 10.0, "%.2f");

            if (ImGui::Button("Set Output")) {
                if (g_use_visa && g_awg) {
                    if (g_wave_type == 0) viPrintf(g_awg, ":SOURce1:FUNCtion SIN\n");
                    else if (g_wave_type == 1) viPrintf(g_awg, ":SOURce1:FUNCtion SQU\n");
                    else if (g_wave_type == 2) viPrintf(g_awg, ":SOURce1:FUNCtion RAMP\n");

                    viPrintf(g_awg, ":SOURce1:VOLTage %e\n", g_gen_amp);
                    viPrintf(g_awg, ":SOURce1:FREQuency %e\n", g_target_freq_shared);
                    viPrintf(g_awg, ":OUTPut1:STATe ON\n");

                    if (g_scope) viPrintf(g_scope, ":ASETup:EXECute\n");
                    status = "Output ON (Auto Setup)";
                }
                else {
                    status = "Sim Mode (No VISA)";
                }
            }
            ImGui::SameLine();
            ImGui::Text("Status: %s", status.c_str());
        }
        ImGui::End();
    }

    // =========================================================
    // Window 3: Bode Sweep Control (Improved PSD)
    // =========================================================
    void ShowSweepControlWindow(const char* title) {
        ImGui::SetNextWindowPos(ImVec2(410 * Gui::monitorScale, 0), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(800 * Gui::monitorScale, 600 * Gui::monitorScale), ImGuiCond_FirstUseEver);

        static bool sweep_running = false;
        static std::vector<double> sweep_freqs;
        static int sweep_idx = 0;
        static auto last_time = std::chrono::steady_clock::now();
        static std::string status = "Ready";

        auto UpdateInstrumentSettings = [](double freq, double amp_vpp) {
            if (!g_awg || !g_scope) return;
            viPrintf(g_awg, ":SOURce1:FREQuency %e\n", freq);
            viPrintf(g_scope, ":ASETup:EXECute\n");
        };

        if (ImGui::Begin(title)) {
            // --- 設定エリア ---
            ImGui::TextColored(ImVec4(0, 1, 1, 1), "Sweep Settings");
            ImGui::PushItemWidth(120);
            ImGui::InputDouble("Start Freq (Hz)", &g_sweep_start_freq, 1.0, 100.0, "%.1f"); ImGui::SameLine();
            ImGui::InputDouble("Stop Freq (Hz)", &g_sweep_stop_freq, 100.0, 100000000.0, "%.1f");
            ImGui::InputInt("Points/Decade", &g_sweep_points_dec, 1, 5); ImGui::SameLine();
            ImGui::InputDouble("Amp (Vpp)", &g_gen_amp, 0.1, 1.0, "%.2f");
            ImGui::PopItemWidth();

            ImGui::Separator();

            // --- 操作ボタン ---
            if (!sweep_running) {
                if (ImGui::Button("START SWEEP", ImVec2(120, 30))) {
                    if (!g_use_visa || !g_awg || !g_scope) {
                        status = "Error: VISA not ready";
                    }
                    else {
                        sweep_freqs.clear();
                        g_sweep_results.clear();
                        if (g_sweep_points_dec < 1) g_sweep_points_dec = 1;

                        double f = g_sweep_start_freq;
                        while (f <= g_sweep_stop_freq * 1.0001) {
                            sweep_freqs.push_back(f);
                            f *= std::pow(10.0, 1.0 / (double)g_sweep_points_dec);
                        }

                        sweep_idx = 0;
                        sweep_running = true;
                        viPrintf(g_awg, ":SOURce1:FUNCtion SIN\n");
                        viPrintf(g_awg, ":SOURce1:VOLTage %e\n", g_gen_amp);
                        viPrintf(g_awg, ":OUTPut1:STATe ON\n");

                        g_target_freq_shared = sweep_freqs[0];
                        UpdateInstrumentSettings(g_target_freq_shared, g_gen_amp);
                        last_time = std::chrono::steady_clock::now();
                        status = "Running (Auto Setup)...";
                    }
                }
            }
            else {
                if (ImGui::Button("STOP SWEEP", ImVec2(120, 30))) {
                    sweep_running = false;
                    status = "Aborted";
                }
                ImGui::SameLine();
                ImGui::ProgressBar((float)sweep_idx / (float)sweep_freqs.size(), ImVec2(200, 30),
                    (std::to_string(sweep_idx) + "/" + std::to_string(sweep_freqs.size())).c_str());
            }
            ImGui::SameLine();
            ImGui::Text("  Status: %s", status.c_str());

            // --- スイープ処理 ---
            if (sweep_running) {
                auto now = std::chrono::steady_clock::now();
                double elapsed = std::chrono::duration<double>(now - last_time).count();

                if (elapsed >= 4.0) {
                    viPrintf(g_scope, ":CHANnel1:DISPlay ON\n");
                    viPrintf(g_scope, ":CHANnel2:DISPlay ON\n");

                    viPrintf(g_scope, ":STOP\n");
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));

                    std::vector<double> t1, v1, t2, v2;
                    double dt1 = 0, dt2 = 0;
                    bool ok1 = FetchWaveformFromScope(1, t1, v1, dt1);
                    bool ok2 = FetchWaveformFromScope(2, t2, v2, dt2);

                    if (ok1 && ok2) {
                        g_monitor_t = t1;
                        g_monitor_v1 = v1;
                        g_monitor_v2 = v2;

                        double gain = 0.0, phase = 0.0;
                        // 新しいPSD関数を使用 (Ch1基準の相対位相)
                        psd_relative(v1, v2, g_target_freq_shared, dt1, &gain, &phase);

                        g_sweep_results.push_back({ g_target_freq_shared, gain, phase });
                        status = "Meas: " + std::to_string((int)g_target_freq_shared) + " Hz";
                    }
                    else {
                        status = "Capture Failed";
                    }
                    viPrintf(g_scope, ":STARt\n");

                    sweep_idx++;
                    if (sweep_idx >= (int)sweep_freqs.size()) {
                        sweep_running = false;
                        status = "Completed";
                        std::ofstream ofs(SWEEP_FILENAME);
                        ofs << "freq,gain,deg\n";
                        for (auto& r : g_sweep_results) ofs << r.freq << "," << r.gain_db << "," << r.phase_deg << "\n";
                    }
                    else {
                        g_target_freq_shared = sweep_freqs[sweep_idx];
                        UpdateInstrumentSettings(g_target_freq_shared, g_gen_amp);
                        last_time = std::chrono::steady_clock::now();
                    }
                }
            }

            ImGui::Separator();

            std::vector<double> x, y_g, y_p;
            if (!g_sweep_results.empty()) {
                for (auto& r : g_sweep_results) {
                    x.push_back(r.freq);
                    y_g.push_back(r.gain_db);
                    y_p.push_back(r.phase_deg);
                }
            }

            if (ImPlot::BeginPlot("Gain Plot", ImVec2(-1, 220))) {
                ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
                ImPlot::SetupAxisLimits(ImAxis_X1, 1.0, 100000000.0, ImGuiCond_Once);
                ImPlot::SetupAxes("Freq (Hz)", "Gain (dB)");
                if (!x.empty()) {
                    ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 4);
                    ImPlot::PlotLine("Gain", x.data(), y_g.data(), (int)x.size());
                }
                ImPlot::EndPlot();
            }

            if (ImPlot::BeginPlot("Phase Plot", ImVec2(-1, 220))) {
                ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
                ImPlot::SetupAxisLimits(ImAxis_X1, 1.0, 100000000.0, ImGuiCond_Once);
                ImPlot::SetupAxes("Freq (Hz)", "Phase (deg)");
                if (!x.empty()) {
                    ImPlot::SetNextMarkerStyle(ImPlotMarker_Square, 4);
                    ImPlot::SetNextLineStyle(ImVec4(1, 0.5f, 0, 1));
                    ImPlot::PlotLine("Phase", x.data(), y_p.data(), (int)x.size());
                }
                ImPlot::EndPlot();
            }
        }
        ImGui::End();
    }

    // =========================================================
    // Window 4: Realtime Monitor (Passive Update)
    // =========================================================
    void ShowViewerWindow(const char* title) {
        ImGui::SetNextWindowPos(ImVec2(0, 400 * Gui::monitorScale), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(400 * Gui::monitorScale, 250 * Gui::monitorScale), ImGuiCond_FirstUseEver);

        if (ImGui::Begin(title)) {
            ImGui::Text("Monitoring Last Capture (Ch1:Blue, Ch2:Orange)");

            if (ImPlot::BeginPlot("Live Waveform", ImVec2(-1, -1))) {
                ImPlot::SetupAxis(ImAxis_X1, "Time (s)", ImPlotAxisFlags_AutoFit);
                ImPlot::SetupAxis(ImAxis_Y1, "Voltage (V)", ImPlotAxisFlags_AutoFit);

                if (!g_monitor_t.empty()) {
                    ImPlot::SetNextLineStyle(ImVec4(0.4f, 0.7f, 1.0f, 1.0f));
                    ImPlot::PlotLine("Ch1", g_monitor_t.data(), g_monitor_v1.data(), (int)g_monitor_t.size());

                    if (!g_monitor_v2.empty()) {
                        ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.6f, 0.2f, 1.0f));
                        ImPlot::PlotLine("Ch2", g_monitor_t.data(), g_monitor_v2.data(), (int)g_monitor_t.size());
                    }
                }
                ImPlot::EndPlot();
            }
        }
        ImGui::End();
    }
}