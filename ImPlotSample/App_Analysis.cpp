#define _CRT_SECURE_NO_WARNINGS

// --- 警告抑制 (Visual Studio用) ---
#pragma warning(push)
#pragma warning(disable: 26812) // enum class 推奨警告
#pragma warning(disable: 26451) // 演算オーバーフロー警告(ライブラリ内)
#pragma warning(disable: 26495) // 変数の初期化警告
#pragma warning(disable: 6031)  // 戻り値無視の警告

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

// OpenGL / GLFW / ImGui
#include <GLFW/glfw3.h>
#include "Gui.h" 

#pragma warning(pop)

// --- VISAライブラリ ---
#pragma comment(lib, "visa64.lib")
#include "visa.h"
#include "AppApps.h"

namespace App_Analysis {
    // --- 定数定義 ---
#ifndef PI
#define PI 3.14159265358979323846
#endif

// --- グローバル設定 ---
    static double g_dt = 1.0e-7;
    static int    g_size = 4096;
    static double g_target_freq_shared = 1000.0;

    // FG設定用グローバル (Input波形保存のために共有)
    static int    g_wave_type = 0;    // 0:Sine, 1:Square, 2:Triangle, 3:Sawtooth
    static double g_gen_amp = 1.0;    // Vpp
    static double g_gen_phase = 0.0;  // Degree

    // VISA関連グローバル
    static bool g_use_visa = false;
    static ViSession g_rm = VI_NULL;
    static ViSession g_awg = VI_NULL;
    static ViSession g_scope = VI_NULL;

    // デバイス選択
    static std::vector<std::string> g_visa_resources;
    static int g_selected_awg_idx = 0;
    static int g_selected_scope_idx = 0;

    // フィルタ設定 (OFFを追加)
    enum FilterType { FILT_OFF, FILT_LPF, FILT_HPF, FILT_BPF };
    static int g_filter_type = FILT_OFF; // デフォルトをOFFに変更（任意）
    static double g_cutoff_freq = 100000.0;
    static double g_q_val = 0.707;

    const char* CSV_FILENAME = "waveform_data.csv";
    const char* BMP_FILENAME = "waveform_capture.bmp";

    // --- キャプチャ用グローバル変数 ---
    static bool   g_capture_request = false;
    static ImVec2 g_capture_pos;
    static ImVec2 g_capture_size;

    // --- プロトタイプ宣言 ---
    void ShowConnectionWindow(const char* title);
    void ShowGeneratorWindow(const char* title);
    void ShowBodePlotWindow(const char* title);
    void ShowViewerWindow(const char* title);
    void SaveSnapshotBMP(const char* filename, int x, int y, int w, int h);
    void RefreshVisaResources();
    void ConnectVISA();
    void DisconnectVISA();

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
            if (pad_size > 0) {
                ofs.write((char*)pad.data(), pad_size);
            }
        }
    }

    struct WavHeader {
        char riff[4]; unsigned int overall_size; char wave[4];
        char fmt_chunk_marker[4]; unsigned int length_of_fmt; unsigned short format_type;
        unsigned short channels; unsigned int sample_rate; unsigned int byterate;
        unsigned short block_align; unsigned short bits_per_sample;
    };

    bool LoadWavFile(const std::string& filename, std::vector<double>& t_out, std::vector<double>& v_out) {
        std::ifstream ifs(filename, std::ios::binary);
        if (!ifs.is_open()) return false;
        WavHeader header;
        ifs.read((char*)&header, sizeof(WavHeader));
        if (std::strncmp(header.riff, "RIFF", 4) != 0 || std::strncmp(header.wave, "WAVE", 4) != 0) return false;
        char chunk_id[4]; unsigned int chunk_size = 0;
        while (ifs.read(chunk_id, 4) && ifs.read((char*)&chunk_size, 4)) {
            if (std::strncmp(chunk_id, "data", 4) == 0) break;
            ifs.seekg(chunk_size, std::ios::cur);
        }

        size_t bytes_per_sample = header.bits_per_sample / 8;
        size_t denom = static_cast<size_t>(header.channels) * bytes_per_sample;
        if (denom == 0) return false;
        size_t num_samples = (size_t)chunk_size / denom;

        if (num_samples > 1000000) num_samples = 1000000;

        std::vector<double> temp_v(num_samples);
        std::vector<char> buffer(chunk_size);
        ifs.read(buffer.data(), chunk_size);

        for (size_t i = 0; i < num_samples; ++i) {
            double val = 0.0;
            size_t index = i * denom;
            if (header.bits_per_sample == 8) {
                val = ((unsigned char)buffer[index] - 128) / 128.0;
            }
            else if (header.bits_per_sample == 16) {
                if (index + 2 <= buffer.size()) {
                    short s; memcpy(&s, &buffer[index], 2); val = s / 32768.0;
                }
            }
            temp_v[i] = val;
        }
        g_dt = 1.0 / header.sample_rate;
        g_size = (int)num_samples;
        t_out.resize(g_size);
        v_out = temp_v;
        for (int i = 0; i < g_size; ++i) t_out[i] = i * g_dt;
        return true;
    }

    class FilterStage {
    public:
        double b0, b1, b2, a1, a2;
        double x1, x2, y1, y2;
        FilterStage(int type, double freq, double sampleRate, double q) {
            setup(type, freq, sampleRate, q);
            reset();
        }
        void setup(int type, double freq, double sampleRate, double q) {
            double nyquist = sampleRate / 2.0;
            double safe_freq = (freq < nyquist * 0.95) ? freq : nyquist * 0.95;
            double omega = 2.0 * PI * safe_freq / sampleRate;
            double sn = sin(omega);
            double cs = cos(omega);
            double alpha = sn / (2.0 * q);
            if (type == FILT_LPF) {
                double k = tan(PI * safe_freq / sampleRate);
                double norm = 1.0 / (k + 1.0);
                b0 = k * norm; b1 = k * norm; b2 = 0.0;
                a1 = (k - 1.0) * norm; a2 = 0.0;
            }
            else if (type == FILT_HPF) {
                double k = tan(PI * safe_freq / sampleRate);
                double norm = 1.0 / (k + 1.0);
                b0 = 1.0 * norm; b1 = -1.0 * norm; b2 = 0.0;
                a1 = (k - 1.0) * norm; a2 = 0.0;
            }
            else if (type == FILT_BPF) {
                double norm = 1.0 / (1.0 + alpha);
                b0 = alpha * norm; b1 = 0.0; b2 = -alpha * norm;
                a1 = -2.0 * cs * norm; a2 = (1.0 - alpha) * norm;
            }
            else {
                // FILT_OFF or unknown: pass through
                b0 = 1.0; b1 = 0.0; b2 = 0.0;
                a1 = 0.0; a2 = 0.0;
            }
        }
        void reset() { x1 = x2 = y1 = y2 = 0.0; }
        double process(double in) {
            double out = b0 * in + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
            x2 = x1; x1 = in; y2 = y1; y1 = out;
            return out;
        }
        std::complex<double> response(double f, double dt) {
            double omega_d = 2.0 * PI * f * dt;
            std::complex<double> z1 = std::polar(1.0, -omega_d);
            std::complex<double> z2 = std::polar(1.0, -2.0 * omega_d);
            std::complex<double> num = b0 + b1 * z1 + b2 * z2;
            std::complex<double> den = 1.0 + a1 * z1 + a2 * z2;
            return num / den;
        }
    };

    void _fft(std::vector<std::complex<double>>& a) {
        int n = (int)a.size();
        if (n <= 1) return;
        std::vector<std::complex<double>> even(n / 2), odd(n / 2);
        for (int i = 0; i < n / 2; ++i) { even[i] = a[i * 2]; odd[i] = a[i * 2 + 1]; }
        _fft(even); _fft(odd);
        for (int k = 0; k < n / 2; ++k) {
            std::complex<double> t = std::polar(1.0, -2 * PI * k / n) * odd[k];
            a[k] = even[k] + t; a[k + n / 2] = even[k] - t;
        }
    }
    void compute_fft(const std::vector<double>& in_array, std::vector<double>& out_mag, std::vector<double>& out_freq) {
        if (in_array.empty()) return;
        int original_n = (int)in_array.size();
        int n = 1; while (n < original_n) n *= 2;
        if (n > 1048576) n = 1048576;
        std::vector<std::complex<double>> vec(n, std::complex<double>(0, 0));
        for (int i = 0; i < n && i < original_n; i++) vec[i] = in_array[i];
        _fft(vec);
        out_mag.resize(n); out_freq.resize(n);
        for (int i = 0; i < n; i++) {
            out_freq[i] = i / (g_dt * n);
            out_mag[i] = std::abs(vec[i]) * 2.0 / n;
        }
        out_mag[0] /= 2.0;
    }
    void psd(const std::vector<double>& arr, double freq, double dt, double* pX, double* pY) {
        *pX = 0.0; *pY = 0.0;
        if (arr.empty()) return;
        int n = (int)arr.size();
        for (int i = 0; i < n; i++) {
            double wt = 2.0 * PI * freq * dt * i;
            *pX += arr[i] * 2.0 * sin(wt);
            *pY += arr[i] * 2.0 * cos(wt);
        }
        *pX /= n; *pY /= n;
    }

    // =========================================================
    // VISA 接続・検索・切断関数
    // =========================================================

    void RefreshVisaResources() {
        g_visa_resources.clear();
        bool local_rm = false;
        if (g_rm == VI_NULL) {
            if (viOpenDefaultRM(&g_rm) < VI_SUCCESS) return;
            local_rm = true;
        }
        ViFindList findList;
        ViUInt32 retCount;
        char desc[VI_FIND_BUFLEN];
        if (viFindRsrc(g_rm, (ViString)"?*INSTR", &findList, &retCount, desc) >= VI_SUCCESS) {
            g_visa_resources.push_back(desc);
            for (ViUInt32 i = 1; i < retCount; ++i) {
                if (viFindNext(findList, desc) >= VI_SUCCESS) {
                    g_visa_resources.push_back(desc);
                }
            }
            viClose(findList);
        }
    }

    void ConnectVISA() {
        if (g_visa_resources.empty()) RefreshVisaResources();
        if (g_rm == VI_NULL) {
            if (viOpenDefaultRM(&g_rm) < VI_SUCCESS) return;
        }
        if (g_visa_resources.empty()) return;

        if (g_selected_awg_idx < 0 || g_selected_awg_idx >= (int)g_visa_resources.size()) g_selected_awg_idx = 0;
        if (g_selected_scope_idx < 0 || g_selected_scope_idx >= (int)g_visa_resources.size()) g_selected_scope_idx = 0;

        if (g_awg == VI_NULL) {
            const char* rsrc = g_visa_resources[g_selected_awg_idx].c_str();
            viOpen(g_rm, (ViRsrc)rsrc, VI_NULL, VI_NULL, &g_awg);
            if (g_awg) viSetAttribute(g_awg, VI_ATTR_TMO_VALUE, 10000);
        }
        if (g_scope == VI_NULL) {
            const char* rsrc = g_visa_resources[g_selected_scope_idx].c_str();
            viOpen(g_rm, (ViRsrc)rsrc, VI_NULL, VI_NULL, &g_scope);
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

    // =========================================================
    // メイン関数
    // =========================================================
    int Run() {
        Gui::Initialize("Waveform Analysis Tool v4.3 (Filter OFF Supported)", 0, 30, 1280, 800);
        if (Gui::GetWindow() == nullptr) return -1;

        g_dt = 1.0e-7; g_size = 4096;

        while (!glfwWindowShouldClose(Gui::GetWindow())) {
            Gui::BeginFrame();

            ShowConnectionWindow("VISA Connection Manager");
            ShowGeneratorWindow("Generate waveform");
            ShowBodePlotWindow("Bode plots");
            ShowViewerWindow("View waveform");

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

                if (ix < 0) ix = 0; if (iy < 0) iy = 0;
                if (ix + iw > fb_w) iw = fb_w - ix;
                if (iy + ih > fb_h) ih = fb_h - iy;

                if (iw > 0 && ih > 0) {
                    glReadBuffer(GL_FRONT);
                    SaveSnapshotBMP(BMP_FILENAME, ix, iy, iw, ih);
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
        ImGui::SetNextWindowSize(ImVec2(450 * Gui::monitorScale, 200 * Gui::monitorScale), ImGuiCond_FirstUseEver);

        if (ImGui::Begin(title)) {
            if (ImGui::Button("Search Devices")) {
                RefreshVisaResources();
            }
            ImGui::SameLine();
            ImGui::Text("Found: %d", (int)g_visa_resources.size());

            auto vector_getter = [](void* vec, int idx, const char** out_text) {
                auto& vector = *static_cast<std::vector<std::string>*>(vec);
                if (idx < 0 || idx >= static_cast<int>(vector.size())) { return false; }
                *out_text = vector.at(idx).c_str();
                return true;
            };

            if (!g_visa_resources.empty()) {
                ImGui::Combo("FG (AWG)", &g_selected_awg_idx, vector_getter,
                    static_cast<void*>(&g_visa_resources), (int)g_visa_resources.size());
                ImGui::Combo("Oscilloscope", &g_selected_scope_idx, vector_getter,
                    static_cast<void*>(&g_visa_resources), (int)g_visa_resources.size());
            }
            else {
                ImGui::TextColored(ImVec4(1, 1, 0, 1), "No VISA devices. Click Search.");
            }

            ImGui::Separator();

            if (ImGui::Checkbox("Connect (Use VISA)", &g_use_visa)) {
                if (g_use_visa) ConnectVISA();
                else DisconnectVISA();
            }

            if (g_use_visa && g_awg) ImGui::TextColored(ImVec4(0, 1, 0, 1), "AWG: Connected");
            else if (g_use_visa) ImGui::TextColored(ImVec4(1, 0, 0, 1), "AWG: Not Connected");

            if (g_use_visa && g_scope) ImGui::TextColored(ImVec4(0, 1, 0, 1), "Scope: Connected");
            else if (g_use_visa) ImGui::TextColored(ImVec4(1, 0, 0, 1), "Scope: Not Connected");
        }
        ImGui::End();
    }

    // =========================================================
    // Window 1: 波形生成
    // =========================================================
    void ShowGeneratorWindow(const char* title) {
        // wave_type も共有変数(g_wave_type)を使用
        static double noise = 0.0;
        static char file_path[128] = "test.wav";
        static std::string status_text = "";
        static bool output_enabled = false;

        ImGui::SetNextWindowPos(ImVec2(0, 210 * Gui::monitorScale), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(450 * Gui::monitorScale, 280 * Gui::monitorScale), ImGuiCond_FirstUseEver);

        if (ImGui::Begin(title)) {
            if (g_use_visa) {
                ImGui::TextColored(ImVec4(0, 1, 0, 1), "[VISA Mode] Devices Connected");
            }
            else {
                ImGui::TextColored(ImVec4(1, 1, 0, 1), "[Simulation Mode]");
            }
            ImGui::Separator();

            // --- VISA Output ON/OFF ---
            if (g_use_visa) {
                if (ImGui::Checkbox("FG Output ON", &output_enabled)) {
                    if (g_awg) {
                        if (output_enabled) viPrintf(g_awg, ":OUTPut1:STATe ON\n");
                        else                viPrintf(g_awg, ":OUTPut1:STATe OFF\n");
                    }
                }
                ImGui::SameLine(); ImGui::TextDisabled("(Direct Control)");
            }

            ImGui::Combo("Wave Type", &g_wave_type, "Sine\0Square\0Triangle\0Sawtooth\0File (WAV)\0");

            if (g_wave_type == 4 && !g_use_visa) {
                ImGui::InputText("Filename", file_path, sizeof(file_path));
            }
            else {
                ImGui::PushItemWidth(200 * Gui::monitorScale);
                ImGui::InputDouble("Frequency (Hz)", &g_target_freq_shared, 10.0, 100000.0, "%.1f");
                ImGui::InputDouble("Amplitude (Vpp)", &g_gen_amp, 0.1, 10.0, "%.2f");
                if (!g_use_visa) {
                    ImGui::InputDouble("Phase (Deg.)", &g_gen_phase, 1.0, 10.0, "%.2f");
                    ImGui::InputDouble("Noise (V)", &noise, 0.1, 1.0, "%.2f");
                }
                ImGui::PopItemWidth();
            }

            const char* btnLabel = g_use_visa ? "Apply to Device" : (g_wave_type == 4 ? "Load & Save" : "Generate & Save");
            if (ImGui::Button(btnLabel)) {
                std::vector<double> t_data, v_data;

                if (g_use_visa) {
                    if (g_awg) {
                        std::string typeInfo = "";
                        if (g_wave_type == 0) {
                            viPrintf(g_awg, ":SOURce1:FUNCtion SIN\n"); typeInfo = "SIN";
                        }
                        else if (g_wave_type == 1) {
                            viPrintf(g_awg, ":SOURce1:FUNCtion SQU\n"); typeInfo = "SQU";
                        }
                        else if (g_wave_type == 2) {
                            viPrintf(g_awg, ":SOURce1:FUNCtion RAMP\n");
                            viPrintf(g_awg, ":SOURce1:FUNCtion:RAMP:SYMMetry 50\n"); typeInfo = "TRI";
                        }
                        else if (g_wave_type == 3) {
                            viPrintf(g_awg, ":SOURce1:FUNCtion RAMP\n");
                            viPrintf(g_awg, ":SOURce1:FUNCtion:RAMP:SYMMetry 0\n"); typeInfo = "SAW";
                        }
                        else {
                            typeInfo = "Unsupp";
                        }

                        if (g_wave_type <= 3) {
                            viPrintf(g_awg, ":SOURce1:FREQuency %e\n", g_target_freq_shared);
                            viPrintf(g_awg, ":SOURce1:VOLTage %e\n", g_gen_amp);

                            viPrintf(g_awg, ":OUTPut1:STATe ON\n");
                            output_enabled = true;

                            if (g_scope && g_target_freq_shared > 0) {
                                double tdiv = 0.3 / g_target_freq_shared;
                                viPrintf(g_scope, ":TIMebase:TDIV %e\n", tdiv);
                                status_text = "Set: " + typeInfo + " & Scope T/Div";
                            }
                            else {
                                status_text = "Set: " + typeInfo;
                            }
                        }
                        else {
                            status_text = "Invalid Type for VISA";
                        }
                    }
                    else {
                        status_text = "AWG Error (Not Connected).";
                    }
                }
                else {
                    // シミュレーション
                    bool success = false;
                    if (g_wave_type == 4) {
                        if (LoadWavFile(file_path, t_data, v_data)) success = true;
                        else status_text = "Load Failed.";
                    }
                    else {
                        g_size = 8192;
                        double period = (g_target_freq_shared > 0.0) ? (1.0 / g_target_freq_shared) : 1.0;
                        double target_duration = period * 5.0;
                        g_dt = target_duration / g_size;

                        t_data.resize(g_size); v_data.resize(g_size);
                        std::mt19937 mt(1); std::uniform_real_distribution<double> dist(-1.0, 1.0);
                        double phase_rad = g_gen_phase * PI / 180.0;

                        for (int i = 0; i < g_size; ++i) {
                            double t = i * g_dt;
                            double wt = 2.0 * PI * g_target_freq_shared * t + phase_rad;
                            double sig = 0.0;
                            if (g_wave_type == 0) sig = sin(wt);
                            else if (g_wave_type == 1) sig = (sin(wt) >= 0) ? 1.0 : -1.0;
                            else if (g_wave_type == 2) sig = 2.0 / PI * asin(sin(wt));
                            else if (g_wave_type == 3) sig = 2.0 / PI * atan(tan(wt / 2.0));
                            t_data[i] = t; v_data[i] = g_gen_amp * sig + noise * dist(mt);
                        }
                        success = true;
                    }

                    if (success) {
                        std::ofstream ofs(CSV_FILENAME);
                        if (ofs) {
                            ofs << "t,v\n";
                            for (size_t i = 0; i < v_data.size(); ++i) ofs << t_data[i] << "," << v_data[i] << "\n";
                            status_text = "Success.";
                        }
                    }
                }
            }
            ImGui::SameLine(); ImGui::Text(status_text.c_str());
        }
        ImGui::End();
    }

    // =========================================================
    // Window 2: ボード線図
    // =========================================================
    void ShowBodePlotWindow(const char* title) {
        ImGui::SetNextWindowPos(ImVec2(0, 500 * Gui::monitorScale), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(450 * Gui::monitorScale, 520 * Gui::monitorScale), ImGuiCond_FirstUseEver);

        static std::vector<double> freqs, g1, g2, g3, p1, p2, p3;
        static double prev_cutoff = -1.0;
        static int prev_type = -1;
        static double prev_q = -1.0;
        static bool initialized = false;

        if (ImGui::Begin(title)) {
            bool manual_run = ImGui::Button("Run Calc");
            ImGui::SameLine(); ImGui::Text("(Sim Only)");

            bool changed = (g_cutoff_freq != prev_cutoff) || (g_filter_type != prev_type) || (g_q_val != prev_q);

            if (manual_run || changed || !initialized) {
                initialized = true;
                prev_cutoff = g_cutoff_freq;
                prev_type = g_filter_type;
                prev_q = g_q_val;

                freqs.clear(); g1.clear(); g2.clear(); g3.clear(); p1.clear(); p2.clear(); p3.clear();
                const double sim_dt = 1.0e-7; const double sim_rate = 1.0 / sim_dt;
                FilterStage filt(g_filter_type, g_cutoff_freq, sim_rate, g_q_val);

                int n_steps = 200;
                double log_start = log10(10e3); double log_stop = log10(1000e3);
                double log_step = (log_stop - log_start) / (n_steps - 1);

                for (int i = 0; i < n_steps; ++i) {
                    double f = pow(10.0, log_start + i * log_step);
                    freqs.push_back(f);
                    std::complex<double> H = filt.response(f, sim_dt);
                    double mag = std::abs(H);
                    double ph = std::arg(H) * 180.0 / PI;
                    while (ph > 100.0) ph -= 360.0; while (ph < -270.0) ph += 360.0;
                    g1.push_back(20.0 * log10(mag)); p1.push_back(ph);
                    g2.push_back(20.0 * log10(mag * mag));
                    double ph2 = ph * 2.0; while (ph2 > 100.0) ph2 -= 360.0; while (ph2 < -500.0) ph2 += 360.0;
                    p2.push_back(ph2);
                    g3.push_back(20.0 * log10(mag * mag * mag));
                    double ph3 = ph * 3.0; while (ph3 > 100.0) ph3 -= 360.0; while (ph3 < -700.0) ph3 += 360.0;
                    p3.push_back(ph3);
                }
            }

            ImPlot::SetNextAxesToFit();
            if (ImPlot::BeginPlot("Gain", ImVec2(-1, 210 * Gui::monitorScale))) {
                ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
                ImPlot::SetupAxes("Frequency (Hz)", "Gain (dB)");
                if (!freqs.empty()) {
                    ImPlot::PlotLine("1st", freqs.data(), g1.data(), (int)freqs.size());
                    ImPlot::PlotLine("2nd", freqs.data(), g2.data(), (int)freqs.size());
                }
                ImPlot::EndPlot();
            }
            ImPlot::SetNextAxesToFit();
            if (ImPlot::BeginPlot("Phase", ImVec2(-1, 210 * Gui::monitorScale))) {
                ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
                ImPlot::SetupAxes("Frequency (Hz)", "Phase (Deg.)");
                if (!freqs.empty()) {
                    ImPlot::PlotLine("1st", freqs.data(), p1.data(), (int)freqs.size());
                    ImPlot::PlotLine("2nd", freqs.data(), p2.data(), (int)freqs.size());
                }
                ImPlot::EndPlot();
            }
        }
        ImGui::End();
    }

    // =========================================================
    // Window 3: 波形・フィルタ・FFT表示
    // =========================================================
    void ShowViewerWindow(const char* title) {
        static int order = 2;
        static std::string status_view = "", status_save = "";
        static std::vector<double> t_raw, v_raw, v_lpf, fft_freq, fft_mag_raw, fft_mag_lpf;
        static std::vector<double> fft_freq_mhz;
        static double raw_x = 0, raw_y = 0, raw_amp = 0, raw_phase = 0;
        static double lpf_x = 0, lpf_y = 0, lpf_amp = 0, lpf_phase = 0;

        static bool manual_view = false;
        static double view_offset = 0.0, view_span = 0.00005;
        static bool manual_fft_view = false;
        static double fft_view_offset = 0.0, fft_view_span = 1.0;

        static char save_prefix[64] = "data";
        static bool need_update = false, is_init = false;

        // 初期化
        if (!is_init) {
            t_raw.resize(g_size); v_raw.resize(g_size);
            for (int i = 0; i < g_size; i++) {
                t_raw[i] = i * g_dt;
                v_raw[i] = sin(2 * PI * g_target_freq_shared * t_raw[i]);
            }
            need_update = true;
            is_init = true;
        }

        ImGui::SetNextWindowPos(ImVec2(450 * Gui::monitorScale, 0), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(830 * Gui::monitorScale, 800 * Gui::monitorScale), ImGuiCond_FirstUseEver);

        if (ImGui::Begin(title)) {
            ImGui::PushItemWidth(200 * Gui::monitorScale);
            if (ImGui::InputDouble("Ref Freq. (Hz)", &g_target_freq_shared, 100.0, 100000.0, "%.1f")) need_update = true;
            if (ImGui::InputInt("Filter Order", &order)) need_update = true;
            ImGui::PopItemWidth();
            if (order < 1) order = 1; if (order > 3) order = 3;

            // --- キャプチャ制御 ---
            if (ImGui::Button(g_use_visa ? "Capture (Single)" : "View (Load)")) {
                if (g_use_visa) {
                    if (g_scope) {
                        viClear(g_scope);
                        viPrintf(g_scope, ":STOP\n");
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));

                        viPrintf(g_scope, ":COMMunicate:HEADer OFF\n");
                        viPrintf(g_scope, ":WAVeform:TRACe 1\n");
                        viPrintf(g_scope, ":WAVeform:FORMat ASCii\n");
                        viPrintf(g_scope, ":WAVeform:STARt 0\n");
                        viPrintf(g_scope, ":WAVeform:END 12500\n");

                        char tdiv_buf[256] = { 0 };
                        ViUInt32 rc = 0;
                        viPrintf(g_scope, ":TIMebase:TDIV?\n");
                        viRead(g_scope, (ViPBuf)tdiv_buf, 255, &rc);

                        std::string tdiv_str = tdiv_buf;
                        tdiv_str.erase(std::remove(tdiv_str.begin(), tdiv_str.end(), '\n'), tdiv_str.end());
                        tdiv_str.erase(std::remove(tdiv_str.begin(), tdiv_str.end(), '\r'), tdiv_str.end());

                        double tdiv = 0.0;
                        try { if (!tdiv_str.empty()) tdiv = std::stod(tdiv_str); }
                        catch (...) { tdiv = 0.0; }
                        if (tdiv <= 0.0) tdiv = 1.0e-3;

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
                        viPrintf(g_scope, ":STARt\n");

                        std::vector<double> temp_v, temp_t;
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

                            if (!temp_v.empty()) {
                                int new_size = (int)temp_v.size();
                                double total_time = tdiv * 10.0;
                                double new_dt = total_time / new_size;

                                temp_t.resize(new_size);
                                for (int i = 0; i < new_size; ++i) temp_t[i] = i * new_dt;

                                g_size = new_size; g_dt = new_dt;
                                v_raw = temp_v; t_raw = temp_t;
                                status_view = "Captured (T/Div: " + std::to_string(tdiv) + ")";
                                need_update = true;
                            }
                            else status_view = "Parse Error";
                        }
                        else status_view = "No Data Received";
                    }
                    else status_view = "Scope Not Connected";
                }
                else {
                    std::ifstream ifs(CSV_FILENAME);
                    if (ifs) {
                        std::vector<double> temp_v, temp_t;
                        std::string line; std::getline(ifs, line);
                        double t0 = -1, t1 = -1;
                        while (std::getline(ifs, line)) {
                            size_t c = line.find(',');
                            if (c != std::string::npos) {
                                double t = std::stod(line.substr(0, c)), v = std::stod(line.substr(c + 1));
                                if (t0 < 0) t0 = t; else if (t1 < 0) t1 = t;
                                temp_t.push_back(t); temp_v.push_back(v);
                            }
                        }
                        if (!temp_t.empty()) {
                            g_size = (int)temp_t.size();
                            g_dt = (t1 > t0) ? (t1 - t0) : 1.0e-7;
                            t_raw = temp_t; v_raw = temp_v;
                            status_view = "Loaded."; need_update = true;
                        }
                    }
                    else status_view = "File Not Found";
                }
            }
            ImGui::SameLine(); ImGui::Text(status_view.c_str());

            // --- 保存 UI ---
            ImGui::Separator();
            ImGui::PushItemWidth(150 * Gui::monitorScale);
            ImGui::InputText("File Prefix", save_prefix, sizeof(save_prefix));
            ImGui::PopItemWidth();
            ImGui::SameLine();

            if (ImGui::Button("Save CSV") && !t_raw.empty()) {
                // 1. オシロスコープ波形（Filter Output）の保存
                std::string wave_fname = std::string(save_prefix) + "_waveform_out.csv";
                std::ofstream ofs(wave_fname);
                if (ofs) {
                    // OFFの場合は2列だけ出力
                    if (g_filter_type == FILT_OFF) {
                        ofs << "t,raw\n";
                        for (size_t i = 0; i < t_raw.size(); i++) {
                            ofs << t_raw[i] << "," << v_raw[i] << "\n";
                        }
                    }
                    else {
                        ofs << "t,raw,lpf\n";
                        for (size_t i = 0; i < t_raw.size(); i++) {
                            ofs << t_raw[i] << "," << v_raw[i] << "," << (i < v_lpf.size() ? v_lpf[i] : 0) << "\n";
                        }
                    }
                }

                // 2. FG波形（Filter Input）の保存
                std::string input_fname = std::string(save_prefix) + "_waveform_input.csv";
                std::ofstream ofs_in(input_fname);
                if (ofs_in) {
                    ofs_in << "t,v_input\n";
                    double rad_phase = g_gen_phase * PI / 180.0;
                    double amp = g_gen_amp / 2.0;

                    for (size_t i = 0; i < t_raw.size(); i++) {
                        double t = t_raw[i];
                        double wt = 2.0 * PI * g_target_freq_shared * t + rad_phase;
                        double sig = 0.0;

                        if (g_wave_type == 0) sig = sin(wt);
                        else if (g_wave_type == 1) sig = (sin(wt) >= 0) ? 1.0 : -1.0;
                        else if (g_wave_type == 2) sig = 2.0 / PI * asin(sin(wt));
                        else if (g_wave_type == 3) sig = 2.0 / PI * atan(tan(wt / 2.0));

                        double val = amp * sig;
                        ofs_in << t << "," << val << "\n";
                    }
                }

                // 3. FFTデータの保存
                if (!fft_freq.empty() && fft_freq.size() == fft_mag_raw.size()) {
                    std::string fft_fname = std::string(save_prefix) + "_fft_out.csv";
                    std::ofstream ofs_fft(fft_fname);
                    if (ofs_fft) {
                        // OFFの場合はFFTも2列に
                        if (g_filter_type == FILT_OFF) {
                            ofs_fft << "Frequency(Hz),Mag_Raw\n";
                            for (size_t i = 0; i < fft_freq.size(); i++) {
                                ofs_fft << fft_freq[i] << "," << fft_mag_raw[i] << "\n";
                            }
                        }
                        else {
                            ofs_fft << "Frequency(Hz),Mag_Raw,Mag_LPF\n";
                            for (size_t i = 0; i < fft_freq.size(); i++) {
                                double mag_l = (i < fft_mag_lpf.size()) ? fft_mag_lpf[i] : 0.0;
                                ofs_fft << fft_freq[i] << "," << fft_mag_raw[i] << "," << mag_l << "\n";
                            }
                        }
                    }
                }
                status_save = "Saved All(In/Out/FFT).";
            }

            ImGui::SameLine();
            if (ImGui::Button("Capture BMP")) { g_capture_request = true; status_save = "BMP Saved."; }
            ImGui::SameLine(); ImGui::Text(status_save.c_str());

            ImGui::Separator();
            // OFFボタンを追加
            if (ImGui::RadioButton("OFF", g_filter_type == FILT_OFF)) { g_filter_type = FILT_OFF; need_update = true; } ImGui::SameLine();
            if (ImGui::RadioButton("LPF", g_filter_type == FILT_LPF)) { g_filter_type = FILT_LPF; need_update = true; } ImGui::SameLine();
            if (ImGui::RadioButton("HPF", g_filter_type == FILT_HPF)) { g_filter_type = FILT_HPF; need_update = true; } ImGui::SameLine();
            if (ImGui::RadioButton("BPF", g_filter_type == FILT_BPF)) { g_filter_type = FILT_BPF; need_update = true; }

            ImGui::PushItemWidth(300 * Gui::monitorScale);
            double min_c = 100.0, max_c = (1.0 / g_dt) / 2.0; if (max_c > 1.0e7) max_c = 1.0e7;
            if (ImGui::SliderScalar("Cutoff", ImGuiDataType_Double, &g_cutoff_freq, &min_c, &max_c, "%.0fHz")) need_update = true;
            if (g_filter_type == FILT_BPF) {
                double min_q = 0.1, max_q = 10.0;
                if (ImGui::SliderScalar("Q Factor", ImGuiDataType_Double, &g_q_val, &min_q, &max_q, "%.2f")) need_update = true;
            }
            ImGui::PopItemWidth();

            if (need_update && !v_raw.empty()) {
                double sample_rate = 1.0 / g_dt;
                std::vector<FilterStage> filters;
                for (int i = 0; i < order; i++) filters.push_back(FilterStage(g_filter_type, g_cutoff_freq, sample_rate, g_q_val));

                v_lpf.resize(v_raw.size());

                // OFFの場合はフィルタ計算をスキップ（またはRawをコピー）してグラフ表示用に備える
                if (g_filter_type == FILT_OFF) {
                    v_lpf = v_raw; // グラフ上でRawと重なるようにする
                }
                else {
                    for (size_t i = 0; i < v_raw.size(); ++i) {
                        double val = v_raw[i];
                        for (auto& f : filters) val = f.process(val);
                        v_lpf[i] = val;
                    }
                }

                fft_mag_raw.clear(); fft_freq.clear(); fft_mag_lpf.clear();
                compute_fft(v_raw, fft_mag_raw, fft_freq);
                compute_fft(v_lpf, fft_mag_lpf, fft_freq);

                double x, y;
                psd(v_raw, g_target_freq_shared, g_dt, &x, &y);
                raw_amp = sqrt(x * x + y * y); raw_phase = atan2(y, x) * 180.0 / PI;
                psd(v_lpf, g_target_freq_shared, g_dt, &x, &y);
                lpf_amp = sqrt(x * x + y * y); lpf_phase = atan2(y, x) * 180.0 / PI;

                fft_freq_mhz.resize(fft_freq.size());
                for (size_t k = 0; k < fft_freq.size(); k++) fft_freq_mhz[k] = fft_freq[k] / 1.0e6;

                need_update = false;
            }

            ImGui::Text("Ref: %.1f Hz | RAW A: %.3f V, P: %.1f | FILT A: %.3f V, P: %.1f",
                g_target_freq_shared, raw_amp, raw_phase, lpf_amp, lpf_phase);

            ImGui::Separator();
            if (ImGui::Checkbox("Manual Time", &manual_view)) { if (!manual_view) ImPlot::SetNextAxesToFit(); }
            if (manual_view) {
                double max_time = (!t_raw.empty()) ? t_raw.back() : 1.0;
                double min_span = g_dt * 10.0;
                ImGui::SliderScalar("Time Pos", ImGuiDataType_Double, &view_offset, &min_span, &max_time);
                ImGui::SliderScalar("Time Width", ImGuiDataType_Double, &view_span, &min_span, &max_time);
            }

            if (ImPlot::BeginPlot("Waveform", ImVec2(-1, 280 * Gui::monitorScale))) {
                if (g_capture_request) {
                    g_capture_pos = ImGui::GetItemRectMin();
                    g_capture_size = ImGui::GetItemRectSize();
                }
                ImPlot::SetupLegend(ImPlotLocation_NorthEast);
                ImPlot::SetupAxis(ImAxis_Y1, NULL, ImPlotAxisFlags_AutoFit);
                ImPlot::SetupAxes("Time (s)", "Voltage (V)");

                if (manual_view) ImPlot::SetupAxisLimits(ImAxis_X1, view_offset, view_offset + view_span, ImPlotCond_Always);
                else {
                    double disp_duration = (g_dt * g_size > 0) ? (g_dt * g_size) : 1.0;
                    ImPlot::SetupAxisLimits(ImAxis_X1, 0, disp_duration, ImPlotCond_Always);
                }

                if (!t_raw.empty()) {
                    ImPlot::PlotLine("Ch1", t_raw.data(), v_raw.data(), (int)t_raw.size());
                    // OFFじゃない時だけフィルタ線を描画したい場合はここをifで囲む
                    // 今回はON/OFFでCSVの列制御がメインなので、OFF時はRawと重なって表示されます
                    ImPlot::PlotLine("Filt", t_raw.data(), v_lpf.data(), (int)v_lpf.size());
                }
                ImPlot::EndPlot();
            }

            ImGui::Separator();
            if (ImGui::Checkbox("Manual FFT", &manual_fft_view)) { if (!manual_fft_view) ImPlot::SetNextAxesToFit(); }

            if (manual_fft_view) {
                double max_freq_mhz = (1.0 / g_dt) / 2.0 / 1.0e6; if (max_freq_mhz < 0.1) max_freq_mhz = 0.1;
                double min_span = max_freq_mhz * 0.001;
                ImGui::SliderScalar("Freq Pos", ImGuiDataType_Double, &fft_view_offset, &min_span, &max_freq_mhz);
                ImGui::SliderScalar("Freq Width", ImGuiDataType_Double, &fft_view_span, &min_span, &max_freq_mhz);
            }

            if (ImPlot::BeginPlot("FFT", ImVec2(-1, 280 * Gui::monitorScale))) {
                ImPlot::SetupAxis(ImAxis_Y1, NULL, ImPlotAxisFlags_AutoFit);
                ImPlot::SetupAxes("Freq (MHz)", "Mag (V)");

                if (manual_fft_view) {
                    ImPlot::SetupAxisLimits(ImAxis_X1, fft_view_offset, fft_view_offset + fft_view_span, ImPlotCond_Always);
                }
                else {
                    double max_disp = 5.0;
                    if (g_target_freq_shared > 0) {
                        double nyquist_mhz = (1.0 / g_dt) / 2.0 / 1.0e6;
                        double harmonics_view = (g_target_freq_shared * 20.0) / 1.0e6;
                        max_disp = (harmonics_view < nyquist_mhz) ? harmonics_view : nyquist_mhz;
                        if (max_disp < 0.02) max_disp = 0.02;
                    }
                    ImPlot::SetupAxisLimits(ImAxis_X1, 0, max_disp, ImPlotCond_Always);
                }
                if (!fft_freq_mhz.empty()) {
                    int pn = (int)fft_freq_mhz.size() / 2;
                    ImPlot::PlotLine("Ch1", fft_freq_mhz.data(), fft_mag_raw.data(), pn);
                    ImPlot::PlotLine("Filt", fft_freq_mhz.data(), fft_mag_lpf.data(), pn);
                }
                ImPlot::EndPlot();
            }
        }
        ImGui::End();
    }
}