#include <iostream>
#include <string>
#include "AppApps.h" // 共通ヘッダーをインクルード

int main() {
    int choice = 0;

    std::cout << "========================================\n";
    std::cout << " Select Application to Run:\n";
    std::cout << "========================================\n";
    std::cout << " 1. Waveform Analysis & Filter (251217)\n";
    std::cout << " 2. Bode Plot & Sweep Control  (251218)\n";
    std::cout << "========================================\n";
    std::cout << " Enter number (1 or 2): ";

    if (!(std::cin >> choice)) {
        std::cout << "Invalid input.\n";
        return -1;
    }

    // 選択に応じてそれぞれの名前空間内の Run() を呼び出す
    if (choice == 1) {
        std::cout << "Starting Waveform Analysis App...\n";
        return App_Analysis::Run();
    }
    else if (choice == 2) {
        std::cout << "Starting Bode Sweep App...\n";
        return App_Sweep::Run();
    }
    else {
        std::cout << "Unknown selection.\n";
        return -1;
    }
}