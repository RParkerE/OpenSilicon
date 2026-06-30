#ifndef TFT_BL
#define TFT_BL -1
#endif
#include "hal/implementations/TFTDisplayHAL.h"
#include <TFT_eSPI.h>

namespace HAL {

    void TFTDisplayHAL::begin() {
        tft_.init();
        tft_.setRotation(1);

        if (TFT_BL >= 0) {
            pinMode(TFT_BL, OUTPUT);
            digitalWrite(TFT_BL, HIGH);
        }

        tft_.fillScreen(TFT_BLACK);

        tft_.setTextFont(1);
        tft_.setTextDatum(TL_DATUM);
        tft_.setTextColor(TFT_WHITE, TFT_BLACK);
        tft_.setTextSize(2);
        tft_.setCursor(60, 100);
        tft_.println("SPIN COATER");
        tft_.setTextSize(1);
        tft_.setCursor(80, 130);
        tft_.println("Initializing...");

        Serial.println("TFT init OK");
    }

    void TFTDisplayHAL::render(const Domain::DisplayModel& model) {
        tft_.startWrite();

        if (model.activePage == Domain::DisplayPage::Fault) {
            tft_.fillScreen(TFT_BLACK);
            tft_.setTextDatum(TL_DATUM);
            tft_.setTextColor(TFT_RED, TFT_BLACK);
            tft_.setTextFont(1);
            tft_.setTextSize(3);
            tft_.setCursor(10, 30);
            tft_.printf("FAULT %d", model.faultCode);
            tft_.setTextSize(2);
            tft_.setCursor(10, 90);
            tft_.println(model.faultDescription);
            tft_.endWrite();
            return;
        }

        tft_.fillScreen(TFT_BLACK);
        tft_.setTextFont(1);
        tft_.setTextDatum(TL_DATUM);

        // Title
        tft_.setTextColor(TFT_WHITE, TFT_BLACK);
        tft_.setTextSize(2);
        tft_.setCursor(10, 8);
        tft_.print("SPIN COATER");

        // Phase
        tft_.setTextColor(TFT_CYAN, TFT_BLACK);
        tft_.setTextSize(2);
        tft_.setCursor(10, 40);
        tft_.print(model.statusMessage);

        // Actual RPM (large)
        tft_.setTextColor(TFT_GREEN, TFT_BLACK);
        tft_.setTextSize(4);
        tft_.setCursor(10, 76);
        tft_.printf("%ld", static_cast<long>(model.actualSpeed));
        tft_.setTextSize(2);
        tft_.print(" RPM");

        // Target RPM
        tft_.setTextColor(TFT_WHITE, TFT_BLACK);
        tft_.setTextSize(2);
        tft_.setCursor(10, 128);
        tft_.printf("Target: %ld", static_cast<long>(model.targetSpeed));

        // Recipe name + step
        tft_.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft_.setTextSize(2);
        tft_.setCursor(10, 165);
        tft_.print(model.recipeName[0] ? model.recipeName : "(no recipe)");

        if (model.totalSteps > 0) {
            tft_.setTextColor(TFT_WHITE, TFT_BLACK);
            tft_.setCursor(10, 195);
            tft_.printf("Step %d/%d", model.currentStep, model.totalSteps);
        }

        tft_.endWrite();
    }

    void TFTDisplayHAL::clear() {
        tft_.startWrite();
        tft_.fillScreen(TFT_BLACK);
        tft_.endWrite();
    }

    void TFTDisplayHAL::setBrightness(uint8_t percent) {
        if (TFT_BL >= 0) {
            uint8_t pwm = map(percent, 0, 100, 0, 255);
            analogWrite(TFT_BL, pwm);
        }
    }
}
