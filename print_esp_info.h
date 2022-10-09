/*
 * print_esp_info.h
 *
 *  Created on: 2022-10-06
 *      Author: MrCejota (Carlos Sola)
 */

 void printESP_Info(void){
    printf("\n");
    printf("SketchSize: %d B\n", ESP.getSketchSize());
    printf("FreeSketchSpace: %d B\n", ESP.getFreeSketchSpace());
    printf("FlashChipSize: %d B\n", ESP.getFlashChipSize());
    printf("FlashChipRealSize: %d B\n", ESP.getFlashChipRealSize());
    printf("FlashChipSpeed: %d\n", ESP.getFlashChipSpeed());
    printf("SdkVersion: %s\n", ESP.getSdkVersion());
    printf("FullVersion: %s\n", ESP.getFullVersion().c_str());
    printf("CpuFreq: %dMHz\n", ESP.getCpuFreqMHz());
    printf("FreeHeap: %d B\n", ESP.getFreeHeap());
    printf("ResetInfo: %s\n", ESP.getResetInfo().c_str());
    printf("ResetReason: %s\n", ESP.getResetReason().c_str());

 }