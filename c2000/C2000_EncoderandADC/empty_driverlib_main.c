#include "driverlib.h"
#include "device.h"
#include <stdio.h>

// --- Ayarlar ---
#define SAMPLES_PER_WINDOW 200  // Peak-to-peak hesabı için kaç örnek alınacak

// --- Değişkenler ---
volatile int32_t encoderCount = 0;
uint16_t ledSayac = 0;

// --- Fonksiyon Prototipleri ---
void initADC(void);
void initSCIA(void);
void initSoftwareEncoder(void);
__interrupt void xint1_isr(void);

void main(void)
{
    // 1. Cihaz ve Clock Ayarları
    Device_init();
    Device_initGPIO();

    // 2. LED Ayarı
    GPIO_setPadConfig(DEVICE_GPIO_PIN_LED1, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_LED1, GPIO_DIR_MODE_OUT);

    // 3. Interrupt Tablosu Hazırlığı
    Interrupt_initModule();
    Interrupt_initVectorTable();

    // 4. Modülleri Başlat
    initADC();
    initSCIA();
    initSoftwareEncoder();

    // 5. Global Interruptları Aç
    EINT;
    ERTM;

    while(1)
    {
        // --- A) PEAK-TO-PEAK HESAPLAMA ---
        uint16_t vMax = 0;
        uint16_t vMin = 4095;
        uint16_t currentSample;
        int i;

        // Çok hızlı bir şekilde sinyalin tepelerini ve diplerini tara
        for(i = 0; i < SAMPLES_PER_WINDOW; i++)
        {
            // ADC Tetikle
            ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER0);

            // Dönüşüm bitene kadar bekle
            while(ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1) == 0);
            ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

            // Değeri oku
            currentSample = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);

            // En büyük ve en küçüğü güncelle
            if(currentSample > vMax) vMax = currentSample;
            if(currentSample < vMin) vMin = currentSample;

            // Örnekleme hızı ayarı (Sinyal frekansınıza göre burayı azaltıp artırabilirsiniz)
            DEVICE_DELAY_US(10);
        }

        // Peak-to-Peak değerini bul
        uint16_t finalVpp = vMax - vMin;

        // --- B) Encoder Değerini Al ---
        int32_t currentEnc = encoderCount;

        // --- C) VERİ PAKETİ GÖNDERİMİ (SCIA) ---
        // Toplam 6 Byte: 2 Byte Vpp + 4 Byte Encoder

        // Vpp (ADC Sonucu)
        SCI_writeCharBlockingFIFO(SCIA_BASE, (finalVpp >> 8) & 0xFF);
        SCI_writeCharBlockingFIFO(SCIA_BASE, finalVpp & 0xFF);

        // Encoder (32-bit signed integer)
        SCI_writeCharBlockingFIFO(SCIA_BASE, (uint16_t)((currentEnc >> 24) & 0xFF));
        SCI_writeCharBlockingFIFO(SCIA_BASE, (uint16_t)((currentEnc >> 16) & 0xFF));
        SCI_writeCharBlockingFIFO(SCIA_BASE, (uint16_t)((currentEnc >> 8) & 0xFF));
        SCI_writeCharBlockingFIFO(SCIA_BASE, (uint16_t)(currentEnc & 0xFF));

        // --- D) Döngü Hızı ve LED ---
        // Veri gönderim hızı (Örn: 20ms'de bir paket)
        DEVICE_DELAY_US(100000);

        ledSayac++;
        if(ledSayac > 25)
        {
            GPIO_togglePin(DEVICE_GPIO_PIN_LED1);
            ledSayac = 0;
        }
    }
}

// --- Modül Başlatma Fonksiyonları ---

void initSCIA(void)
{
    GPIO_setPinConfig(GPIO_42_SCITXDA);
    GPIO_setPinConfig(GPIO_43_SCIRXDA);

    // Python tarafı 460800 kullanıyorsa burayı da 460800 yapın!
    // Eğer Python 115200 ise burası böyle kalsın.
    SCI_setConfig(SCIA_BASE, DEVICE_LSPCLK_FREQ, 115200,
                  (SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE | SCI_CONFIG_PAR_NONE));

    SCI_resetChannels(SCIA_BASE);
    SCI_resetRxFIFO(SCIA_BASE);
    SCI_resetTxFIFO(SCIA_BASE);
    SCI_enableModule(SCIA_BASE);
    SCI_enableFIFO(SCIA_BASE);
}

void initADC(void)
{
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);
    ADC_enableConverter(ADCA_BASE);
    DEVICE_DELAY_US(1000);

    // S+H Window 30: Daha hızlı örnekleme için biraz düşürüldü
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 30);

    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
}

void initSoftwareEncoder(void)
{
    GPIO_setDirectionMode(94, GPIO_DIR_MODE_IN); // Kanal A
    GPIO_setPadConfig(94, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(97, GPIO_DIR_MODE_IN); // Kanal B
    GPIO_setPadConfig(97, GPIO_PIN_TYPE_PULLUP);

    GPIO_setInterruptPin(94, GPIO_INT_XINT1);
    GPIO_setInterruptType(GPIO_INT_XINT1, GPIO_INT_TYPE_BOTH_EDGES);

    Interrupt_register(INT_XINT1, &xint1_isr);
    GPIO_enableInterrupt(GPIO_INT_XINT1);
    Interrupt_enable(INT_XINT1);
}

__interrupt void xint1_isr(void)
{
    uint16_t aState = GPIO_readPin(94);
    uint16_t bState = GPIO_readPin(97);
    if (aState != bState) encoderCount++;
    else                  encoderCount--;
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}
