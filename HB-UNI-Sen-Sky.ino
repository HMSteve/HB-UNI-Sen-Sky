//---------------------------------------------------------
// HB-UNI-Sen-Sky 2020-07-31 HMSteve (Stephan)
// https://creativecommons.org/licenses/by-nc-sa/4.0/
// You are free to Share & Adapt under the following terms:
// Give Credit, NonCommercial, ShareAlike
// +++
// AskSin++ 2016-10-31 papa Creative Commons
// HB-UNI-Sensor1 2019-10-09 Tom Major (Creative Commons)
//---------------------------------------------------------

//---------------------------------------------------------
// !! NDEBUG sollte aktiviert werden wenn die Sensorentwicklung und die Tests abgeschlossen sind und das Gerät in den 'Produktionsmodus' geht.
// Insbesondere die RAM-Einsparungen sind wichtig für die Stabilität / dynamische Speicherzuweisungen etc.
// Dies beseitigt dann auch die mögliche Arduino-Warnung 'Low memory available, stability problems may occur'.
//
//#define NDEBUG

//---------------------------------------------------------
// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER

#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <AskSinPP.h>
#include <LowPower.h>
#include <MultiChannelDevice.h>
#include <Register.h>
#include "Sensors/tmBattery.h"

//---------------------------------------------------------
// Alle Device Parameter werden aus einer .h Datei (hier im Beispiel Cfg/Device_Example.h) geholt um mehrere Geräte ohne weitere Änderungen des
// Sketches flashen zu können. Für mehrere Geräte einfach mehrere .h Dateien anlegen und dort die Unterschiede zwischen den Geräten definieren. Die
// konfigurierbaren Device Parameter in der .h Datei sind im Einzelnen:
// - Device ID und Device Serial
// - Aktivierung der verwendeten Sensoren
// - Pin Definitionen Allgemein
// - Pin und Address Definitionen der Sensoren
// - Clock Definition
// - Schaltungsvariante und Pins für Batteriespannungsmessung
// - Schwellwerte für Batteriespannungsmessung


// unique device radio ID and logic serial
#define cDEVICE_ID      { 0xF8, 0x20, 0x01 }
#define cDEVICE_SERIAL  "SGSENSKY01"

// used sensors and addresses
#define SENSOR_MAX44009     
#define SENSOR_VEML6075
#define SENSOR_MLX90614
#define ONEWIRE_PIN       3
#define MAX44009_ADDR       0x4A
#define MLX90614_I2CADDR       0x5A

// scaling factors for compensated signals, calibrated by regression to BSF Lindenberg meassurements
#define VEML6075_UVA_RESPONSE 0.003204
#define VEML6075_UVB_RESPONSE 0.0002274

// pin definitions
#define CONFIG_BUTTON_PIN   5
#define LED_PIN             6


// Clock options
// CLOCK_SYSCLOCK: 8MHz Quarz an XTAL oder 8MHz int. RC-Oszillator, Sleep Strom ca. 4uA
// CLOCK_RTC:      8MHz int. RC-Oszillator, 32.768kHz Quarz an XTAL, Sleep Strom ca. 1uA
#define CLOCK_SYSCLOCK


// Schaltungsvariante und Pins für Batteriespannungsmessung, siehe README
// 1) Standard: tmBattery, UBatt = Betriebsspannung AVR: #define BAT_SENSOR tmBattery
// 2) für StepUp/StepDown: tmBatteryResDiv, sense pin A0, activation pin A1, Faktor = Rges/Rlow*1000, z.B. 470k/100k, Faktor 570k/100k*1000 = 5700: #define BAT_SENSOR tmBatteryResDiv<A0, A1, 5700>
// 3) Echte Batteriespannungsmessung unter Last: tmBatteryLoad: sense pin A0, activation pin D9, Faktor = Rges/Rlow*1000, z.B. 10/30 Ohm, Faktor 40/10*1000 = 4000, 200ms Belastung vor Messung: //#define BAT_SENSOR tmBatteryLoad<A0, 9, 4000, 200>
#define BAT_SENSOR tmBatteryResDiv<A0, A1, 5700>


// Schwellwerte für Batteriespannungsmessung
#define BAT_VOLT_LOW        18  // 1.8V
#define BAT_VOLT_CRITICAL   16  // 1.5V


// number of available peers per channel
#define PEERS_PER_CHANNEL 6


// all library classes are placed in the namespace 'as'
using namespace as;

#ifdef SENSOR_DS18X20
#include "Sensors/Sens_DS18X20.h"    // HB-UNI-Sensor1 custom sensor class
#endif

#ifdef SENSOR_MAX44009
#include "Sensors/Sens_MAX44009.h"    // HB-UNI-Sensor1 custom sensor class
#endif

#ifdef SENSOR_VEML6075
#include "Sensors/Sens_VEML6075.h"    // HB-UNI-Sensor1 custom sensor class, needs original 6075 lib too
#endif

#ifdef SENSOR_MLX90614
#include "Sensors/Sens_MLX90614.h"  
//#include <sensors/MLX90614.h>  
#endif


#ifdef CLOCK_SYSCLOCK
#define CLOCK sysclock
#define SAVEPWR_MODE Sleep<>
#elif defined CLOCK_RTC
#define CLOCK rtc
#define SAVEPWR_MODE SleepRTC
#undef seconds2ticks
#define seconds2ticks(tm) (tm)
#else
#error INVALID CLOCK OPTION
#endif

// define all device properties
const struct DeviceInfo PROGMEM devinfo = {
    cDEVICE_ID,        // Device ID
    cDEVICE_SERIAL,    // Device Serial
    { 0xF8, 0x20 },    // Device Model  
    // Firmware Version 
    // die CCU Addon xml Datei ist mit der Zeile <parameter index="9.0" size="1.0" cond_op="E" const_value="0x13" />
    // fest an diese Firmware Version gebunden! cond_op: E Equal, GE Greater or Equal
    // bei Änderungen von Payload, message layout, Datenpunkt-Typen usw. muss die Version an beiden Stellen hochgezogen werden!
    0x10,                        
    as::DeviceType::THSensor,    // Device Type
    { 0x01, 0x01 }               // Info Bytes
};

// Configure the used hardware
typedef AvrSPI<10, 11, 12, 13>                 SPIType;
typedef Radio<SPIType, 2>                      RadioType;
typedef StatusLed<LED_PIN>                     LedType;
typedef AskSin<LedType, BAT_SENSOR, RadioType> BaseHal;

class Hal : public BaseHal {
public:
    void init(const HMID& id)
    {
        BaseHal::init(id);
#ifdef CLOCK_RTC
        rtc.init();    // init real time clock - 1 tick per second
#endif
        // measure battery every 12h
        battery.init(seconds2ticks(12UL * 60 * 60), CLOCK); 
        battery.low(BAT_VOLT_LOW);
        battery.critical(BAT_VOLT_CRITICAL);
    }

    bool runready() { return CLOCK.runready() || BaseHal::runready(); }
} hal;

class WeatherEventMsg : public Message {
public:
    void init(uint8_t msgcnt, int16_t ambTemperature, int16_t skyTemperature, uint32_t brightness, uint16_t uvIndex,
              uint16_t batteryVoltage, bool batLow)
    {

        uint8_t t1 = (ambTemperature >> 8) & 0x7f;
        uint8_t t2 = ambTemperature & 0xff;
        if (batLow == true) {
            t1 |= 0x80;    // set bat low bit
        }

        // als Standard wird BCAST gesendet um Energie zu sparen, siehe Beschreibung unten.
        // Bei jeder 20. Nachricht senden wir stattdessen BIDI|WKMEUP, um eventuell anstehende Konfigurationsänderungen auch
        // ohne Betätigung des Anlerntaster übernehmen zu können (mit Verzögerung, worst-case 20x Sendeintervall).
        uint8_t flags = BCAST;
        if ((msgcnt % 20) == 2) {
            flags = BIDI | WKMEUP;
        }
        Message::init(21, msgcnt, 0x70, flags, t1, t2);

        // Message Length (first byte param.): 11 + payload
        //  1 Byte payload -> length 12
        // 12 Byte payload -> length 23
        // max. payload: 17 Bytes (https://www.youtube.com/watch?v=uAyzimU60jw)

        // BIDI|WKMEUP: erwartet ACK vom Empfänger, ohne ACK wird das Senden wiederholt
        // LazyConfig funktioniert, d.h. eine anstehende Conf.Änderung von der CCU wird nach dem nächsten Senden übernommen. Aber erhöhter
        // Funkverkehr wegen ACK
        //
        // BCAST: ohne ACK zu Erwarten, Standard für HM Sensoren.
        // LazyConfig funktioniert nicht, d.h. eine anstehende Conf.Änderung von der CCU muss durch den Config Button am Sensor übernommen
        // werden!!

        // papa:
        // BIDI - fordert den Empfänger auf ein Ack zu schicken. Das wird auch zwingend für AES-Handling gebraucht. BCAST - signalisiert
        // eine Broadcast-Message. Das wird z.B. verwendet, wenn mehrere Peers vor einen Sensor existieren. Es wird dann an einen Peer
        // gesndet und zusätzlich das BCAST-Flag gesetzt. So dass sich alle die Nachrricht ansehen. Ein Ack macht dann natürlich keinen Sinn
        // - es ist ja nicht klar, wer das senden soll.
        //
        // WKMEUP - wird für LazyConfig verwendet. Ist es in einer Message gesetzt, so weiss
        // die Zentrale, dass das Geräte noch kurz auf weitere Nachrichten wartet. Die Lib setzt diese Flag für die StatusInfo-Message
        // automatisch. Außerdem bleibt nach einer Kommunikation der Empfang grundsätzlich für 500ms angeschalten.

        // skyTemperature
        pload[0] = (skyTemperature >> 8) & 0xff;
        pload[1] = skyTemperature & 0xff;

        // brightness (Lux)
        pload[2] = (brightness >> 24) & 0xff;
        pload[3] = (brightness >> 16) & 0xff;
        pload[4] = (brightness >> 8) & 0xff;
        pload[5] = (brightness >> 0) & 0xff;

        // UV index
        pload[6] = (uvIndex >> 8) & 0xff;
        pload[7] = uvIndex & 0xff;

        // batteryVoltage
        pload[8] = (batteryVoltage >> 8) & 0xff;
        pload[9] = batteryVoltage & 0xff;
    }
};

// die "freien" Register 0x20/21 werden hier als 16bit memory für das Update
// Intervall in Sek. benutzt siehe auch hb-uni-sensor1.xml, <parameter
// id="Sendeintervall"> 
DEFREGISTER(Reg0, MASTERID_REGS, DREG_LEDMODE, DREG_LOWBATLIMIT, DREG_TRANSMITTRYMAX, 0x20, 0x21)
class SensorList0 : public RegList0<Reg0> {
public:
    SensorList0(uint16_t addr)
        : RegList0<Reg0>(addr)
    {
    }

    bool     updIntervall(uint16_t value) const { return this->writeRegister(0x20, (value >> 8) & 0xff) && this->writeRegister(0x21, value & 0xff); }
    uint16_t updIntervall() const { return (this->readRegister(0x20, 0) << 8) + this->readRegister(0x21, 0); }


    void defaults()
    {
        clear();
        ledMode(1);
        lowBatLimit(BAT_VOLT_LOW);
        transmitDevTryMax(6);
        updIntervall(10);
    }
};

class WeatherChannel : public Channel<Hal, List1, EmptyList, List4, PEERS_PER_CHANNEL, SensorList0>, public Alarm {

    WeatherEventMsg msg;

    int16_t  ambTemperature10;
    int16_t  skyTemperature10;
    uint32_t brightness1000;
    uint16_t uvIndex10;
    uint16_t batteryVoltage;
    bool     regularWakeUp;


#ifdef SENSOR_DS18X20
    Sens_DS18X20 ds18x20;
#endif

#ifdef SENSOR_MAX44009
    Sens_MAX44009<MAX44009_ADDR> max44009;
#endif

#ifdef SENSOR_VEML6075
    Sens_VEML6075 veml6075;
#endif

#ifdef SENSOR_MLX90614
    Sens_MLX90614<MLX90614_I2CADDR> mlx90614;
#endif

public:
    WeatherChannel()
        : Channel()
        , Alarm(seconds2ticks(60))
        , ambTemperature10(0)
        , skyTemperature10(0)
        , brightness1000(0)
        , uvIndex10(0)
        , batteryVoltage(0)
        , regularWakeUp(true)
    {
    }
    virtual ~WeatherChannel() {}

    virtual void trigger(AlarmClock& clock)
    {
        measure();
        uint8_t msgcnt = device().nextcount();
        msg.init(msgcnt, ambTemperature10, skyTemperature10, brightness1000, uvIndex10, batteryVoltage, device().battery().low());
        if (msg.flags() & Message::BCAST) {
          device().broadcastEvent(msg, *this);
        }
        else
        {
          device().sendPeerEvent(msg, *this);
        }
         // reactivate for next measure
        uint16_t updCycle = this->device().getList0().updIntervall();
        set(seconds2ticks(updCycle));
        clock.add(*this);
        regularWakeUp = true;
    }

    void forceSend()
    {
        CLOCK.cancel(*this);
        regularWakeUp = false;    // Verhindert enableINT in trigger()
        trigger(CLOCK);           // Messen/Senden
        delay(250);               // Verzögerung für wiederholtes Senden bzw. digitalInput Entprellen
    }

    void measure()
    {


// Ambient und Sky Temperature vom MLX90614
#ifdef SENSOR_MLX90614
        mlx90614.wake();
        mlx90614.measure();
        ambTemperature10 = mlx90614.temperatureAmb();
        skyTemperature10 = mlx90614.temperatureObj();
        mlx90614.sleep();
#endif

// Falls DS18X20 vorhanden, statt MLX90614 ambient dessen Temp verwenden
#ifdef SENSOR_DS18X20
        ds18x20.measure();
        ambTemperature10 = ds18x20.temperature();
#endif

// Helligkeit  
#ifdef SENSOR_MAX44009
        max44009.measure();
        brightness1000 = max44009.brightnessLux();
#endif

// UV-Index 
#ifdef SENSOR_VEML6075
        veml6075.measure(VEML6075_UVA_RESPONSE, VEML6075_UVB_RESPONSE);
        uvIndex10 = veml6075.uvIndex10();
#endif

        batteryVoltage = device().battery().current();    // BatteryTM class, mV resolution
    }

    void initSensors()
    {
#ifdef SENSOR_DS18X20
        ds18x20.init(ONEWIRE_PIN);
#endif
#ifdef SENSOR_MAX44009
        max44009.init();
#endif
#ifdef SENSOR_VEML6075
        veml6075.init();
#endif
#ifdef SENSOR_MLX90614
        mlx90614.init();
#endif
 
        DPRINTLN(F("Sensor setup done"));
        DPRINT(F("Serial: "));
        DPRINTLN(cDEVICE_SERIAL);
#ifdef CLOCK_SYSCLOCK
        DPRINTLN(F("Clock SYSCLOCK"));
#elif defined CLOCK_RTC
        DPRINTLN(F("Clock RTC"));
#endif
    }

    void setup(Device<Hal, SensorList0>* dev, uint8_t number, uint16_t addr)
    {
        Channel::setup(dev, number, addr);
        initSensors();
        set(seconds2ticks(5));    // first message in 5 sec.
        CLOCK.add(*this);
    }

    void configChanged()
    {
        // DPRINTLN(F("Config changed: List1"));
    }

    uint8_t status() const { return 0; }

    uint8_t flags() const { return 0; }
};

class SensChannelDevice : public MultiChannelDevice<Hal, WeatherChannel, 1, SensorList0> {
public:
    typedef MultiChannelDevice<Hal, WeatherChannel, 1, SensorList0> TSDevice;
    SensChannelDevice(const DeviceInfo& info, uint16_t addr)
        : TSDevice(info, addr)
    {
    }
    virtual ~SensChannelDevice() {}

    virtual void configChanged()
    {
        TSDevice::configChanged();
        DPRINTLN(F("Config Changed: List0"));

        uint8_t ledMode = this->getList0().ledMode();
        DPRINT(F("ledMode: "));
        DDECLN(ledMode);

        uint8_t lowBatLimit = this->getList0().lowBatLimit();
        DPRINT(F("lowBatLimit: "));
        DDECLN(lowBatLimit);
        battery().low(lowBatLimit);

        uint8_t txDevTryMax = this->getList0().transmitDevTryMax();
        DPRINT(F("transmitDevTryMax: "));
        DDECLN(txDevTryMax);

        uint16_t updCycle = this->getList0().updIntervall();
        DPRINT(F("updCycle: "));
        DDECLN(updCycle);
    }
};

SensChannelDevice               sdev(devinfo, 0x20);
ConfigButton<SensChannelDevice> cfgBtn(sdev);

void setup()
{
    DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
    sdev.init(hal);
    buttonISR(cfgBtn, CONFIG_BUTTON_PIN);
    sdev.initDone();
}

void loop()
{
    bool worked = hal.runready();
    bool poll   = sdev.pollRadio();
    if (worked == false && poll == false) {
        // deep discharge protection
        // if we drop below critical battery level - switch off all and sleep forever
        if (hal.battery.critical()) {
            // this call will never return
            hal.activity.sleepForever(hal);
        }
        // if nothing to do - go sleep
        hal.activity.savePower<SAVEPWR_MODE>(hal);
    }
}
