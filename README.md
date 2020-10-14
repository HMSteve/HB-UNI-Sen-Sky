# Himmelssensor (Helligkeit, UV-Index, Himmelstemperatur)

Eine AskSinPP-Implementierung eines Sensors auf Basis des MAX44009, VEML6065 und MLX90614. Als Hardwarebasis dient z.B. meine Universalsensorplatine:

[AskSinPP Universal Board](https://github.com/HMSteve/PCBs/tree/master/AskSinPP_UniversalBoard)

Neben UV-Index und Helligkeit wird mittels IR-Sensor die Himmelstemperatur und zudem die Umgebungstemperatur gemessen. Relativ niedrige Himmelstemperatur ist ein Indikator fuer klaren Himmel, steigende Temperatur ein Indikator fuer aufziehende Bedeckung. Damit ist der Sensor fuer automatisierte Strofotografie hilfreich - es kann auf umschlagende Wetterbedingungen fruehzeitig reagiert werden.

## Software

Sofern noch nicht vorhanden oder nicht aktuell, ist das [Addon](https://github.com/HMSteve/SG-HB-Devices-Addon/raw/master/CCU_RM/sg-hb-devices-addon.tgz) auf der CCU zu installieren. Der Sensor benoetigt mindestens Version 1.0.

Die AskSinPP-Platine wird wie ueblich geflasht und kann dann angelernt werden.


## Hardwarebasis

Der MLX90614 muss ohne vorgelagerten Wetterschutz montiert werden. Es bietet sich der wasserdichte Einbau in eine PG-Verschraubung an. Der VEML9065 sollte hinter einem UV-durchlaessigen Schutzglas montiert werden.


## Disclaimer

Die Nutzung der hier veroeffentlichten Inhalte erfolgt vollstaendig auf eigenes Risiko und ohne jede Gewaehr.


## Lizenz

Creative Commons BY-NC-SA
