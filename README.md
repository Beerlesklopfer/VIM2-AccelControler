# VIM2-AccelControler

Diese Arbeit erläutert den Zusammenhang und Aufbau eines Versuchs zur rotatorischen Beschleunigung und
dessen Widerstandsmomentes. Die Verwendung eines Schrittmotors in der mechanischen Versuchsapparatur und die
Anbindung via Arduino mit custom Firmware ermöglicht die Kontrolle aller Parameter via Modbus. Der Versuch kann
von einer Widget-basierten Matlab-App per GUI kontrolliert werden. Versuchsdaten werden dabei vollautomatisch
erzeugt, aufgezeichnet und visuell für den Nutzer aufbereitet. Die asynchrone Modbus-Kommunikation zwischen App
und Controller ließ sich im Rahmen dieses Projektes nicht fehlerfrei realisieren. Auftretenden Fehler wurden
aber eingehend analysiert und Lösungsvorschläge entwickelt.
