# Modulo-Two to ARINC-429 converter
Converting a Modulo-Two Serial Stream to Arinc-429 Serial Words.
### What's it for?
The Modulo-Two Serial Stream originates from a rather obsolete (around 1970 style) airborn High Frequency Communication Radio Control Panel.
This panel is installed in the cockpit and it controls a High Frequency Transceiver located in an aircrafts electronic compartment.
The panel is a Collins Controller 514A-4 and the transceiver a Collins HF Receiver Transmitter HFS-700.
### Why do we need one?
We want this panel to control a somewhat less obsolete High Frequency Communication Transceiver (1990 to 2010?) that requires Arinc-429 standard control signals.
### OK, show us.
The converter is an electronic device build in a box. Signals from the panel go in, and the output is routed to the transceiver. The device consists of an electronic circuit board and connectors. On this board a microcontroller works together with some support circuits to transform the incoming serial stream in Arinc-429 serial words.

