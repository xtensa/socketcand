socketcand
==========

Socketcand is a daemon that provides access to CAN interfaces on a machine via a network interface. The communication protocol uses a TCP/IP connection and a specific protocol to transfer CAN frames and control commands. The protocol specification can be found in ./doc/protocol.md.

Fork
----
This fork implements CAN232 protocol over TCP. Upon connection it automatically goes into CAN232 mode and most commands as discribed in CAN232 protocol will work. Detailed information about CAN232 support is discribed under ./doc/can232.md.

Installation
------------

To build and run socketcand make sure you have the following tools installed:

* autoconf
* make
* gcc or another C compiler
* a kernel that includes the SocketCAN modules
* the headers for your kernel version
* the libconfig with headers (libconfig-dev under debian based systems)

First run

    $ autoconf

to create the 'configure' script.

Then run the created script with

    $ ./configure

to check your system and create the Makefile. If you want to install scripts for a init system other than SysVinit check the available settings with './configure -h'.
To compile and install the socketcand run

    $ make
    $ make install

Service discovery
-----------------

The daemon uses a simple UDP beacon mechanism for service discovery. A beacon containing the service name, type and address is sent to the broadcast address (port 42000) at minimum every 3 seconds. A client only has to listen for messages of this type to detect all SocketCAN daemons in the local network.

Usage
-----

    socketcand [-v | --verbose] [-i interfaces | --interfaces interfaces] [-p port | --port port] [-l ip_addr | --listen interface] [-h | --help]

###Description of the options
* **-v** activates verbose output to STDOUT
* **-i interfaces** is used to specify the SocketCAN interfaces the daemon shall provide access to
* **-p port** changes the default port (29536) the daemon is listening at
* **-l interface** changes the default network interface (eth0) the daemon will bind to
* **-h** prints a help message
