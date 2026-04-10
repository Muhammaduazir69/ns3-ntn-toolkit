.. include:: replace.txt

3GPP HTTP applications
----------------------

Model Description
*****************

The model is a part of the applications library. The HTTP model is based on a commonly
used 3GPP model in standardization `[4]`_.

Design
======

This traffic generator simulates web browsing traffic using the Hypertext
Transfer Protocol (HTTP). It consists of one or more ``ThreeGppHttpClient``
applications which connect to a ``ThreeGppHttpServer`` application. The client
models a web browser which requests web pages to the server. The server
is then responsible to serve the web pages as requested. Please refer to
``ThreeGppHttpClientHelper`` and ``ThreeGppHttpServerHelper`` for usage instructions.

Technically speaking, the client transmits *request objects* to demand a
service from the server. Depending on the type of request received, the
server transmits either:

  - a *main object*, i.e., the HTML file of the web page; or
  - an *embedded object*, e.g., an image referenced by the HTML file.

The main and embedded object sizes are illustrated in figures :ref:`fig-http-main-object-size`
and :ref:`fig-http-embedded-object-size`.


.. _fig-http-main-object-size:

.. figure:: figures/http-main-object-size.*
   :figwidth: 15cm

   3GPP HTTP main object size histogram

.. _fig-http-embedded-object-size:

.. figure:: figures/http-embedded-object-size.*
   :figwidth: 15cm

   3GPP HTTP embedded object size histogram

\

A major portion of the traffic pattern is *reading time*, which does not
generate any traffic. Because of this, one may need to simulate a good
number of clients and/or sufficiently long simulation duration in order to
generate any significant traffic in the system. Reading time is illustrated in
:ref:`fig-http-reading-time`.

.. _fig-http-reading-time:

.. figure:: figures/http-reading-time.*
   :figwidth: 15cm

   3GPP HTTP reading time histogram


3GPP HTTP server description
############################

3GPP HTTP server is a model application which simulates the traffic of a web server. This
application works in conjunction with ``ThreeGppHttpClient`` applications.

The application works by responding to requests. Each request is a small
packet of data which contains ``ThreeGppHttpHeader``. The value of the *content type*
field of the header determines the type of object that the client is
requesting. The possible type is either a *main object* or an *embedded object*.

The application is responsible to generate the right type of object and send
it back to the client. The size of each object to be sent is randomly
determined (see ``ThreeGppHttpVariables``). Each object may be sent as multiple packets
due to limited socket buffer space.

To assist with the transmission, the application maintains several instances
of ``ThreeGppHttpServerTxBuffer``. Each instance keeps track of the object type to be
served and the number of bytes left to be sent.

The application accepts connection request from clients. Every connection is
kept open until the client disconnects.

Maximum transmission unit (MTU) size is configurable in ``ThreeGppHttpServer`` or in
``ThreeGppHttpVariables``. By default, the low variant is 536 bytes and high variant is 1460 bytes.
The default values are set with the intention of having a TCP header (size of which is 40 bytes) added
in the packet in such way that lower layers can avoid splitting packets. The change of MTU sizes
affects all TCP sockets after the server application has started. It is mainly visible in sizes of
packets received by ``ThreeGppHttpClient`` applications.

3GPP HTTP client description
############################

3GPP HTTP client is a model application which simulates the traffic of a web browser. This
application works in conjunction with an ThreeGppHttpServer application.

In summary, the application works as follows.

1. Upon start, it opens a connection to the destination web server
   (ThreeGppHttpServer).
2. After the connection is established, the application immediately requests
   a *main object* from the server by sending a request packet.
3. After receiving a main object (which can take some time if it consists of
   several packets), the application "parses" the main object. Parsing time
   is illustrated in figure :ref:`fig-http-parsing-time`.
4. The parsing takes a short time (randomly determined) to determine the
   number of *embedded objects* (also randomly determined) in the web page.
   Number of embedded object is illustrated in :ref:`fig-http-num-of-embedded-objects`.
    - If at least one embedded object is determined, the application requests
      the first embedded object from the server. The request for the next
      embedded object follows after the previous embedded object has been
      completely received.
    - If there is no more embedded object to request, the application enters
      the *reading time*.
5. Reading time is a long delay (again, randomly determined) where the
   application does not induce any network traffic, thus simulating the user
   reading the downloaded web page.
6. After the reading time is finished, the process repeats to step #2.

.. _fig-http-parsing-time:

.. figure:: figures/http-parsing-time.*
   :figwidth: 15cm

   3GPP HTTP parsing time histogram

.. _fig-http-num-of-embedded-objects:

.. figure:: figures/http-num-of-embedded-objects.*
   :figwidth: 15cm

   3GPP HTTP number of embedded objects histogram

The client models HTTP *persistent connection*, i.e., HTTP 1.1, where the
connection to the server is maintained and used for transmitting and receiving
all objects.

Each request by default has a constant size of 350 bytes. A ``ThreeGppHttpHeader``
is attached to each request packet. The header contains information
such as the content type requested (either main object or embedded object)
and the timestamp when the packet is transmitted (which will be used to
compute the delay and RTT of the packet).


References
==========

Many aspects of the traffic are randomly determined by ``ThreeGppHttpVariables``.
A separate instance of this object is used by the HTTP server and client applications.
These characteristics are based on a legacy 3GPP specification. The description
can be found in the following references:

\

.. _`[1]`:

[1] 3GPP TR 25.892, "Feasibility Study for Orthogonal Frequency Division Multiplexing (OFDM) for UTRAN enhancement"

\

.. _`[2]`:

[2] IEEE 802.16m, "Evaluation Methodology Document (EMD)", IEEE 802.16m-08/004r5, July 2008.

\

.. _`[3]`:

[3] NGMN Alliance, "NGMN Radio Access Performance Evaluation Methodology", v1.0, January 2008.

\

.. _`[4]`:

[4] 3GPP2-TSGC5, "HTTP, FTP and TCP models for 1xEV-DV simulations", 2001.

\

Usage
*****

The three-gpp-http-example can be referenced to see basic usage of the HTTP applications.
In summary, using the ``ThreeGppHttpServerHelper`` and ``ThreeGppHttpClientHelper`` allow the
user to easily install ``ThreeGppHttpServer`` and ``ThreeGppHttpClient`` applications to nodes.
The helper objects can be used to configure attribute values for the client
and server objects, but not for the ``ThreeGppHttpVariables`` object. Configuration of variables
is done by modifying attributes of ``ThreeGppHttpVariables``, which should be done prior to helpers
installing applications to nodes.

The client and server provide a number of ns-3 trace sources such as
"Tx", "Rx", "RxDelay", and "StateTransition" on the server side, and a large
number on the client side ("ConnectionEstablished",
"ConnectionClosed","TxMainObjectRequest", "TxEmbeddedObjectRequest",
"RxMainObjectPacket", "RxMainObject", "RxEmbeddedObjectPacket",
"RxEmbeddedObject", "Rx", "RxDelay", "RxRtt", "StateTransition").


Building the 3GPP HTTP applications
===================================

Building the applications does not require any special steps to be taken. It suffices to enable
the applications module.

Examples
========

For an example demonstrating HTTP applications
run::

  $ ./waf --run 'three-gpp-http-example'

By default, the example will print out the web page requests of the client and responses of the
server and client receiving content packets by using LOG_INFO of ``ThreeGppHttpServer`` and ``ThreeGppHttpClient``.

Tests
=====

For testing HTTP applications, three-gpp-http-client-server-test is provided. Run::

  $ ./test.py -s three-gpp-http-client-server-test

The test consists of simple Internet nodes having HTTP server and client applications installed.
Multiple variant scenarios are tested: delay is 3ms, 30ms or 300ms, bit error rate 0 or 5.0*10^(-6),
MTU size 536 or 1460 bytes and either IPV4 or IPV6 is used. A simulation with each combination of
these parameters is run multiple times to verify functionality with different random variables.

Test cases themselves are rather simple: test verifies that HTTP object packet bytes sent match
total bytes received by the client, and that ``ThreeGppHttpHeader`` matches the expected packet.


NRTV (Near Real-Time Video) applications
----------------------------------------

Model Description
*****************

The model is a part of the applications library. The NRTV model is based on a 3GPP model in standardization `[7]`_.

Design
======

This traffic generator simulates NRTV traffic using either TCP or UDP on transport
layer. When streamed over TCP, the model consists of one or more ``NrtvTcpClient``
applications which connect to a ``NrtvTcpServer`` application, and over UDP
a slightly different ``NrtvUdpServer`` with ``PacketSink`` as receiver
application is used. The client
models a video client application which connects to a video server. The server
starts video workers (``NrtvVideoWorker``) for each socket created for connected clients.

Video workers create packets modeling video traffic according to NRTV specifications `[7]`_:
First, a video length in frames is decided. The workers attempt
to deliver video frames according to a configured frame rate to the client.
Each frame is then split into slices, which can be delivered each in their own packets.
Between generating slices and frame, encoding delays are applied.

To take things even further, each slice packet carries an overhead caused by a 24-byte header
containing information about the slice: the frame it belongs to, the number of slices the frame is
split to, and the index of the slice. To confirm that the entire slice is delivered in received packets,
the header also contains the length of slice. Each of these slice characteristics are handled
in more detail by ``NrtvTcpClient``, since TCP may split and reassemble packets. To be more
precise, ``NrtvTcpClient`` implements an Rx buffer for the slice packets and parses slices
from the buffer once they are delivered. An example of traffic received by a single client is illustrated in
:ref:`fig-nrtv-client-trace`.

.. _fig-nrtv-client-trace:

.. figure:: figures/nrtv-client-trace.*
   :figwidth: 15cm

   NRTV client traffic received with default configuration


NRTV Server Description
#######################

NRTV servers are model applications which simulate the traffic of a NRTV video server. These
applications work in conjunction with NRTV client applications:
``NrtvTcpServer`` with ``NrtvTcpClient`` and ``NrtvUdpServer`` with ``PacketSink``.
Packet sink is used as client application over UDP since there is no connection
between server and client in UDP.

``NrtvTcpServer`` works by responding to connecting ``NrtvTcpClient`` applications:
an ``NrtvVideoWorker`` instance is created to stream video to the client.
Once client disconnects or the video has ended, the socket will be closed
and video worker removed.

``NrtvUdpServer``  is connected to a ``PacketSink`` client application by
manually calling ``AddClient ()`` method of the server application. The method
requires the remote address of the node to which a ``PacketSink`` is installed.
It is also possible to decide how many videos are streamed to the client by
input parameters of the same method. After the videos have each ended, the socket
and corresponding video worker will be closed. Note that there is no way of knowing
if a client application has stopped before the videos have ended, and thus
the server will keep sending video slice packets until the end.

To configure the applications more efficiently, it is recommended to use
``NrtvHelper``.


NRTV Client Description
#######################

NRTV clients are model applications which simulate the video receiver applications.
``NrtvTcpClient`` is used over TCP and a generic ``PacketSink`` over UDP.

Unlike ``PacketSink``, the ``NrtvTcpClient`` is a more active receiver: it requests
a TCP connection to the server and keeps the connection alive until application is
either stopped or the video stream has ended. The latter is done by the server:
When the server terminates the connection, the application regards it as the
end of a video session. At this point, the application enters the IDLE state,
which is a randomly determined delay that simulates the user "resting"
between videos (e.g., commenting or picking the next video) - this is illustrated in
:ref:`fig-nrtv-idle-time`. After the IDLE
timer expires, the application restarts again by sending another connection
request.

.. _fig-nrtv-idle-time:

.. figure:: figures/nrtv-idle-time.*
   :figwidth: 15cm

   NRTV TCP client idle time histogram

NRTV Video Worker
#################

The NRTV video worker represents a single video session and its transmission:
It works under an NRTV server application and handles video traffic generation.
The worker will determine the length of video using `` NrtvVariables`` class.
Other variables are also retrieved from this class, such as number of
frames per second (frame rate) and number of slices per frame. The average
number of frames is illustrated in :ref:`fig-nrtv-num-of-frames`.

.. _fig-nrtv-num-of-frames:

.. figure:: figures/nrtv-num-of-frames.*
   :figwidth: 15cm

   NRTV number of frames per video histogram

The first video frame starts once the server has given a permission to do so
via ``ChangeState ()`` public method. Each frame has a fixed number of
slices, and each slice is preceded by a random length of encoding delay.
Each slice constitutes a single packet, which size is also determined
randomly. Each packet begins with a 24-byte ``NrtvHeader``. Slice sizes
and encoding delays with default attribute values can be seen in figures
:ref:`fig-nrtv-slice-size` and :ref:`fig-nrtv-slice-encoding-delay`.

.. _fig-nrtv-slice-size:

.. figure:: figures/nrtv-slice-size.*
   :figwidth: 15cm

   NRTV slice size histogram

.. _fig-nrtv-slice-encoding-delay:

.. figure:: figures/nrtv-slice-encoding-delay.*
   :figwidth: 15cm

   NRTV slice encoding delay histogram

Each frame always abides to the given frame rate, i.e., the start of each
frame is always punctual according to the frame rate. If the transmission
of the slices takes longer than the length of a single frame, then the
remaining unsent slices would be discarded, without postponing the start
time of the next frame.

Each slice sent will invoke the callback function specified using
``SetTxCallback ()``. After all the frames have been transmitted, another
callback function, specified using ``SetVideoCompletedCallback()``, will be
invoked.

.. _fig-nrtv-slice-size:

.. figure:: figures/nrtv-slice-size.*
   :figwidth: 15cm

   NRTV slice size histogram

NRTV Variables
##############

``NrtvVariables`` is a container class of various random variables for assisting
the generation of streaming traffic pattern by the Near Real-Time Video (NRTV)
traffic model. A separate instance of this object is used by all the NRTV objects
(e.g. servers, TCP client, video worker).

The default configuration of some of the underlying random distributions are
according to NGMN `[5]`_ and WiMAX `[6]`_ specifications.

The available random values to be retrieved are:

- number of frames per video --- truncated LogNormal distribution with mean
  of 3000 frames (i.e., 5 minutes of 10 fps video);
- frame interval --- constant 100 ms (i.e., 10 fps);
- number of slices per frame --- constant 8 slices (packets);
- slice size --- truncated Pareto distribution with mean of approximately
  82.64 bytes;
- slice encoding delay --- truncated Pareto distribution with mean of
  approximately 5.31 ms;
- client's de-jitter buffer window size --- constant 5 seconds; and
- client's idle time --- unbounded exponential distribution with mean of
  5 seconds.

Most parameters of the random distributions are configurable via attributes
and methods of this class.

References
==========

\

.. _`[5]`:

[5] NGMN Alliance, "NGMN Radio Access Performance Evaluation Methodology", v1.0, January 2008.

\

.. _`[6]`:

[6] WiMAX Forum, "WiMAX (TM) System Evaluation Methodology", Version 2.1, July 2008.

\

.. _`[7]`:

[7] 3rd Generation Partnership Project (3GPP), "TR 25.892: Feasibility Study for OFDM for UTRAN enhancemen", V2.0.0, June 2004

\

Usage
*****

The nrtv-p2p-example can be referenced to see basic usage of the NRTV applications.
In summary, using the ``NrtvHelper`` allows the
user to easily install ``NrtvTcpServer`` and ``NrtvTcpClient`` or ``NrtvUdpServer`` and ``PacketSink``
applications to nodes, depending on the protocol used.
The helper object can be used to configure attribute values for the client
and server objects via ``SetClientAttribute ()`` and ``SetClientAttribute ()``  methods - note that
it is most efficient to configure these before installing applications by calling ``InstallUsingIpv4 ()``
on server and client nodes.
Configuration of variables is done by modifying attributes of ``NrtvVariables``,
which should be done prior to helpers installing applications to nodes.

The client and server provide a number of ns-3 trace sources such as
"Tx", "StateTransition" on the server side, and depending on the protocol some
on the client side number on the client side: TCP client offers "Rx", "RxDelay","RxSlice",
"RxFrame", and "StateTransition" trace sources, while currently Packet Sink, which is used
as a UDP client, offers only "Rx".

Building the NRTV applications
==============================

Building the applications does not require any special steps to be taken. It suffices to enable
the applications module.

Examples
========

For an example demonstrating NRTV applications
run::

  $ ./waf --run 'nrtv-p2p-example'

By default, the example will run a point-to-point scenario of NRTV server and client using either TCP or UDP,
depending on the input arguments.
In the end, time trace of the received bytes by client is given as a Gnuplot script. Running::

  $ gnuplot NRTV-TCP-client-trace.plt

will print out a PNG file of the trace.

Another example using the same plot mechanism to demonstrate the default values given by NRTV variables is
nrtv-variables-plot. Run it by::

  $ ./waf --run 'nrtv-variables-plot'

and use gnuplot on the output plt files to get the PNG files.

Tests
=====

For testing NRTV applications, nrtv-test is provided. Run::

  $ ./test.py -s nrtv

The test consists of simple Internet nodes having NRTV server and client applications installed.
Currently, the test tries to run the server and client for 5 seconds using a delay of 3ms, 30ms or 300ms,
and either TCP or UDP. In TCP case, it is verified that slice packets can be parsed in a correct order
from the Rx buffer. In UDP case, since the error rate is 0, it is just tested that packets are sent and got in
correct order and their sizes are the same.



