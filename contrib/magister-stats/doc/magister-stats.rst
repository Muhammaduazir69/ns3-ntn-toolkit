Magister Statistics Module Documentation
----------------------------------------

.. heading hierarchy:
   ------------- Chapter
   ************* Section (#.#)
   ============= Subsection (#.#.#)
   ############# Paragraph (no number)

Magister Statistics module provides statistics model additions to default stats
module of NS-3. The models are mainly used for statistics of the Satellite module,
but by deploying the provided generic statistics helpers, one can easily use the
helpers outside of SNS3.

Aggregators
***********

MultiFileAggregator
===================

This aggregator sends values it receives to one or more files.

Input
~~~~~

This class provides 10 methods for receiving input values in `double`. Each
of these methods is a function with a signature similar to the following:

::

  void WritePd (std::string context, double v1, double v2, ... double vP);

where `P` is a number between 1 and 10. In addition, the class provides the
method WriteString() which accepts a string input. These input methods
usually act as trace sinks of output from collectors' trace sources.

Output
~~~~~~

Each invocation to the input methods described above will produce a single
line of output. The `double` input arguments will be printed using the
formatting type selected using SetFileType() method or `FileType` attribute.
The `string` argument, on the other hand, will be printed as it is.

The first argument of each of the above mentioned input methods is a short
string indicating the context of the input sample. When the `MultiFileMode`
attribute is enabled (the default), this aggregator will create an
individual file for each unique context value, and then send each input
sample to the corresponding file.

When the `EnableContextPrinting` attribute is enabled (disabled by default),
each output line will begin with the context string and then a space. This
style is useful to determine the context of different data when all the
contexts are mixed together in one file, i.e., when `MultiFileMode` is
disabled.

The name of every file created begins with the value of the `OutputFileName`
attribute, and then followed by the context string. Finally, a ".txt"
extension is added at the end.

Note that all outputs are stored internally in string buffers. Upon destruction,
e.g., at the end of simulation, the whole buffer content is written to
the destination files.

Examples
~~~~~~~~

Statistics helper classes derived from StatsHelper utilize MultiFileAggregator
to produce separate statistics files for each identifier. User does not necessarily
need to handle MultiFileaggregator, but rather let the helpers do the complicated tasks.

Collectors
**********

DistributionCollector
=====================

Collector which computes the value distribution of the input samples.

Input
~~~~~

This class provides 9 trace sinks for receiving inputs. Each trace sink
is a function with a signature similar to the following:

::

  void TraceSinkP (P oldData, P newData);

where `P` is one of the 9 supported data types. This type of signature
follows the trace source signature types commonly exported by probes.
Although different data types are accepted, they are all internally
processed using `double` data type.

Processing
~~~~~~~~~~

This class begins by setting up a set of _bins_. Each bin covers an equal
length of input value range which does not overlap with the range of other
bins. Each received input sample is categorized into exactly one of these
bins. In this case, that bin's counter is increased by one. At the end of
the simulation, the bins would represent the distribution information of all
the received samples.

The setup of the bins can be configured through the `NumOfBins` attribute.
The corresponding method SetNumOfBins() can also be used for the same
purpose.

The class will compute a prediction of range of the bins based on the first
1000 samples received (see ns3::AdaptiveBins). Any subsequent samples which
do not fit into the predicted range are handled as follows.

- Input values which are less than lower bound of the first bin are
  categorized into the first bin.
- Input values which are equal or greater than upper bound of the last bin
  are categorized into the last bin.

Output
~~~~~~

At the end of the instance's life (e.g., when the simulation ends), the
`Output` trace source is fired, typically several times in a row, to export
the output. Each time the trace source is fired, it contains the bin
identifier (i.e., the center value of the bin) and the corresponding value
of that bin. The bin value is determined by the selected output type, which
can be modified by calling the SetOutputType() method or setting the
`OutputType` attribute. The burst of output is guaranteed to be in order
from the first bin (the lowest identifier) until the last bin.

In addition, the class also computes several statistical information and
export them as output trace sources.

- `OutputCount`
- `OutputSum`
- `OutputMin`
- `OutputMax`
- `OutputMean`
- `OutputStddev`
- `OutputVariance`
- `OutputSqrSum`

Finally, when the OUTPUT_TYPE_CUMULATIVE is selected as the output type, the
class also includes percentile information in the following trace sources.

- `Output5thPercentile`
- `Output25thPercentile`
- `Output50thPercentile`
- `Output75thPercentile`
- `Output95thPercentile`

Note that linear interpolation is used to calculate these percentile
information, and thus might have some errors.

All the additional statistical and percentile trace sources mentioned above
are also emitted in string format through the `OutputString` trace source.
The resulting string also includes the parameters used to collect the
samples. Example `OutputString` output:

::

  % min_value: 0
  % max_value: 1
  % bin_length: 0.02
  % num_of_bins: 50
  % output_type: 'OUTPUT_TYPE_CUMULATIVE'
  % count: 9
  % sum: 4.40882
  % min: 0.258985
  % max: 1.29714
  % mean: 0.489869
  % stddev: 0.457671
  % variance: 0.209463
  % sqr_sum: 3.83545
  % percentile_5: 0.2315
  % percentile_25: 0.2375
  % percentile_50: 0.245
  % percentile_75: 0.265
  % percentile_95: 0.9855


IntervalRateCollector
=====================

Collector which partitions the simulation into fixed length time
intervals and produce the sum of input sample data during each
interval as output.

Input
~~~~~

This class provides 5 trace sinks for receiving inputs. Each trace sink
is a function with a signature similar to the following:

::

  void TraceSinkP (P oldData, P newData);

where `P` is one of the 5 supported data types. This type of signature
follows the trace source signature types commonly exported by probes. The
input data is processed using either `double` (the default) or `uint64_t`
data types, depending on the input data type selected by calling the
SetInputDataType() method or setting the `InputDataType` attribute.

Processing
~~~~~~~~~~

Upon created, this class instance begins an interval. It lasts for a fixed
time duration (one second by default) that can be specified by calling the
SetIntervalLength() method or setting the `IntervalLength` attribute. The
instance accumulates the received input during the interval into a summed
value. Then at the end of each interval, the summed value is emitted as
output and reset back to zero. For boolean data type, a `true` value is
regarded as 1, while a `false` value is regarded as 0.

Output
~~~~~~

Samples received are *consolidated* using one of 3 available ways (e.g., sum,
count, average). It can be selected by calling the SetOutputType() method or
setting the `OutputType` attribute.

After that, this class utilizes 3 trace sources to export the output:

- `OutputWithoutTime`: the consolidated value from an interval, emitted at
  the end of every interval.
- `OutputWithTime`: the interval's ending time and its consolidated value,
  emitted at the end of every interval.
- `OutputOverall`: the consolidated value from all intervals, emitted when
  the instance is destroyed.

The consolidated values are exported in `double` data type in the same unit
as the inputs. The time information is exported in unit of seconds by
default, or as specified otherwise by calling the SetTimeUnit() method or
setting the `TimeUnit` attribute.

In addition, the class also exports the total number of input samples
received during the simulation as the `OutputCount` trace source. The same
information is also available from the `OutputString` trace source, which
has an output similar to the following:

::

  % output_type: 'OUTPUT_TYPE_SUM'
  % count: 57
  % sum: 672.72

These two trace sources are exported at the end of simulation.

ScalarCollector
===============

Collector which sums all the input data and emits the sum as a single
scalar output value.

Input
~~~~~

This class provides 5 trace sinks for receiving inputs. Each trace sink
is a function with a signature similar to the following:

::

  void TraceSinkP (P oldData, P newData);

where `P` is one of the 5 supported data types. This type of signature
follows the trace source signature types commonly exported by probes. The
input data is processed using either `double` (the default) or `uint64_t`
data types, depending on the input data type selected by calling the
SetInputDataType() method or setting the `InputDataType` attribute.

Processing
~~~~~~~~~~

This class sums all the received input values. The operation utilized to sum
those values is by default a simple addition operation. Additional operation,
such as averaging, may be specified by calling the the SetOutputType() method
or setting the `OutputType` attribute. For boolean data type, a `true` value
is regarded as 1, while a `false` value is regarded as 0.

Output
~~~~~~

At the end of the instance's life (e.g., when the simulation ends), the
`Output` trace source is fired to export the output. It contains a single
value in `double` type carrying the sum accumulated during the simulation.


UnitConversionCollector
=======================

Collector which converts input sample data to a different unit.

Input
~~~~~

This class provides 9 trace sinks for receiving inputs. Each trace sink
is a function with a signature similar to the following:

::

  void TraceSinkP (P oldData, P newData);

where `P` is one of the 9 supported data types. This type of signature
follows the trace source signature types commonly exported by probes.
Although different data types are accepted, they are all internally
processed using `double` data type.

Processing
~~~~~~~~~~

This class provides 7 types of unit conversion procedure. It can be selected
by calling the SetConversionType() method or setting the `ConversionType`
attribute.

- `TRANSPARENT` (no conversion at all)
- `FROM_BYTES_TO_BIT`
- `FROM_BYTES_TO_KBIT`
- `FROM_BYTES_TO_MBIT`
- `FROM_SECONDS_TO_MS`
- `FROM_LINEAR_TO_DB`
- `FROM_LINEAR_TO_DBM`

Output
~~~~~~

This class utilizes 3 trace sources to export the converted data:

- `Output`: the converted old data and the converted new data (similar
  signature as probe's trace source, hence can be used to export to another
  collector).
- `OutputValue`: the converted new data.
- `OutputTimeValue`: the current simulation time and the converted new data.

All of the above information are exported using `double` data type in the
unit specified by the selected conversion type. An exception here is in the
trace source `OutputTimeValue` where the time information is exported in
unit of seconds by default, or as specified otherwise by calling the
SetTimeUnit() method or setting the `TimeUnit` attribute.

Data Collection Helpers
***********************

StatsHelper classes
===================

Overview
########

StatsHelper is a base class for DCF-based statistics helper classes.
Helper classes based on it are responsible of locating source objects,
create probes, collectors, and aggregators, and connect them together
in a proper way to produce the required statistics.

The main inputs for the helpers are a
name, an identifier type, an output type and nodes which are monitored.
After all the necessary inputs have been set, the statistics
can be started into action by invoking Install(). For example:

::

  NodeContainer nodes;
  nodes.Create (2);

  // ... (snip) ...

  Ptr<StatsAppThroughputHelper> stat = CreateObject<StatsAppThroughputHelper> ();
  stat->SetName ("name");
  stat->SetIdentifierType (StatsHelper::IDENTIFIER_GLOBAL);
  stat->SetOutputType (StatsHelper::OUTPUT_SCALAR_FILE);
  stat->InstallNodes (nodes);
  stat->Install ();

  // ... (run simulation) ...

  stat->Dispose (); // Disposing of helper creates output files


This parent abstract class hosts several protected methods which are intended
to simplify the development of child classes. These methods handle
tasks related to DCF components.

Output path and file naming
~~~~~~~~~~~~~~~~~~~~~~~~~~~

To select the output directory for the statistics files, StatsHelper has an
OutputPath attribute. By default, its value is ns-3-root-directory/output/.
File names are based on helper name: if we create a helper

::

  Ptr<StatsAppThroughputHelper> stat = CreateObject<StatsAppThroughputHelper> ();
  stat->SetName ("throughput_by_node_scatter");
  stat->SetIdentifierType (StatsHelper::IDENTIFIER_NODE);
  stat->SetOutputType (StatsHelper::OUTPUT_SCATTER_FILE);

then the output files of this helper are named throughput_by_node_scatter.txt.
Note that there may be other file extensions, or node ID suffixes in the file name
based on output type.

Identifier types
~~~~~~~~~~~~~~~~

By default, there are two identifier types available: IDENTIFIER_GLOBAL and
IDENTIFIER_NODE. IDENTIFIER_GLOBAL causes statistics from all installed nodes
to be combined under single global identifier ("0" as id). IDENTIFIER_NODE will
separate statistics of all installed nodes under different IDs (node IDs).
More identifiers may naturally be created, but this will require some other
type of identifier ID creation in DoInstallProbes method of inherited helper
classes.

Output types
~~~~~~~~~~~~

There are several output types available:

* OUTPUT_SCALAR_FILE
* OUTPUT_SCATTER_FILE
* OUTPUT_HISTOGRAM_FILE
* OUTPUT_PDF_FILE
* OUTPUT_CDF_FILE
* OUTPUT_SCALAR_PLOT
* OUTPUT_SCATTER_PLOT
* OUTPUT_HISTOGRAM_PLOT
* OUTPUT_PDF_PLOT
* OUTPUT_CDF_PLOT

Output types with FILE suffix produce a generic text file or PDf file
based on collected statistics. Types with PLOT suffix generate Gnuplot
data files and scripts for automatically generating PNG figures with Gnuplot.

Scatter output types produce files, which have a list of time values and corresponding
statistics values. Scalar output types calculate single average value for each identifier,
e.g. in StatsAppThroughputHelper
a scalar file would contain sum of transmitted application layer bytes divided by time
per identifier.

Note that depending on the inherited helper class, not all output types may
be supported. In some cases, it is not even meaningful to enable all formats.


StatsAppDelayHelper
###################

StatsAppDelayHelper produces application layer delay statistics from nodes
installed to it. It is mainly an example of using the StatsHelper as a template
for statistics collection.

StatsAppDelayHelper collects delay information from installed nodes by adding
TrafficTimeTags as byte tags to sent packets. It does this by hooking into
Rx and Tx trace sources of applications installed into nodes.
Once a packet with a tag is received
by other application, the timestamp from the tag is removed and difference between
send and receive times is registered as delay.

Installing StatsAppDelayHelper is straightforward:

::

  Ptr<StatsAppDelayHelper> delayCdfByNode = CreateObject<StatsAppDelayHelper> ();
  delayCdfByNode->SetName ("stat-app-delay-scatter-node");
  delayCdfByNode->SetIdentifierType (StatsHelper::IDENTIFIER_NODE);
  delayCdfByNode->SetOutputType (StatsHelper::OUTPUT_CDF_FILE);
  delayCdfByNode->InstallNodes (nodes);
  delayCdfByNode->Install ();

  // Run the simulation

  delayCdfByNode->Dispose ();

This type of configuration causes helper to produce cumulative distribution function of the delay
values placed in text files with node id suffixes.

StatsAppThroughputHelper
########################

StatsAppThroughputHelper produces application layer throughput statistics from nodes
installed to it. As StatsAppDelayHelper, it is mainly an example of using the StatsHelper
as a template for statistics collection.

StatsAppThroughputHelper collects throughput statistics by creating ApplicationPacketProbes and
hooking them into Rx trace sources of any applications in target nodes.
Once a packet is received by application, the size of the packet is saved for later use.
The packet sizes and timing are use to calculate the data rate of received packets.

Installing StatsAppThroughputHelper is straightforward:

::

  Ptr<StatsAppThroughputHelper> throughputScatterByNode = CreateObject<StatsAppThroughputHelper> ();
  throughputScatterByNode->SetName ("stat-app-delay-scatter-node");
  throughputScatterByNode->SetIdentifierType (StatsHelper::IDENTIFIER_NODE);
  throughputScatterByNode->SetOutputType (StatsHelper::OUTPUT_SCATTER_FILE);
  throughputScatterByNode->InstallNodes (nodes);
  throughputScatterByNode->Install ();

  // Run the simulation

  throughputScatterByNode->Dispose ();

This type of configuration causes helper to produce scatter values, i.e. tuples of time and throughput on
preceding interval, listed in a separate text file for each node.

Note that some of the StatsAppThroughputHelper configuration modes require averaging mode to be set on.
This can be done by configuring

::

  throughputScatterByNode->SetAveragingMode (true);
