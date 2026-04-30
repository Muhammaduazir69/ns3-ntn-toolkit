# -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

def build(bld):
    module = bld.create_ns3_module('traffic', ['core',
                                               'applications',
                                               'flow-monitor',
                                               'magister-stats',
                                               'point-to-point'])
    module.source = [
        'helper/cbr-helper.cc',
        'helper/client-rx-trace-plot.cc',
        'helper/nrtv-helper.cc',
        'helper/three-gpp-http-satellite-helper.cc',
        'model/cbr-application.cc',
        'model/nrtv-header.cc',
        'model/nrtv-tcp-client.cc',
        'model/nrtv-tcp-server.cc',
        'model/nrtv-udp-server.cc',
        'model/nrtv-variables.cc',
        'model/nrtv-video-worker.cc',
        'model/traffic-time-tag.cc',
        'model/three-gpp-http-satellite-client.cc',
        'stats/application-stats-helper.cc',
        'stats/application-stats-delay-helper.cc',
        'stats/application-stats-throughput-helper.cc',
        'stats/application-stats-helper-container.cc',
        ]

    module_test = bld.create_ns3_module_test_library('traffic')
    module_test.source = [
        'test/cbr-test.cc',    
        'test/nrtv-test.cc',
        ]

    headers = bld(features='ns3header')
    headers.module = 'traffic'
    headers.source = [
        'helper/cbr-helper.h',
        'helper/client-rx-trace-plot.h',
        'helper/histogram-plot-helper.h',
        'helper/nrtv-helper.h',
        'helper/three-gpp-http-satellite-helper.h',
        'model/traffic.h',
        'model/cbr-application.h',
        'model/nrtv-header.h',
        'model/nrtv-tcp-client.h',
        'model/nrtv-tcp-server.h',
        'model/nrtv-udp-server.h',
        'model/nrtv-variables.h',
        'model/nrtv-video-worker.h',
        'model/traffic-time-tag.h',
        'model/three-gpp-http-satellite-client.h',
        'stats/application-stats-helper.h',
        'stats/application-stats-delay-helper.h',
        'stats/application-stats-throughput-helper.h',
        'stats/application-stats-helper-container.h',
        ]

    if (bld.env['ENABLE_EXAMPLES']):
        bld.recurse('examples')
