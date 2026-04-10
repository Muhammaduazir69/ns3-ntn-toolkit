## -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

def build(bld):
    obj = bld.create_ns3_module('magister-stats',['core','network','stats','internet','applications'])
    obj.source = [
        'helper/stats-delay-helper.cc',
        'helper/stats-helper.cc',
        'helper/stats-throughput-helper.cc',
        'model/collector-map.cc',
        'model/distribution-collector.cc',
        'model/interval-rate-collector.cc',
        'model/address-boolean-probe.cc',
        'model/address-double-probe.cc',
        'model/address-tag.cc',
        'model/address-time-probe.cc',
        'model/address-uinteger-probe.cc',
        'model/application-delay-probe.cc',
        'model/bytes-probe.cc',
        'model/magister-gnuplot-aggregator.cc',
        'model/multi-file-aggregator.cc',
        'model/scalar-collector.cc',
        'model/time-tag.cc',
        'model/uinteger-32-single-probe.cc',
        'model/unit-conversion-collector.cc',
        ]

    module_test = bld.create_ns3_module_test_library('magister-stats')
    module_test.source = [
        'test/distribution-collector-test-suite.cc',
        ]

    headers = bld(features='ns3header')
    headers.module = 'magister-stats'
    headers.source = [
        'helper/stats-delay-helper.h',
        'helper/stats-helper.h',
        'helper/stats-throughput-helper.h',
        'model/collector-map.h',
        'model/distribution-collector.h',
        'model/interval-rate-collector.h',
        'model/address-boolean-probe.h',
        'model/address-double-probe.h',
        'model/address-tag.h',
        'model/address-time-probe.h',
        'model/address-uinteger-probe.h',
        'model/application-delay-probe.h',
        'model/bytes-probe.h',
        'model/magister-gnuplot-aggregator.h',
        'model/magister-stats.h',
        'model/multi-file-aggregator.h',
        'model/scalar-collector.h',
        'model/time-tag.h',
        'model/uinteger-32-single-probe.h',
        'model/unit-conversion-collector.h',
        ]
        
    if (bld.env['ENABLE_EXAMPLES']):
        bld.recurse('examples')
