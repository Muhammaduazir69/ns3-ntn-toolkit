#!/usr/bin/env python3
#
# Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
#
# SPDX-License-Identifier: GPL-2.0-only

"""
Run the ns-3 nr-external-core-roaming example against Open5GS through nr-bridge.

It starts the core container (Open5GS and nr-bridge), sets up its links with ns-3 (N3 and the
DN), its N6 NAT, and its network functions, runs the example from the ns-3 build of the host in
the network namespace of the container, reports the traffic of the UEs through N6, and then
stops and removes the containers, which removes their network settings.

Usage: run.py [--keep] [--compose CMD] [--packetrusher-src DIR] [-- example arguments]
"""

import argparse
import os
import shlex
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
NS3_DIR = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
PROJECT = "nr-open5gs-roaming"

# Network functions of the visited network, in start order
NETWORK_FUNCTIONS = ["nrf", "sepp", "ausf", "udm", "udr", "pcf", "bsf", "nssf", "smf", "upf", "amf"]

# Links with ns-3 and N6 of the core container. The EmuFdNetDevices of ns-3 receive the packets
# through raw sockets, so their checksums must be complete on the wire.
NETWORK_SETUP = """
set -e
# N3: the visited gNB of ns-3 has an EmuFdNetDevice on n3-ns3 (10.210.1.2)
ip link add n3-upf type veth peer name n3-ns3
ip addr add 10.210.1.1/24 dev n3-upf
ip link set n3-upf up
ip link set n3-ns3 promisc on up
# DN: a node of ns-3 has an EmuFdNetDevice on n6-ns3 (10.210.2.2)
ip link add n6-core type veth peer name n6-ns3
ip addr add 10.210.2.1/24 dev n6-core
ip link set n6-core up
ip link set n6-ns3 promisc on up
for dev in n3-upf n3-ns3 n6-core n6-ns3; do ethtool -K $dev tx off >/dev/null; done
# the PDU sessions of the UPF
ip tuntap add name ogstun2 mode tun
ip addr add 10.46.0.1/16 dev ogstun2
ip link set ogstun2 up
# N6: the UEs reach the internet through a NAT; 1.0.0.0/8 holds the remote hosts of the core
# networks of ns-3, not the internet hosts using those addresses. The FORWARD rules count the
# traffic of the UEs in each direction.
iptables -t nat -A POSTROUTING -s 10.46.0.0/16 ! -o ogstun2 ! -d 10.210.2.0/24 -j MASQUERADE
iptables -A FORWARD -i ogstun2 -d 1.0.0.0/8 -j REJECT
iptables -A FORWARD -i ogstun2 -s 10.46.0.0/16 -j ACCEPT
iptables -A FORWARD -o ogstun2 -d 10.46.0.0/16 -j ACCEPT
"""


def run(cmd, check=True, capture=False):
    """Run a command, printing it."""
    print("+", shlex.join(cmd), flush=True)
    return subprocess.run(cmd, check=check, text=True, capture_output=capture)


def is_podman(command):
    """Whether a container engine command is podman (possibly under the docker name)."""
    version = subprocess.run([command, "--version"], text=True, capture_output=True, check=False)
    return "podman" in version.stdout.lower()


def find_compose(requested):
    """
    Find a compose command and the container engine it drives.

    Without a requested command, it uses podman-compose or the docker compose plugin if
    available, and otherwise installs podman-compose in a virtual environment of the user cache.
    """
    if requested:
        compose = shlex.split(requested)
        engine = "podman" if "podman" in compose[0] or is_podman(compose[0]) else "docker"
        return compose, engine
    if shutil.which("podman-compose"):
        return ["podman-compose"], "podman"
    if shutil.which("docker"):
        plugin = subprocess.run(["docker", "compose", "version"], capture_output=True, check=False)
        if plugin.returncode == 0:
            return ["docker", "compose"], "podman" if is_podman("docker") else "docker"
    if not shutil.which("podman"):
        sys.exit("No compose command found: install podman-compose or the docker compose plugin")

    cache = os.environ.get("XDG_CACHE_HOME", os.path.expanduser("~/.cache"))
    venv = os.path.join(cache, "ns-3", "open5gs-roaming-venv")
    tool = os.path.join(venv, "bin", "podman-compose")
    if not os.access(tool, os.X_OK):
        print(f"Installing podman-compose in {venv}", flush=True)
        # the interpreter running this script may lack ensurepip (e.g., some Homebrew builds)
        for python in dict.fromkeys([sys.executable, "/usr/bin/python3", "python3"]):
            shutil.rmtree(venv, ignore_errors=True)
            if subprocess.run([python, "-m", "venv", venv], capture_output=True).returncode == 0:
                break
        else:
            sys.exit("Cannot create a virtual environment for podman-compose")
        run([os.path.join(venv, "bin", "pip"), "install", "-q", "podman-compose"])
    return [tool], "podman"


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[1])
    parser.add_argument("--keep", action="store_true", help="leave the containers running")
    parser.add_argument(
        "--compose",
        default=os.environ.get("COMPOSE"),
        help="compose command (default: podman-compose or docker compose if found, otherwise "
        "podman-compose installed in a virtual environment of the user cache)",
    )
    parser.add_argument(
        "--packetrusher-src",
        default=os.environ.get("PACKETRUSHER_SRC"),
        help="local PacketRusher checkout to build nr-bridge from (default: GitHub nr-bridge)",
    )
    parser.add_argument("--profile", default="default", help="ns-3 build profile")
    parser.add_argument("example_args", nargs="*", help="arguments of the example (after --)")
    args = parser.parse_args()

    binary = os.path.join(
        NS3_DIR,
        "build",
        "contrib",
        "nr",
        "examples",
        f"ns3-dev-nr-external-core-roaming-{args.profile}",
    )
    if not os.access(binary, os.X_OK):
        sys.exit("Build the example first: ./ns3 build nr-external-core-roaming")

    compose, engine = find_compose(args.compose)
    compose += ["-f", os.path.join(HERE, "docker-compose.yml")]
    podman = engine == "podman"
    # enter the network namespace of the container, with its capabilities
    enter = ["podman", "unshare", "nsenter"] if podman else ["sudo", "nsenter"]
    if args.packetrusher_src:
        os.environ["PACKETRUSHER_SRC"] = os.path.abspath(args.packetrusher_src)

    run(compose + ["build"])
    run(compose + ["up", "-d"])
    status = 1
    container = None
    try:
        container = run(
            [
                engine,
                "ps",
                "-q",
                "--filter",
                f"label=com.docker.compose.project={PROJECT}",
                "--filter",
                "label=com.docker.compose.service=core",
            ],
            capture=True,
        ).stdout.split()[0]

        def sh(script, detach=False, check=True, capture=False):
            return run(
                [engine, "exec"] + (["-d"] if detach else []) + [container, "bash", "-c", script],
                check=check,
                capture=capture,
            )

        sh(NETWORK_SETUP)
        for nf in NETWORK_FUNCTIONS:
            sh(
                f"exec open5gs-{nf}d -c /etc/open5gs/roaming.yaml >/var/log/open5gs-{nf}.log 2>&1",
                detach=True,
            )
            time.sleep(0.5)
        sh("exec nr-bridge -config /etc/nr-bridge.json >/var/log/nr-bridge.log 2>&1", detach=True)

        print("Waiting for the AMF and nr-bridge...", flush=True)
        for _ in range(60):
            ready = sh(
                "grep -q 'ngap_server()' /var/log/open5gs-amf.log && "
                "grep -q 'Waiting for ns-3' /var/log/nr-bridge.log",
                check=False,
                capture=True,
            )
            if ready.returncode == 0:
                break
            time.sleep(1)
        else:
            raise RuntimeError("the core network did not start")
        time.sleep(2)

        pid = run(
            [engine, "inspect", "-f", "{{.State.Pid}}", container], capture=True
        ).stdout.strip()
        status = run(
            enter + ["-t", pid, "-n", "--", binary] + args.example_args, check=False
        ).returncode

        print("UE traffic through N6 (NAT and forwarded packets):", flush=True)
        sh(
            "iptables -t nat -L POSTROUTING -v -n -x | grep -E 'pkts|10.46.0.0'; "
            "iptables -L FORWARD -v -n -x | grep -E 'pkts|10.46.0.0'",
            check=False,
        )
    finally:
        if container:
            log = os.path.join(HERE, "core.log")
            with open(log, "w") as f:
                subprocess.run(
                    [
                        engine,
                        "exec",
                        container,
                        "bash",
                        "-c",
                        "tail -n +1 /var/log/open5gs-*.log /var/log/nr-bridge.log",
                    ],
                    stdout=f,
                    stderr=subprocess.STDOUT,
                    text=True,
                )
            print(f"Core network log: {log}")
        if not args.keep:
            run(compose + ["down", "-v"], check=False)
    return status


if __name__ == "__main__":
    try:
        sys.exit(main())
    except subprocess.CalledProcessError as error:
        sys.exit(f"Failed ({error.returncode}): {shlex.join(error.cmd)}")
